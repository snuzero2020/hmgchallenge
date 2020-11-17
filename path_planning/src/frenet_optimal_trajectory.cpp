#include <iostream>
#include <chrono>
#include "frenet_optimal_trajectory.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "utils.h"
#include "ros/console.h"

using namespace std;

FrenetOptimalTrajectory::FrenetOptimalTrajectory(FrenetInitialConditions *fot_ic_, FrenetHyperparameters *fot_hp_, CubicSpline2D *csp_) {
    auto start = chrono::high_resolution_clock::now();
    
    fot_ic = fot_ic_;
    fot_hp = fot_hp_;
    csp = csp_;
    x.assign(fot_ic->wx, fot_ic->wx + fot_ic->nw);
    y.assign(fot_ic->wy, fot_ic->wy + fot_ic->nw);
    setObstacles();

    best_frenet_path = nullptr;

    if (x.size() < 2) {
        return;
    }

    //csp = new CubicSpline2D(x, y);
    
    calc_frenet_paths();

    double mincost = INFINITY;
    for (FrenetPath* fp : frenet_paths) {
        if (fp->cf <= mincost) {
            mincost = fp->cf;
            best_frenet_path = fp;
        }
    }
    if(best_frenet_path==nullptr){
        ROS_ERROR("No Path Error");
    }
    auto end = chrono::high_resolution_clock::now();
    double run_time = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
    run_time *= 1e-9;
    cout << "Planning runtime " << run_time << endl;
}

FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {
    //delete csp;
    for (FrenetPath* fp : frenet_paths) {
        delete fp;
    }

    for (Obstacle* ob : obstacles) {
        delete ob;
    }
}

CubicSpline2D* FrenetOptimalTrajectory::getcsp(){
    return csp;
}

FrenetPath* FrenetOptimalTrajectory::getBestPath() {
    return best_frenet_path;
}

void FrenetOptimalTrajectory::setObstacles() {
    vector<double> llx(fot_ic->o_llx, fot_ic->o_llx + fot_ic->no);
    vector<double> lly(fot_ic->o_lly, fot_ic->o_lly + fot_ic->no);
    vector<double> urx(fot_ic->o_urx, fot_ic->o_urx + fot_ic->no);
    vector<double> ury(fot_ic->o_ury, fot_ic->o_ury + fot_ic->no);

    for (int i = 0; i < fot_ic->no; i++) {
        addObstacle(Vector2f(llx[i], lly[i]), Vector2f(urx[i], ury[i]));
    }
}

void FrenetOptimalTrajectory::addObstacle(Vector2f first_point, Vector2f second_point) {
    obstacles.push_back(new Obstacle(std::move(first_point), std::move(second_point), fot_hp->obstacle_clearance));
}

void FrenetOptimalTrajectory::calc_frenet_paths() {
    double t, ti, tv;
    double lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
    double longitudinal_acceleration, longitudinal_jerk;
    FrenetPath* fp, *tfp;
    int num_paths = 0;
    int num_viable_paths = 0;
    double valid_path_time = 0;
    double di = -fot_hp->max_road_width_l;

    while (di <= fot_hp->max_road_width_r) {
        ti = fot_hp->mint;

        while (ti <= fot_hp->maxt) {
            lateral_deviation = 0;
            lateral_velocity = 0;
            lateral_acceleration = 0;
            lateral_jerk = 0;

            fp = new FrenetPath(fot_hp);
            QuinticPolynomial lat_qp = QuinticPolynomial(fot_ic->c_d, fot_ic->c_d_d, fot_ic->c_d_dd, di, 0.0, 0.0, ti);

            t = 0;
            while (t <= ti) {
                fp->t.push_back(t);
                fp->d.push_back(lat_qp.calc_point(t));
                fp->d_d.push_back(lat_qp.calc_first_derivative(t));
                fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
                lateral_deviation += abs(lat_qp.calc_point(t));
                lateral_velocity += abs(lat_qp.calc_first_derivative(t));
                lateral_acceleration += abs(lat_qp.calc_second_derivative(t));
                lateral_jerk += abs(lat_qp.calc_third_derivative(t));
                if(t<=2*fot_hp->planning_t){
                    t += fot_hp->control_t;
                }else {
                    t += fot_hp->dt - (ti-t)/(ti-2*fot_hp->planning_t)*(fot_hp->dt - fot_hp->control_t);
                }
            }

            //tv = fot_ic->target_speed - fot_hp->d_t_s * fot_hp->n_s_sample;
            tv = min(fot_ic->target_speed, (fot_ic->c_accel/6 + fot_hp->max_accel)*ti) - fot_hp->d_t_s * fot_hp->n_s_sample; 
            while (tv <= fot_ic->target_speed + fot_hp->d_t_s * fot_hp->n_s_sample) {
                longitudinal_acceleration = 0;
                longitudinal_jerk = 0;

                tfp = new FrenetPath(fot_hp);
                tfp->t.assign(fp->t.begin(), fp->t.end());
                tfp->d.assign(fp->d.begin(), fp->d.end());
                tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
                tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
                tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());
                QuarticPolynomial lon_qp = QuarticPolynomial(fot_ic->s0, fot_ic->c_speed, fot_ic->c_accel, tv, 0.0, ti);

                for (double tp : tfp->t) {
                    tfp->s.push_back(lon_qp.calc_point(tp));
                    tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));
                    tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp));
                    tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp));
                    longitudinal_acceleration += abs(lon_qp.calc_second_derivative(tp));
                    longitudinal_jerk += abs(lon_qp.calc_third_derivative(tp));
                }

                num_paths++;

                bool success = tfp->to_global_path(csp);
                num_viable_paths++;
                if (!success) {
                    delete tfp;
                    tv += fot_hp->d_t_s;
                    continue;
                }

                bool valid_path = tfp->is_valid_path(obstacles);

                if (!valid_path) {
                    delete tfp;
                    tv += fot_hp->d_t_s;
                    continue;
                }

                tfp->c_lateral_deviation = lateral_deviation;
                tfp->c_lateral_velocity = lateral_velocity;
                tfp->c_lateral_acceleration = lateral_acceleration;
                tfp->c_lateral_jerk = lateral_jerk;
                tfp->c_lateral = fot_hp->kd * tfp->c_lateral_deviation + fot_hp->kv * tfp->c_lateral_velocity + fot_hp->ka * tfp->c_lateral_acceleration + fot_hp->kj * tfp->c_lateral_jerk;

                tfp->c_longitudinal_acceleration = longitudinal_acceleration;
                tfp->c_longitudinal_jerk = longitudinal_jerk;
                tfp->c_end_speed_deviation = abs(fot_ic->target_speed - tfp->s_d.back());
                tfp->c_time_taken = ti + fot_hp->target_t*fot_hp->target_t/ti;
                tfp->c_longitudinal = fot_hp->ka * tfp->c_longitudinal_acceleration + fot_hp->kj * tfp->c_longitudinal_jerk + fot_hp->kt * tfp->c_time_taken + fot_hp->kd * tfp->c_end_speed_deviation;

                tfp->c_inv_dist_to_obstacles = tfp->inverse_distance_to_obstacles(obstacles);

                tfp->cf = fot_hp->klat * tfp->c_lateral + fot_hp->klon * tfp->c_longitudinal + fot_hp->ko * tfp->c_inv_dist_to_obstacles;

                frenet_paths.push_back(tfp);
                tv += fot_hp->d_t_s;
            }
            ti += fot_hp->dt;
            delete fp;
        }
        di += fot_hp->d_road_w;
    }
}