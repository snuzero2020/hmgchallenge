#include <iostream>
#include <chrono>
#include <vector>
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
    vector<double> t;
    double ti;
    double lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
    double longitudinal_acceleration, longitudinal_jerk;
    FrenetPath* tfp;
    int num_paths = 0;
    int num_viable_paths = 0;
    double valid_path_time = 0;
    ti =0;
    while (ti <= 2*fot_hp->planning_t){
        t.push_back(ti);
        ti+=fot_hp->control_t;
    }
    while (ti <= fot_hp->mint){
        t.push_back(ti);
        ti+=fot_hp->dt;
    }
    while (ti <= fot_hp->maxt) {
        t.push_back(ti);
        vector<FrenetPath1D*> lateral_paths = quintic_paths_1D(
            t, fot_ic->c_d, fot_ic->c_d_d, fot_ic->c_d_dd, 
            -fot_hp->max_road_width_r, fot_hp->max_road_width_l, fot_hp->d_road_w, 
            0, 0, 0);
        
        // 속도 유지
        // double target_speed = fot_ic->target_speed;
        // double possible_max_speed = fot_ic->c_speed + (fot_hp->max_accel*2/3 + fot_ic->c_accel*1/6)*ti - fot_hp->d_t_s * fot_hp->n_s_sample;
        // double possible_min_speed = fot_ic->c_speed + (-fot_hp->max_break*2/3 + fot_ic->c_accel*1/6)*ti + fot_hp->d_t_s * fot_hp->n_s_sample;
        // if(target_speed > possible_max_speed){
        //     target_speed = possible_max_speed;
        // }
        // else if(target_speed < possible_min_speed){
        //     target_speed = possible_min_speed;
        // }
        // vector<FrenetPath1D*> longitudinal_paths = quartic_paths_1D(
        //     t, fot_ic->s0, fot_ic->c_speed, fot_ic->c_accel, 
        //     target_speed - fot_hp->d_t_s*fot_hp->n_s_sample, target_speed + fot_hp->d_t_s*fot_hp->n_s_sample, fot_hp->d_t_s, 
        //     fot_ic->target_speed, 0);
        
        // 점 추종
        double target_point = fot_ic->s0 + fot_ic->target_speed*ti; //일단 테스트 용
        vector<FrenetPath1D*> longitudinal_paths = quintic_paths_1D(
            t, fot_ic->s0, fot_ic->c_speed, fot_ic->c_accel, 
            target_point - fot_hp->max_ds, target_point + fot_hp->max_ds, fot_hp->ds, 
            target_point, fot_ic->target_speed, 0);

        for(FrenetPath1D* lateral_path : lateral_paths){
            for(FrenetPath1D* longitudinal_path : longitudinal_paths){
                tfp = new FrenetPath(fot_hp);
                tfp->t.assign(t.begin(),t.end());
                tfp->d.assign(lateral_path->y.begin(),lateral_path->y.end());
                tfp->d_d.assign(lateral_path->y_d.begin(),lateral_path->y_d.end());
                tfp->d_dd.assign(lateral_path->y_dd.begin(),lateral_path->y_dd.end());
                tfp->d_ddd.assign(lateral_path->y_ddd.begin(),lateral_path->y_ddd.end());
                tfp->s.assign(longitudinal_path->y.begin(),longitudinal_path->y.end());
                tfp->s_d.assign(longitudinal_path->y_d.begin(),longitudinal_path->y_d.end());
                tfp->s_dd.assign(longitudinal_path->y_dd.begin(),longitudinal_path->y_dd.end());
                tfp->s_ddd.assign(longitudinal_path->y_ddd.begin(),longitudinal_path->y_ddd.end());

                num_paths++;
                bool success = tfp->to_global_path(csp);
                if (!success) {
                    delete tfp;
                    continue;
                }
                bool valid_path = tfp->is_valid_path(obstacles);
                if (!valid_path) {
                    delete tfp;
                    continue;
                }
                num_viable_paths++;

                tfp->c_lateral_deviation = lateral_path->c_deviation;
                tfp->c_lateral_velocity = lateral_path->c_velocity;
                tfp->c_lateral_acceleration = lateral_path->c_accel;
                tfp->c_lateral_jerk = lateral_path->c_jerk;
                tfp->c_lateral = fot_hp->kd * tfp->c_lateral_deviation + fot_hp->kv * tfp->c_lateral_velocity + fot_hp->ka * tfp->c_lateral_acceleration + fot_hp->kj * tfp->c_lateral_jerk;

                tfp->c_longitudinal_jerk = longitudinal_path->c_jerk;
                tfp->c_longitudinal_end = longitudinal_path->c_end;
                tfp->c_longitudinal = fot_hp->kj * tfp->c_longitudinal_jerk + fot_hp->kd * tfp->c_longitudinal_end;

                tfp->c_time_taken = sq(ti + fot_hp->target_t*fot_hp->target_t/ti);
                tfp->c_inv_dist_to_obstacles = sq(tfp->inverse_distance_to_obstacles(obstacles));

                tfp->cf = fot_hp->klat*tfp->c_lateral + fot_hp->klon*tfp->c_longitudinal + fot_hp->kt*tfp->c_time_taken + fot_hp->ko*tfp->c_inv_dist_to_obstacles;
                frenet_paths.push_back(tfp);
            }
        }
        ti += fot_hp->dt;
    }
}


vector<FrenetPath1D*> FrenetOptimalTrajectory::quartic_paths_1D(vector<double> t, double y0, double dy0, double ddy0, double min_v, double max_v, double step_v, double target_v, double dvf){
    vector<FrenetPath1D*> fp1s;
    for(double tv = min_v; tv<=max_v; tv+=step_v){
        QuarticPolynomial qp = QuarticPolynomial(y0, dy0, ddy0, tv, dvf, t.back());
        FrenetPath1D* fp1 = new FrenetPath1D();
        fp1->t.assign(t.begin(),t.end());
        for (double tp : t) {
            fp1->y.push_back(qp.calc_point(tp));
            fp1->y_d.push_back(qp.calc_first_derivative(tp));
            fp1->y_dd.push_back(qp.calc_second_derivative(tp));
            fp1->y_ddd.push_back(qp.calc_third_derivative(tp));
            fp1->c_jerk += sq(qp.calc_third_derivative(tp));
        }
        fp1->c_end += sq( fp1->y_d.back() - target_v );
        fp1s.push_back(fp1);
    }    
    return fp1s;
}
vector<FrenetPath1D*> FrenetOptimalTrajectory::quintic_paths_1D(vector<double> t, double y0, double dy0, double ddy0, double min_y, double max_y, double step_y, double target_y, double dyf, double ddyf){
    vector<FrenetPath1D*> fp1s;
    for(double ty = min_y; ty<=max_y; ty+=step_y){
        QuinticPolynomial qp = QuinticPolynomial(y0, dy0, ddy0, ty, dyf, ddyf, t.back());
        FrenetPath1D* fp1 = new FrenetPath1D(); 
        fp1->t.assign(t.begin(),t.end());
        for (double tp : t) {
            fp1->y.push_back(qp.calc_point(tp));
            fp1->y_d.push_back(qp.calc_first_derivative(tp));
            fp1->y_dd.push_back(qp.calc_second_derivative(tp));
            fp1->y_ddd.push_back(qp.calc_third_derivative(tp));
            fp1->c_deviation += sq(qp.calc_point(tp));
            fp1->c_velocity += sq(qp.calc_first_derivative(tp));
            fp1->c_accel += sq(qp.calc_second_derivative(tp));
            fp1->c_jerk += sq(qp.calc_third_derivative(tp));
        }
        fp1->c_end += sq( fp1->y.back() - target_y );
        fp1s.push_back(fp1);
    }
    return fp1s;
}