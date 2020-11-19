#include "frenet_path.h"
#include "utils.h"
#include "ros/console.h"

#include <math.h>
#include <algorithm>


const float COLLISION_CHECK_THRESHOLD = 0.1;

FrenetPath::FrenetPath(FrenetHyperparameters *fot_hp_) {
    fot_hp = fot_hp_;
}

bool FrenetPath::to_global_path(CubicSpline2D* csp) {
    double ix_, iy_, iyaw_, icurvature_, di, fx, fy, dx, dy, ddx, ddy;
    // calc global positions
    for (size_t i = 0; i < s.size(); i++) {
        SLState sls;
        sls.s=s[i];
        sls.l=d[i];
        sls.ds=s_d[i];
        sls.dl=d_d[i];
        sls.dds=s_dd[i];
        sls.ddl=d_dd[i];
        PoseState ps = csp->sl_to_xy(sls);

        // ix_ = csp->calc_x(s[i]);
        // iy_ = csp->calc_y(s[i]);
        // if (isnan(ix_) || isnan(iy_)) break;
        // iyaw_ = csp->calc_yaw(s[i]);
        // ix.push_back(ix_);
        // iy.push_back(iy_);
        // iyaw.push_back(iyaw_);
        
        if (isnan(ps.x) || isnan(ps.y)) break;
        x.push_back(ps.x);
        y.push_back(ps.y);
        ds.push_back(hypot(ps.vx,ps.vy));
        if(hypot(ps.vx,ps.vy)==0){
            yaw.push_back(0);
            c.push_back(0);
            accel.push_back(hypot(ps.ax,ps.ay));
        }
        else{
            yaw.push_back(atan2(ps.vy,ps.vx));
            c.push_back( (ps.vx*ps.ay - ps.ax*ps.vy) / (hypot(ps.vx,ps.vy)*hypot(ps.vx,ps.vy)*hypot(ps.vx,ps.vy)) );
            accel.push_back( (ps.vx*ps.ax+ps.vy*ps.ay)/hypot(ps.vx,ps.vy) );
        }
        //if(isnan(c.back())||isnan(accel.back())) ROS_ERROR("nan c accel, %d, %d, %d, %d",isnan(c.back()),isnan(accel.back()),isnan(ps.ax),isnan(ps.ay));
    }

    // if (x.size() <= 1) {
    //     return false;
    // }

    // for (size_t i = 0; i < x.size() - 1; i++) {
    //     dx = x[i+1] - x[i];
    //     dy = y[i+1] - y[i];
    //     yaw.push_back(atan2(dy, dx));
    //     ds.push_back(hypot(dx, dy));
    // }
    // yaw.push_back(yaw.back());
    // ds.push_back(ds.back());

    // for (size_t i = 0; i < yaw.size() - 1; i++) {
    //     double dyaw = yaw[i+1] - yaw[i];
    //     if (dyaw > M_PI_2) {
    //         dyaw -= M_PI;
    //     } else if (dyaw < -M_PI_2) {
    //         dyaw += M_PI;
    //     }
    //     c.push_back(dyaw / ds[i]);
    // }

    return true;
}

bool FrenetPath::is_valid_path(const vector<Obstacle *> obstacles) {
    if (any_of(s_d.begin()+1, s_d.end(),
            [this](int i){return abs(i) > fot_hp->max_speed;})) {
        return false;
    }
    
    else if (any_of(s_dd.begin()+1, s_dd.end(),
            [this](int i){return i > fot_hp->max_accel;})) {
        return false;
    }

    else if (any_of(s_dd.begin()+1, s_dd.end(),
            [this](int i){return i < -fot_hp->max_break;})) {
        return false;
    }
    
    else if (any_of(c.begin()+1, c.end(),
            [this](int i){return abs(i) > fot_hp->max_curvature;})) {
        return false;
    }
    //if(false)return true;
    else if (is_collision(obstacles)) {
        return false;
    }
    else {
        return true;
    }
}

bool FrenetPath::is_collision(const vector<Obstacle *> obstacles) {
    if (obstacles.empty()) {
        return false;
    }

    Pose pose;
    Car car = Car();
    Vector2f p1, p2;
    vector<point> car_corners;
    vector<point> prev_car_corners;

    for (auto obstacle : obstacles) {
        double llx = obstacle->bbox.first.x(); //lower left
        double lly = obstacle->bbox.first.y();
        double urx = obstacle->bbox.second.x(); //upper right
        double ury = obstacle->bbox.second.y();
        
        pose.assign({x[1], y[1], yaw[1]});
        car.setPose(pose);
        prev_car_corners = car.getCorner();
        for (size_t i = 2; i < x.size(); i++) {
            // double d1 = norm(llx - x[i], lly - y[i]);
            // double d2 = norm(llx - x[i], ury - y[i]);
            // double d3 = norm(urx - x[i], ury - y[i]);
            // double d4 = norm(urx - x[i], lly - y[i]);
            // double closest = min({d1, d2, d3, d4});
            double closest = hypot(x[i]-(llx+urx)/2, y[i]-(lly+ury)/2) - hypot(llx-urx, lly-ury)/2 - hypot(car.head_length+car.tail_length, car.width)/2;

            pose.assign({x[i], y[i], yaw[i]});
            car.setPose(pose);
            car_corners = car.getCorner();
            if (closest <= COLLISION_CHECK_THRESHOLD) {
                prev_car_corners.insert(prev_car_corners.end(), car_corners.begin(), car_corners.end());
                ConvexHull car_hull = ConvexHull(prev_car_corners);
                //ROS_WARN("hull number %d", car_hull.size);
                vector<point> obstacle_corners;
                obstacle_corners.assign({point(llx,lly),point(urx,lly),point(urx,ury),point(llx,ury)});
                ConvexHull obstacle_hull = ConvexHull(obstacle_corners);
                if(checkCollision(car_hull, obstacle_hull)){
                    return true;
                }
            }
            prev_car_corners = car_corners;
        }
    }

    return false;
}

double FrenetPath::inverse_distance_to_obstacles(
    const vector<Obstacle *> obstacles) {
    double total_inverse_distance = 0.0;

    Pose pose;
    Car car = Car();
    vector<Point> car_outline;
    
    for (auto obstacle : obstacles) {
        double llx = obstacle->bbox.first.x();
        double lly = obstacle->bbox.first.y();
        double urx = obstacle->bbox.second.x();
        double ury = obstacle->bbox.second.y();
        
        //for (size_t i = 0; i < x.size(); i++) {
        for (size_t i = 0; i < x.size(); i+=fot_hp->dt/fot_hp->control_t) {
            // double d1 = norm(llx - x[i], lly - y[i]);
            // double d2 = norm(llx - x[i], ury - y[i]);
            // double d3 = norm(urx - x[i], ury - y[i]);
            // double d4 = norm(urx - x[i], lly - y[i]);

            // double d1 = distance_to_segment(x[i],y[i],llx,lly,llx,ury);
            // double d2 = distance_to_segment(x[i],y[i],llx,ury,urx,ury);
            // double d3 = distance_to_segment(x[i],y[i],urx,ury,urx,lly);
            // double d4 = distance_to_segment(x[i],y[i],urx,lly,llx,lly);

            // double closest = min({d1, d2, d3, d4});
            // //total_inverse_distance += 1.0 / closest;
            // total_inverse_distance = max(total_inverse_distance, 1.0/closest);

            pose.assign({x[i], y[i], yaw[i]});
            car.setPose(pose);
            car_outline = car.getOutline();
            vector<point> A;
            vector<point> B;

            for (size_t j = 0; j < car_outline.size(); j++) {
                A.push_back(point(car_outline[j][0],car_outline[j][1]));
            }
            B.push_back(point(llx,lly));
            B.push_back(point(urx,lly));
            B.push_back(point(urx,ury));
            B.push_back(point(llx,ury));
            B.push_back(point(llx,lly));
            total_inverse_distance = max(total_inverse_distance,1/max(getClearance(A,B),1e-6));
        }
    }
    return total_inverse_distance;
}
