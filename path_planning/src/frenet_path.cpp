#include "frenet_path.h"
#include "utils.h"
#include "ros/console.h"

#include <algorithm>

const float COLLISION_CHECK_THRESHOLD = 6; 

FrenetPath::FrenetPath(FrenetHyperparameters *fot_hp_) {
    fot_hp = fot_hp_;
}

bool FrenetPath::to_global_path(CubicSpline2D* csp) {
    double ix_, iy_, iyaw_, icurvature_, di, fx, fy, dx, dy, ddx, ddy;
    // calc global positions
    for (size_t i = 0; i < s.size(); i++) {
        ix_ = csp->calc_x(s[i]);
        iy_ = csp->calc_y(s[i]);
        if (isnan(ix_) || isnan(iy_)) break;

        iyaw_ = csp->calc_yaw(s[i]);
        ix.push_back(ix_);
        iy.push_back(iy_);
        iyaw.push_back(iyaw_);
        di = d[i];
        fx = ix_ + di * cos(iyaw_ + M_PI_2);
        fy = iy_ + di * sin(iyaw_ + M_PI_2);
        x.push_back(fx);
        y.push_back(fy);

        icurvature_ = csp->calc_curvature(s[i]);
        dx = s_d[i] * (1-icurvature_*d[i]) * cos(iyaw_) - d_d[i] * sin(iyaw_);
        dy = s_d[i] * (1-icurvature_*d[i]) * sin(iyaw_) + d_d[i] * cos(iyaw_);
        ddx = (s_dd[i]*(1-icurvature_*d[i]) - s_d[i]*d_d[i]*icurvature_) * cos(iyaw_) - d_dd[i] * sin(iyaw_);
        ddy = (s_dd[i]*(1-icurvature_*d[i]) - s_d[i]*d_d[i]*icurvature_) * sin(iyaw_) + d_dd[i] * cos(iyaw_);
        ds.push_back(hypot(dx,dy));
        yaw.push_back(atan2(dy,dx));
        c.push_back( (dx*ddy - ddx*dy) / (hypot(dx,dy)*hypot(dx,dy)*hypot(dx,dy))  );
        if(dx*ddx+dy*ddy >=0) accel.push_back(hypot(ddx,ddy));
        else accel.push_back(-hypot(ddx,ddy));
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
    if (any_of(s_d.begin(), s_d.end(),
            [this](int i){return abs(i) > fot_hp->max_speed;})) {
        return false;
    }
    
    else if (any_of(s_dd.begin(), s_dd.end(),
            [this](int i){return abs(i) > fot_hp->max_accel;})) {
        return false;
    }
    
    else if (any_of(c.begin(), c.end(),
            [this](int i){return abs(i) > fot_hp->max_curvature;})) {
        return false;
    }
    //if(false)return false;
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
    vector<Point> car_outline;

    for (auto obstacle : obstacles) {
        double llx = obstacle->bbox.first.x();
        double lly = obstacle->bbox.first.y();
        double urx = obstacle->bbox.second.x();
        double ury = obstacle->bbox.second.y();
        for (size_t i = 0; i < x.size(); i++) {
        //for (size_t i = 0; i < x.size(); i+=fot_hp->dt/fot_hp->control_t) {
            double d1 = norm(llx - x[i], lly - y[i]);
            double d2 = norm(llx - x[i], ury - y[i]);
            double d3 = norm(urx - x[i], ury - y[i]);
            double d4 = norm(urx - x[i], lly - y[i]);
            // double d1 = distance_to_segment(x[i],y[i],llx,lly,llx,ury);
            // double d2 = distance_to_segment(x[i],y[i],llx,ury,urx,ury);
            // double d3 = distance_to_segment(x[i],y[i],urx,ury,urx,lly);
            // double d4 = distance_to_segment(x[i],y[i],urx,lly,llx,lly);

            double closest = min({d1, d2, d3, d4});
            if (closest <= COLLISION_CHECK_THRESHOLD) {
                double xp = x[i];
                double yp = y[i];
                double yawp = yaw[i];
                pose.assign({xp, yp, yawp});
                car.setPose(pose);
                car_outline = car.getOutline();
                for (size_t j = 0; j < car_outline.size(); j++) {
                    p1.x() = car_outline[j][0];
                    p1.y() = car_outline[j][1];
                    p2.x() = car_outline[(j+1) % car_outline.size()][0];
                    p2.y() = car_outline[(j+1) % car_outline.size()][1];
                    if (obstacle->isSegmentInObstacle(p1, p2)) {
                        //ROS_ERROR("p1 : %lf %lf, p2 : %lf %lf, obs : %lf %lf %lf %lf",p1.x(),p1.y(),p2.x(),p2.y(),llx,lly,urx,ury);
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

double FrenetPath::inverse_distance_to_obstacles(
    const vector<Obstacle *> obstacles) {
    double total_inverse_distance = 0.0;

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

            double d1 = distance_to_segment(x[i],y[i],llx,lly,llx,ury);
            double d2 = distance_to_segment(x[i],y[i],llx,ury,urx,ury);
            double d3 = distance_to_segment(x[i],y[i],urx,ury,urx,lly);
            double d4 = distance_to_segment(x[i],y[i],urx,lly,llx,lly);

            double closest = min({d1, d2, d3, d4});
            //total_inverse_distance += 1.0 / closest;
            total_inverse_distance = max(total_inverse_distance, 1.0/closest);
        }
    }
    return total_inverse_distance;
}
