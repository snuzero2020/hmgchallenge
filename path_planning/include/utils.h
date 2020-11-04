#ifndef FRENET_OPTIMAL_TRAJECTORY_UTILS_H
#define FRENET_OPTIMAL_TRAJECTORY_UTILS_H

#include <cmath>
#include <tuple>
#include <vector>

using namespace std;

typedef vector<double> Point;
typedef vector<double> Pose;

class PoseState{
    public:
        double x;
        double y;
        double yaw;
        double vx;
        double vy;
        double yawrate;
        double ax;
        double ay;
        Pose getPose(){
            Pose pose;
            pose[0] = x;
            pose[1] = y;
            pose[2] = yaw;
        }
        void setPose(Pose pose){
            x = pose[0];
            y = pose[1];
            yaw = pose[2];
        }
        PoseState transform(Pose pose0, bool inverse = false){
            PoseState transformed;
            if(inverse == true){
                double x0 = pose0[0];
                double y0 = pose0[1];
                double yaw0 = pose0[2];
                pose0[0] = -x0*cos(yaw0) -y0*sin(yaw0);
                pose0[1] = x0*sin(yaw0) -y0*cos(yaw0);
                pose0[2] = -yaw0;
            }
            double tmpx = x - pose0[0];
            double tmpy = y - pose0[1];
            transformed.x = tmpx*cos(pose0[2]) + tmpy*sin(pose0[2]);
            transformed.y = -tmpx*sin(pose0[2]) + tmpy*cos(pose0[2]);
            transformed.yaw = yaw - pose0[2];
            transformed.yaw = remainder(transformed.yaw,2*M_PI);
            transformed.vx = vx*cos(pose0[2]) + vy*sin(pose0[2]);
            transformed.vy = -vx*sin(pose0[2]) + vy*cos(pose0[2]);
            transformed.ax = ax*cos(pose0[2]) + ay*sin(pose0[2]);
            transformed.ay = -ax*sin(pose0[2]) + ay*cos(pose0[2]);
            return transformed;
        }
};

struct SLState{
    public:
        double s;
        double l;
        double ds;
        double dl;
        double dds;
        double ddl;
};

inline double norm(double x, double y){
    return sqrt(pow(x,2) + pow(y,2));
}

inline void as_unit_vector(tuple<double, double>& vec){
    double magnitude = norm(get<0>(vec), get<1>(vec));
    if (magnitude > 0){
        get<0>(vec) = get<0>(vec) / magnitude;
        get<1>(vec) = get<1>(vec) / magnitude;
    }
}

inline double dot(const tuple<double, double>& vec1, const tuple<double, double>& vec2){
    return get<0>(vec1) * get<0>(vec2) + get<1>(vec1) * get<1>(vec2);
}

inline double distance_to_segment(double x, double y, double x1, double y1, double x2, double y2){
    double dx2 = x2 - x1;
    double dy2 = y2 - y1;
    double dx = x - x1;
    double dy = y - y1;
    if(dx*dx2 + dy*dy2 <=0) return norm(dx,dy);
    if(dx*dx2 + dy*dy2 >=norm(dx2,dy2)) return norm(dx-dx2,dy-dy2);
    return abs(dx*dy2 - dy*dx2)/norm(dx2,dy2);
}

#endif