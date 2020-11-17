#ifndef FRENET_OPTIMAL_TRAJECTORY_UTILS_H
#define FRENET_OPTIMAL_TRAJECTORY_UTILS_H

#include <cmath>
#include <tuple>
#include <vector>

using namespace std;

typedef vector<double> Point;
typedef vector<double> Pose;

const double INF = 1e9;

class PoseState{
    public:
        double x=0;
        double y=0;
        double yaw=0;
        double vx=0;
        double vy=0;
        double yawrate=0;
        double ax=0;
        double ay=0;
        Pose getPose(){
            Pose pose;
            pose.push_back(x);
            pose.push_back(y);
            pose.push_back(yaw);
            return pose;
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


struct point{
    double x,y;
    point() : point(0,0){}
    point(double x1, double y1): x(x1),y(y1){}
};

struct line{
    point s,e;
    line(): line(0,0,0,0){}
    line(double x1, double y1, double x2, double y2){
        point p1 = point(x1,y1);
        point p2 = point(x2,y2);
        line(p1,p2);
    }
    line(point p1, point p2):s(p1),e(p2){}
};

inline double sq(double x){ return x*x;}

inline double dist(point a, point b){return sqrt(sq(a.x-b.x)+sq(a.y-b.y));}

inline double area(point a, point b){
    return 0.5*abs(a.y*b.x-a.x*b.y);
}

inline point vec(point s, point e){
    return point(e.x-s.x,e.y-s.y);
}

inline double length(line l){
    return dist(l.s,l.e);
}

inline bool valid(point p, point s, point e){ //수선의 발이 line(s,e)위에 있는지 확인
    double d = sq(dist(s,e));
    double x = sq(dist(p,s));
    double y = sq(dist(p,e));
    if(d+x<y||d+y<x) return false;
    return true;
}

inline double line_distance(line l1, line l2){
    double rt = INF;
    rt = min<double>(rt, dist(l1.s, l2.s));
    rt = min<double>(rt, dist(l1.s, l2.e));
    rt = min<double>(rt, dist(l1.e, l2.s));
    rt = min<double>(rt, dist(l1.e, l2.e));

    if(valid(l1.s, l2.s, l2.e)) rt = min<double>(rt, 2.0*area(vec(l1.s,l2.s),vec(l1.s, l2.e))/length(l2));
    if(valid(l1.e, l2.s, l2.e)) rt = min<double>(rt, 2.0*area(vec(l1.e,l2.s),vec(l1.e, l2.e))/length(l2));
    if(valid(l2.s, l1.s, l1.e)) rt = min<double>(rt, 2.0*area(vec(l2.s,l1.s),vec(l2.s, l1.e))/length(l1));
    if(valid(l2.e, l1.s, l1.e)) rt = min<double>(rt, 2.0*area(vec(l2.e,l1.s),vec(l2.e, l1.e))/length(l1));

    return rt;
}

inline double getClearance(vector<point> A, vector<point>B){
    double rt = INF;
    int a = A.size()-1;
    int b = B.size()-1;
    for(int i=0; i<a;i++){
        line l1 = line(A[i],A[i+1]);
        for(int j=0;j<b;j++){
            line l2 = line(B[j],B[j+1]);
            rt = min<double>(rt, line_distance(l1,l2));
        }
    }
    return rt;
}

inline double distance_to_segment(double x, double y, double x1, double y1, double x2, double y2){
    point p = point(x,y);
    line l = line(x1,y1,x2,y2);
    if(valid(p, l.s, l.e)) return 2.0*area(vec(p,l.s),vec(p,l.e))/length(l);
    return min<double>(dist(p,l.s),dist(p,l.e));
}

/*inline double distance_to_segment(double x, double y, double x1, double y1, double x2, double y2){
    double dx2 = x2 - x1;
    double dy2 = y2 - y1;
    double dx = x - x1;
    double dy = y - y1;
    if(dx*dx2 + dy*dy2 <=0) return norm(dx,dy);
    if(dx*dx2 + dy*dy2 >=norm(dx2,dy2)) return norm(dx-dx2,dy-dy2);
    return abs(dx*dy2 - dy*dx2)/norm(dx2,dy2);
}*/


#endif