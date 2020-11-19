#ifndef FRENET_OPTIMAL_TRAJECTORY_UTILS_H
#define FRENET_OPTIMAL_TRAJECTORY_UTILS_H

#include <cmath>
#include <tuple>
#include <vector>
#include <stack>
#include <ros/console.h>

using namespace std;

typedef vector<double> Point;
typedef vector<double> Pose;

const double INF = 1e9;
const double EPS = 1e-6;

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
    double theta;
    point() : point(0,0){}
    point(double x1, double y1): x(x1),y(y1), theta(0){}
    void update(point p){
        theta = atan2(y-p.y,x-p.x);
    }
    bool operator <(const point& o){
        if(abs(theta-o.theta)>EPS) return theta<o.theta;
        if(abs(y-o.y)>EPS) return y<o.y;
        return x<o.x;
    }
};

struct line{
    point s,e;
    line(): line(0,0,0,0){}
    line(double x1, double y1, double x2, double y2):s(x1,y1),e(x2,y2){}
    line(point p1, point p2):s(p1),e(p2){}
};

inline int ccw(const point &a, const point &b, const point &c){
    double rt = (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
    if(abs(rt)<EPS)return 0;
    if(rt<0) return -1;
    return 1;
}

inline bool intersect(line l1, line l2){
    int ab = ccw(l1.s,l1.e, l2.s)*ccw(l1.s,l1.e,l2.e);
    int cd = ccw(l2.s,l2.e,l1.s)*ccw(l2.s,l2.e,l1.e);

    if(ab==0 && cd ==0){
        if((l1.s.x>l2.s.x && l1.s.x>l2.e.x&&l1.e.x>l2.s.x&&l1.e.x>l2.e.x) ||(l1.s.x<l2.s.x && l1.s.x<l2.e.x&&l1.e.x<l2.s.x&&l1.e.x<l2.e.x)) return false;
        if((l1.s.y>l2.s.y && l1.s.y>l2.e.y&&l1.e.y>l2.s.y&&l1.e.y>l2.e.y) ||(l1.s.y<l2.s.y && l1.s.y<l2.e.y&&l1.e.y<l2.s.y&&l1.e.y<l2.e.y)) return false;
    }
    return (ab<=0)&&(cd<=0);
}

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

struct ConvexHull{
    public:
    vector<point> p;
    int size;

    ConvexHull(){
        size = 0;
    }
    ConvexHull(vector<point> v){
        size = 0;
        if(v.size()<3) {
            p=v;
            ROS_ERROR("ConvexHull Error : few point");
        }
        sort(v.begin(), v.end());
        for(int i=1;i<v.size();i++) v[i].update(v[0]);
        sort(v.begin()+1, v.end());
        stack<int> s;
        s.push(0);
        s.push(1);
        int next = 2;
        while(next<v.size()){
            while(s.size()>=2){
                int first, second;
                first = s.top();
                s.pop();
                second = s.top();
                if(ccw(v[second],v[first],v[next])>0){
                    s.push(first);
                    break;
                }
            }
            s.push(next);
            next++;
        }
        while(!s.empty()){
            p.push_back(v[s.top()]);
            s.pop();
            size++;
        }
    }
};

inline bool checkCollision(ConvexHull a, ConvexHull b){
    for(int i=0;i<a.size;i++){
        for(int j=0;j<b.size;j++){
            line l1 = line(a.p[(i%a.size)],a.p[((i+1)%a.size)]);
            line l2 = line(b.p[(j%b.size)],b.p[((j+1)%b.size)]);
            if(intersect(l1,l2)) return true;
        }
    }

    bool check = true;
    point p = b.p[0];
    int val = ccw(p,a.p[a.size-1],a.p[0]);
    for(int i=0;i<a.size-1;i++){
        if(ccw(p,a.p[i],a.p[i+1])!=val) {
            check = false;
            break;
        }
    }
    if(check) return true;
    check = true;
    p=a.p[0];
    val = ccw(p,b.p[b.size-1],b.p[0]);
    for(int i=0;i<b.size-1;i++){
        if(ccw(p,b.p[i],b.p[i+1])){
            check = false;
            break;
        }
    }
    if(check) return true;
    return false;
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