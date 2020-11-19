#include "cubic_spline_2d.h"
#include "utils.h"
#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;

CubicSpline2D::CubicSpline2D(const vector<double> &x,
                             const vector<double> &y) {
    vector<vector<double>> filtered_points = remove_collinear_points(x, y);
    calc_s(filtered_points[0], filtered_points[1]);
    sx = CubicSpline1D(s, filtered_points[0]);
    sy = CubicSpline1D(s, filtered_points[1]);
}

void CubicSpline2D::calc_s(const vector<double>& x,
                           const vector<double>& y) {
    int nx = x.size();
    vector<double> dx (nx);
    vector<double> dy (nx);
    adjacent_difference(x.begin(), x.end(), dx.begin());
    adjacent_difference(y.begin(), y.end(), dy.begin());
    dx.erase(dx.begin());
    dy.erase(dy.begin());

    double cum_sum = 0.0;
    s.push_back(cum_sum);
    for (int i = 0; i < nx - 1; i++) {
        cum_sum += norm(dx[i], dy[i]);
        if(isnan(cum_sum)) ROS_ERROR("%lf %lf",dx[i],dy[i]);
        s.push_back(cum_sum);
    }
    s.erase(unique(s.begin(), s.end()), s.end());
}

double CubicSpline2D::calc_x(double t) {
    return sx.calc_der0(t);
}

double CubicSpline2D::calc_y(double t) {
    return sy.calc_der0(t);
}

double CubicSpline2D::calc_curvature(double t){
    double dx = sx.calc_der1(t);
    double ddx = sx.calc_der2(t);
    double dy = sy.calc_der1(t);
    double ddy = sy.calc_der2(t);
    double k = (ddy * dx - ddx * dy) /
            pow(pow(dx, 2) + pow(dy, 2), 1.5);
    return k;
}

double CubicSpline2D::calc_yaw(double t) {
    double dx = sx.calc_der1(t);
    double dy = sy.calc_der1(t);
    double yaw = atan2(dy, dx);
    return yaw;
}

double CubicSpline2D::find_s(double x, double y, double s0) {
    // 가장 가까운 i-1 to i 선분을 찾음 
    int i_closest = 1;
    double closest = INFINITY;
    int ssize = s.size();
    double s1 = s[0]; 
    double x1 = calc_x(s1);
    double y1 = calc_y(s1);
    int i = 1;
    while(i<ssize){
        double s2 = s[i];
        double x2 = calc_x(s2);
        double y2 = calc_y(s2);
        double dist = distance_to_segment(x,y,x1,y1,x2,y2);
        if (dist < closest) {
            closest = dist;
            i_closest = i;
        }
        i+=1;
        s1 = s2;
        x1 = x2;
        y1 = y2;
    }
    // i-2 to i+1 세 선분만 탐색
    double s_closest = s[i_closest];
    closest = INFINITY;
    double si = s[max(0,i_closest-2)];
    do {
        double px = calc_x(si);
        double py = calc_y(si);
        double dist = norm(x - px, y - py);
        if (dist < closest) {
            closest = dist;
            s_closest = si;
        }
        si += 0.005;
    } while (si < s[min(ssize-1,i_closest+1)]);
    return s_closest;
}

SLState CubicSpline2D::transform(PoseState ps){
    SLState sls;
    sls.s = find_s(ps.x,ps.y,s.front());
    double px = calc_x(sls.s);
    double py = calc_y(sls.s);
    double pyaw = calc_yaw(sls.s);
    double pcurvature = calc_curvature(sls.s);
    int i = std::upper_bound (s.begin(), s.end(), sls.s) - s.begin();
    //if(s.back()==sls.s) i=s.size()-1;
    double erryaw = pyaw - atan2( calc_y(s[i])-calc_y(s[i-1]), calc_x(s[i])-calc_x(s[i-1]) );
    Pose p = ps.getPose();
    p[2] = pyaw;
    PoseState transformed = ps.transform(p);
    if(1-pcurvature*sls.l<1e-6) ROS_ERROR("too big l");
    sls.l = (ps.x-px)*(-sin(pyaw)) + (ps.y-py)*cos(pyaw);
    sls.ds = transformed.vx/(1-pcurvature*sls.l)*cos(erryaw);
    sls.dl = transformed.vy;
    sls.dds = transformed.ax/(1-pcurvature*sls.l) + transformed.vx*transformed.vy*pcurvature/((1-pcurvature*sls.l)*(1-pcurvature*sls.l));
    sls.dds *= cos(erryaw);
    sls.ddl = transformed.ay - pcurvature/(1-pcurvature*sls.l)*transformed.vx*transformed.vx;
    return sls;
}

PoseState CubicSpline2D::sl_to_xy(SLState sls){
    PoseState ps;
    double ix_, iy_, iyaw_, icurvature_, di;
    ix_ = calc_x(sls.s);
    iy_ = calc_y(sls.s);
    //if (isnan(ix_) || isnan(iy_)) ROS_ERROR("nan xy");
    iyaw_ = calc_yaw(sls.s);
    int i = std::upper_bound (s.begin(), s.end(), sls.s) - s.begin();
    //if(s.back()==sls.s) i=s.size()-1;
    double erryaw = iyaw_ - atan2( calc_y(s[i])-calc_y(s[i-1]), calc_x(s[i])-calc_x(s[i-1]) );
    if(cos(erryaw)<0.9) ROS_ERROR("too big erryaw");
    
    icurvature_ = calc_curvature(sls.s);
    ps.y = sls.l;
    ps.vx = sls.ds * (1-icurvature_*sls.l) / cos(erryaw);
    ps.vy = sls.dl;
    ps.ax = (sls.dds*(1-icurvature_*sls.l) - sls.ds*sls.dl*icurvature_) / cos(erryaw);
    ps.ay = sls.ddl + icurvature_*(1-icurvature_*sls.l)*sls.ds*sls.ds;

    Pose pose;
    pose.assign({ix_,iy_,iyaw_});
    return ps.transform(pose,true);
}


vector<vector<double>>
CubicSpline2D::remove_collinear_points(vector<double> x, vector<double> y) {
    vector<vector<double>> filtered_points;
    vector<double> x_, y_;
    x_.push_back(x[0]);
    x_.push_back(x[1]);
    y_.push_back(y[0]);
    y_.push_back(y[1]);
    for (size_t i = 2; i < x.size()-1; i++) {
        bool collinear = are_collinear(
            x[i - 2], y[i - 2],
            x[i - 1], y[i - 1],
            x[i], y[i]
            );
        if (collinear) {
            continue;
        }
        x_.push_back(x[i]);
        y_.push_back(y[i]);
    }
    
    x_.push_back(x.back());
    y_.push_back(y.back());
    filtered_points.push_back(x_);
    filtered_points.push_back(y_);
    return filtered_points;
}

bool CubicSpline2D::are_collinear(double x1, double y1, double x2, double y2,
    double x3, double y3) {
    double a = x1 * (y2 - y3) +
               x2 * (y3 - y1) +
               x3 * (y1 - y2);
    return a <= 0.01;
}

double CubicSpline2D::calc_s_length(){
    return s.back();
}
