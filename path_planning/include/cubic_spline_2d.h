#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBIC_SPLINE_2D
#define FRENET_OPTIMAL_TRAJECTORY_CUBIC_SPLINE_2D

#include "cubic_spline_1d.h"
#include <vector>

using namespace std;

class CubicSpline2D{
    private:
        vector<double> s;
        CubicSpline1D sx, sy;
        void calc_s(const vector<double>& x, const vector<double>& y);
        vector<vector<double>> remove_collinear_points(vector<double> x, vector<double> y);
        bool are_collinear(double x1, double y1, double x2, double y2, double x3, double y3);
    public:
        CubicSpline2D() = default;
        CubicSpline2D(const vector<double> &x, const vector<double> &y);
        double calc_x(double t);
        double calc_y(double t);
        double calc_curvature(double t);
        double calc_yaw(double t);
        double find_s(double x, double y, double s0);
};

#endif