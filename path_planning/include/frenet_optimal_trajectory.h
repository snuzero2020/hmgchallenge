#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENET_OPTIMAL_TRAJECTORY_H

#include "frenet_path.h"
#include "cpp_struct.h"
#include "cubic_spline_2d.h"
#include "obstacle.h"
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class FrenetPath1D {
 public:
    vector<double> t;          // time
    vector<double> y;          // s position along spline
    vector<double> y_d;        // s speed
    vector<double> y_dd;       // s acceleration
    vector<double> y_ddd;      // s jerk
    double c_deviation = 0;
    double c_velocity = 0;
    double c_accel = 0;
    double c_jerk = 0;
    double c_end = 0;
};

class FrenetOptimalTrajectory {
public:
    FrenetOptimalTrajectory(FrenetInitialConditions *fot_ic_, FrenetHyperparameters *fot_hp_, CubicSpline2D *csp_);
    ~FrenetOptimalTrajectory();
    FrenetPath* getBestPath();
    CubicSpline2D* getcsp();
    void setObstacles();
    void addObstacle(Vector2f first_point, Vector2f second_point);
private:
    FrenetInitialConditions *fot_ic;
    FrenetHyperparameters *fot_hp;
    FrenetPath *best_frenet_path;
    CubicSpline2D *csp;
    vector<Obstacle *> obstacles;
    vector<double> x, y;
    vector<FrenetPath *> frenet_paths;
    void calc_frenet_paths();
    vector<FrenetPath1D*> quartic_paths_1D(vector<double> t, double y0, double dy0, double ddy0, double min_v, double max_v, double step_v, double target_v, double dvf);
    vector<FrenetPath1D*> quintic_paths_1D(vector<double> t, double y0, double dy0, double ddy0, double min_y, double max_y, double step_y, double target_y, double dyf, double ddyf);
};

#endif