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
};

#endif