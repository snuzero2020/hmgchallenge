#ifndef FRENET_OPTIMAL_TRAJECTORY_OBSTACLE
#define FRENET_OPTIMAL_TRAJECTORY_OBSTACLE

#include <Eigen/Dense>

using namespace Eigen;

class Obstacle{
    public:
        std::pair<Vector2f, Vector2f> bbox;
        Obstacle(Vector2f first_point, Vector2f second_point,double obstacle_clearance);
        bool isSegmentInObstacle(Vector2f &p1, Vector2f &p2);
        bool isPointNearObstacle(Vector2f &p, double radius);
        double getArea();
};

#endif