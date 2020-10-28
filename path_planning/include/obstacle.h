#ifndef FRENET_OPTIMAL_TRAJECTORY_OBSTACLE
#define FRENET_OPTIMAL_TRAJECTORY_OBSTACLE

#include <Eigen/Dense>

using namespace Eigen;

class Obstacle{
    private:
        int ccw(Vector2f p1, Vector2f p2, Vector2f p3);
        bool checkIntersect(Vector2f p1, Vector2f p2, Vector2f l_p1, Vector2f l_p2);
    public:
        std::pair<Vector2f, Vector2f> bbox;
        Obstacle(Vector2f first_point, Vector2f second_point,double obstacle_clearance);
        bool isSegmentInObstacle(Vector2f &p1, Vector2f &p2);
        bool isPointNearObstacle(Vector2f &p, double radius);
};

#endif