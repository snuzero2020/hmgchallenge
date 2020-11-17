#include "obstacle.h"
#include <algorithm>
#include "ros/console.h"

using namespace Eigen;
using namespace std;

Obstacle::Obstacle(Vector2f first_point, Vector2f second_point, double obstacle_clearance)
{
    // Get tansLeft and bottomRight points from the given points.
    Vector2f tmp;
    if (first_point.x() > second_point.x() && first_point.y() > second_point.y()) {
        tmp = first_point;
        first_point = second_point;
        second_point = tmp;
    } else if (first_point.x() < second_point.x() && first_point.y() > second_point.y()) {
        float height = first_point.y() - second_point.y();
        first_point.y() -= height;
        second_point.y() += height;
    } else if (first_point.x() > second_point.x() && first_point.y() < second_point.y()) {
        float length = first_point.x() - second_point.x();
        first_point.x() -= length;
        second_point.x() += length;
    }
    first_point.x() -= obstacle_clearance;
    first_point.y() -= obstacle_clearance;
    second_point.x() += obstacle_clearance;
    second_point.y() += obstacle_clearance;

    bbox.first.x() = first_point.x();
    bbox.first.y() = first_point.y();
    bbox.second.x() = second_point.x();
    bbox.second.y() = second_point.y();
}

bool Obstacle::isPointNearObstacle(Vector2f &p, double radius) {
    double dist_to_ll, dist_to_lr, dist_to_ul, dist_to_ur;
    dist_to_ll = sqrt(pow(bbox.first.x() - p.x(), 2) +
                      pow(bbox.first.y() - p.y(), 2));
    dist_to_lr = sqrt(pow(bbox.second.x() - p.x(), 2) +
                      pow(bbox.first.y() - p.y(), 2));
    dist_to_ul = sqrt(pow(bbox.first.x() - p.x(), 2) +
                      pow(bbox.second.y() - p.y(), 2));
    dist_to_ur = sqrt(pow(bbox.second.x() - p.x(), 2) +
                      pow(bbox.second.y() - p.y(), 2));
    if (dist_to_ll <= radius || dist_to_lr <= radius ||
        dist_to_ul <= radius || dist_to_ur <= radius ) {
        return true;
    }
    return false;
}


bool Obstacle::isSegmentInObstacle(Vector2f &p1, Vector2f &p2)
{
    float length = bbox.second.x() - bbox.first.x();
    float breadth = bbox.second.y() - bbox.first.y();

    Vector2f l1_p1(bbox.first.x(), bbox.first.y());
    Vector2f l1_p2(bbox.first.x() + length, bbox.first.y());    
    Vector2f l2_p1(bbox.first.x(), bbox.first.y());
    Vector2f l2_p2(bbox.first.x(), bbox.first.y() + breadth);
    Vector2f l3_p1(bbox.second.x(), bbox.second.y());
    Vector2f l3_p2(bbox.second.x(), bbox.second.y() - breadth);
    Vector2f l4_p1(bbox.second.x(), bbox.second.y());
    Vector2f l4_p2(bbox.second.x() - length, bbox.second.y());

    bool x1, x2, x3, x4;
    x1 = checkIntersect(p1, p2, l1_p1, l1_p2);
    x2 = checkIntersect(p1, p2, l2_p1, l2_p2);
    x3 = checkIntersect(p1, p2, l3_p1, l3_p2);
    x4 = checkIntersect(p1, p2, l4_p1, l4_p2);

    return x1 || x2 || x3 || x4;
}

bool Obstacle::checkIntersect(Vector2f p1, Vector2f p2, Vector2f l_p1, Vector2f l_p2)
{
    int x1 = ccw(p1, p2, l_p1) * ccw(p1, p2, l_p2);
    int x2 = ccw(l_p1, l_p2, p1) * ccw(l_p1, l_p2, p2);
    
    if (x1 == 0 && x2 == 0){
        if((p1.x()>l_p1.x() && p1.x()>l_p2.x() && p2.x()>l_p1.x() && p2.x()>l_p2.x()) || (l_p1.x()>p1.x() && l_p1.x()>p2.x() && l_p2.x()>p1.x() && l_p2.x()>p2.x()))
            return false;
        else if((p1.y()>l_p1.y() && p1.y()>l_p2.y() && p2.y()>l_p1.y() && p2.y()>l_p2.y()) || (l_p1.y()>p1.y() && l_p1.y()>p2.y() && l_p2.y()>p1.y() && l_p2.y()>p2.y()))
            return false;
    }
    
    //if((x1 <= 0)&&(x2 <= 0)) ROS_ERROR("%lf %lf %lf %lf : %lf %lf %lf %lf",p1.x(),p1.y(),p2.x(),p2.y(),l_p1.x(),l_p1.y(),l_p2.x(),l_p2.y());

    return (x1 <= 0)&&(x2 <= 0);
}

int Obstacle::ccw(Vector2f p1, Vector2f p2, Vector2f p3)
{
    double EPS = 1e-6;
    double ans = (p2.x()-p1.x()) * (p3.y()-p1.y()) - (p2.y()-p1.y())* (p3.x()-p1.x()); 
    if (ans > EPS) 
        return 1;
    else if (ans < -EPS) 
        return -1;
    else 
        return 0;
}
