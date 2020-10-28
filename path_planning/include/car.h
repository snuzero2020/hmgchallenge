#ifndef FRENET_OPTIMAL_TRAJECTORY_CAR_H
#define FRENET_OPTIMAL_TRAJECTORY_CAR_H

#include "utils.h"
#include <vector>
#include <tuple>

using namespace std;

const double VEHICLE_LENGTH = 4.93;
const double VEHICLE_WIDTH = 1.86;

class Car {
    private:
        double length;
        double width;
        Pose pose; // x, y, yaw
    public:
        Car(){
            length = VEHICLE_LENGTH;
            width = VEHICLE_WIDTH;
        };
        Car(Pose pose_): pose(pose_) {
            length = VEHICLE_LENGTH;
            width = VEHICLE_WIDTH;
        };
        void setPose(Pose p);
        vector<Point> getOutline();
};

#endif