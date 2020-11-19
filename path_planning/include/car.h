#ifndef FRENET_OPTIMAL_TRAJECTORY_CAR_H
#define FRENET_OPTIMAL_TRAJECTORY_CAR_H

#include "utils.h"
#include <vector>
#include <tuple>

using namespace std;

const double VEHICLE_FRONT_OVERHANG = 1.03; // front end to front wheel
const double VEHICLE_FRONT_LENGTH = 1.0;    // front wheel to mass center
const double VEHICLE_REAR_LENGTH = 1.8; // mass center to rear wheel
const double VEHICLE_REAR_OVERHANG = 1.1;   // real wheel to real end
//const double VEHICLE_ORGIN_FROM_REAR = VEHICLE_FRONT_LENGTH + VEHICLE_REAR_LENGTH + VEHICLE_REAR_OVERHANG;    // vehicle orgin from vehicle rear end. orgin is center of front wheel
const double VEHICLE_ORGIN_FROM_REAR = VEHICLE_REAR_LENGTH + VEHICLE_REAR_OVERHANG;    // vehicle orgin from vehicle rear end. orgin is center of mass
//const double CORNERING_STIFFNESS_RATIO_OF_FRONT_TO_REAR = 1;    // front cornering stiffness / rear cornering stiffness
//const double VEHICLE_LENGTH = 4.93;
const double VEHICLE_WIDTH = 1.86;

class Car {
 public:
    double front_length; // wheel to mass center
    double rear_length;
    //wheel_base = front_length + rear_length;
    double head_length; // end to orgin
    double tail_length;
    //total_length = head_length + tail_length;
    double width;

    Pose pose; // x, y, yaw
    PoseState poseState; // x, y, yaw, vx, vy, yawrate, ax, ay

    Car(){
        front_length = VEHICLE_FRONT_LENGTH;
        rear_length = VEHICLE_REAR_LENGTH;
        head_length = VEHICLE_FRONT_OVERHANG + VEHICLE_FRONT_LENGTH + VEHICLE_REAR_LENGTH + VEHICLE_REAR_OVERHANG - VEHICLE_ORGIN_FROM_REAR;
        tail_length = VEHICLE_ORGIN_FROM_REAR;
        width = VEHICLE_WIDTH;
        pose.push_back(0);
        pose.push_back(0);
        pose.push_back(0);
        setPose(pose);
    };
    Car(Pose pose_): pose(pose_) {
        front_length = VEHICLE_FRONT_LENGTH;
        rear_length = VEHICLE_REAR_LENGTH;
        head_length = VEHICLE_FRONT_OVERHANG + VEHICLE_FRONT_LENGTH + VEHICLE_REAR_LENGTH + VEHICLE_REAR_OVERHANG - VEHICLE_ORGIN_FROM_REAR;
        tail_length = VEHICLE_ORGIN_FROM_REAR;
        width = VEHICLE_WIDTH;
        setPose(pose);
    };
    void setPose(Pose p);
    void setPose(PoseState ps);
    vector<Point> getOutline();
    vector<point> getCorner();
    PoseState simulate( double accel, double steer, double dt );
    vector<double> findAccelSteer( double accel, double curvature );
};

#endif