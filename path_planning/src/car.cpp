#include "car.h"
#include <cmath>

void Car::setPose(Pose p){
    pose = p;
    poseState.setPose(pose);
}
void Car::setPose(PoseState ps){
    poseState=ps;
    pose = ps.getPose();
}

vector<Point> Car::getOutline(){
    double x, y, yaw;
    double tail_x, tail_y, head_x, head_y;
    vector<double> tail_l, tail_r;
    vector<double> head_l, head_r;

    x = pose[0];
    y = pose[1];
    yaw = pose[2];

    tail_x = x - cos(yaw) * tail_length;
    tail_y = y - sin(yaw) * tail_length;
    tail_l.push_back(tail_x + cos(yaw + M_PI_2) * width / 2.0);
    tail_l.push_back(tail_y + sin(yaw + M_PI_2) * width / 2.0);
    tail_r.push_back(tail_x + cos(yaw - M_PI_2) * width / 2.0);
    tail_r.push_back(tail_y + sin(yaw - M_PI_2) * width / 2.0);

    head_x = x + cos(yaw) * head_length;
    head_y = y + sin(yaw) * head_length;
    head_l.push_back(head_x + cos(yaw + M_PI_2) * width / 2.0);
    head_l.push_back(head_y + sin(yaw + M_PI_2) * width / 2.0);
    head_r.push_back(head_x + cos(yaw - M_PI_2) * width / 2.0);
    head_r.push_back(head_y + sin(yaw - M_PI_2) * width / 2.0);

    vector<Point> outline;
    outline.push_back(tail_l);
    outline.push_back(tail_r);
    outline.push_back(head_r);
    outline.push_back(head_l);
    outline.push_back(tail_l);
    return outline;
}

PoseState Car::simulate( double accel, double steer, double dt ){
    PoseState transformed = poseState.transform(poseState.getPose()); //poseState.getPose() == pose
    double curvature = steer*rear_length/(front_length*front_length + rear_length*rear_length);
    // transformed.x, transformed.y == 0
    // transformed.vy ==? 0
    transformed.vy *= pow(0.5,dt);  //일단 야매임

    transformed.ax = accel;
    transformed.ay = curvature*transformed.vx*transformed.vx;
    transformed.x += transformed.vx*dt + 0.5*transformed.ax*dt*dt;
    transformed.y += transformed.vy*dt + 0.5*transformed.ay*dt*dt;
    transformed.vx += transformed.ax*dt;
    transformed.vy += transformed.ay*dt;

    transformed.yaw += curvature*transformed.x;
    transformed.yaw = remainder(transformed.yaw,2*M_PI);
    transformed.yawrate = curvature*transformed.vx;

    return transformed.transform(poseState.getPose(), true);
}

vector<double> Car::findAccelSteer( double accel, double curvature ){
    vector<double> accelSteer;
    accelSteer[0] = accel;
    accelSteer[1] = curvature*(front_length*front_length + rear_length*rear_length)/rear_length;
    return accelSteer;
}