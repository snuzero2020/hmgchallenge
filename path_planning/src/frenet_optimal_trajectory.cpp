#include <iostream>
#include "ros/ros.h"
#include "quintic_polynomial.h"
#include "utils.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frenet_optimal_trajectory");
    std::cout << norm(1.0, 2.0) << std::endl;
    return 0;
}