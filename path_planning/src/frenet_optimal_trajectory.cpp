#include <iostream>
#include "ros/ros.h"
#include "quintic_polynomial.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frenet_optimal_trajectory");
    QuinticPolynomial quintic(1,1,1,1,1,1);
    std::cout<<quintic.calc_first_derivative(1)<<std::endl;
    return 0;
}