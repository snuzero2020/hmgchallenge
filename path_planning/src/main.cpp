#include "frenet_optimal_trajectory.h"
#include "frenet_path.h"
#include "cpp_struct.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "opencv2/opencv.hpp"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "utils.h"
#include <ctime>

#include <vector>
#include <tuple>
#include <cmath>

#include <iostream>

using namespace std;
//using namespace cv;

int VISUALIZE_IMAGE_SIZE = 100;
double IMAGE_SCALE = 12.0;
int IMAGE_SIZE = VISUALIZE_IMAGE_SIZE * IMAGE_SCALE;

int main(int argc, char** argv) {
    ros::init(argc, argv, "frenet_optimal_trejectory_main");

    double wx [3] = {0, 65, 95};
    double wy [3] = {0, 0, 0};
    double o_llx[3] = {25.0, 40.0, 57.0};
    double o_lly[3] = {0.4, -2.2, -2.1};
    double o_urx[3] = {30.0, 45.0, 62.0};
    double o_ury[3] = {2.2, -0.4, -0.3};
    
    FrenetInitialConditions fot_ic = {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        4.0,
        wx,
        wy,
        3,
        o_llx,
        o_lly,
        o_urx,
        o_ury,
        3
    };


    FrenetHyperparameters fot_hp;
    ros::param::get("/max_speed", fot_hp.max_speed);
    ros::param::get("/max_accel", fot_hp.max_accel);
    ros::param::get("/max_curvature", fot_hp.max_curvature);
    ros::param::get("/max_road_width_l", fot_hp.max_road_width_l);
    ros::param::get("/max_road_width_r", fot_hp.max_road_width_r);
    ros::param::get("/d_road_w", fot_hp.d_road_w);
    ros::param::get("/dt", fot_hp.dt);
    ros::param::get("/maxt", fot_hp.maxt);
    ros::param::get("/mint", fot_hp.mint);
    ros::param::get("/target_t", fot_hp.target_t);
    ros::param::get("/control_t", fot_hp.control_t);
    ros::param::get("/d_t_s", fot_hp.d_t_s);
    ros::param::get("/n_s_sample", fot_hp.n_s_sample);
    ros::param::get("/obstacle_clearance", fot_hp.obstacle_clearance);
    ros::param::get("/kd", fot_hp.kd);
    ros::param::get("/kv", fot_hp.kv);
    ros::param::get("/ka", fot_hp.ka);
    ros::param::get("/kj", fot_hp.kj);
    ros::param::get("/kt", fot_hp.kt);
    ros::param::get("/ko", fot_hp.ko);
    ros::param::get("/klat", fot_hp.klat);
    ros::param::get("/klon", fot_hp.klon);


    //working
    vector<double> x,y;
    x.assign(fot_ic.wx, fot_ic.wx + fot_ic.nw);
    y.assign(fot_ic.wy, fot_ic.wy + fot_ic.nw);
    CubicSpline2D *csp = new CubicSpline2D(x, y);
    Car car;

    while(1){
        clock_t startTime = clock();

        SLState sls = csp->transform(car.poseState);
        fot_ic.s0 = sls.s;
        fot_ic.c_speed = sls.ds;
        fot_ic.c_accel = sls.dds;
        fot_ic.c_d = sls.l;
        fot_ic.c_d_d = sls.dl;
        fot_ic.c_d_dd = sls.ddl;
        ROS_WARN("%+lf %lf %lf",car.poseState.x, car.poseState.y, car.poseState.yaw);
        ROS_WARN("% lf %lf %lf",car.poseState.vx, car.poseState.vy, car.poseState.yawrate);
        ROS_WARN("% lf %lf %lf",car.poseState.ax, car.poseState.ay, car.poseState.yaw);

        //FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
        FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp, csp);
        FrenetPath* best_frenet_path = fot.getBestPath();
        //CubicSpline2D* csp = fot.getcsp();

        clock_t endTime = clock();
        clock_t elapsed = endTime - startTime;
        double timeInSecond = (double)(elapsed / CLOCKS_PER_SEC);

        if (best_frenet_path) {
            cout << "Success\n";
        }
        else{
            cout << "Failure\n";
        }

        cv::Mat VisWindow(IMAGE_SIZE, IMAGE_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));

        for (int i=0; i<fot_ic.no; i++){
            cv::rectangle(VisWindow, cv::Rect(IMAGE_SCALE * o_llx[i],IMAGE_SIZE/2 - IMAGE_SCALE * o_ury[i], IMAGE_SCALE * (o_urx[i]-o_llx[i]), IMAGE_SCALE * (o_ury[i] - o_lly[i])), cv::Scalar(255, 0, 0));
        }

        double t_s = csp->calc_s_length();

        for (double t=0; t<t_s; t+=0.5){
            if(csp->calc_x(t) != NAN){
                using namespace cv;
                VisWindow.at<Vec3b>(int(IMAGE_SIZE/2 - IMAGE_SCALE*csp->calc_y(t)), int(IMAGE_SCALE*csp->calc_x(t))) = (0, 0, 0);
                VisWindow.at<Vec3b>(int(IMAGE_SIZE/2 - IMAGE_SCALE*csp->calc_y(t) + 1), int(IMAGE_SCALE*csp->calc_x(t))) = (0, 0, 0);
                VisWindow.at<Vec3b>(int(IMAGE_SIZE/2 - IMAGE_SCALE*csp->calc_y(t) - 1), int(IMAGE_SCALE*csp->calc_x(t))) = (0, 0, 0);
            }
        }

        for (int i=0; i<best_frenet_path->x.size(); i+=fot_hp.dt/fot_hp.control_t){
            cv::circle(VisWindow, cv::Point(int(IMAGE_SCALE*best_frenet_path->x[i]),IMAGE_SIZE/2 - int(IMAGE_SCALE*best_frenet_path->y[i])),5, cv::Scalar(0, 0, 255));
        }
        Car car_visual;
        Pose pose_visual;
        pose_visual.assign({best_frenet_path->x[0], best_frenet_path->y[0], best_frenet_path->yaw[0]});
        car_visual.setPose(pose_visual);
        vector<Point> car_outline = car_visual.getOutline();
        for (size_t j = 0; j < car_outline.size(); j++) {
            double x1 = car_outline[j][0];
            double y1 = car_outline[j][1];
            double x2 = car_outline[(j+1) % car_outline.size()][0];
            double y2 = car_outline[(j+1) % car_outline.size()][1];
            cv::line(VisWindow,cv::Point(int(IMAGE_SCALE*x1),IMAGE_SIZE/2 - int(IMAGE_SCALE*y1)),cv::Point(int(IMAGE_SCALE*x2),IMAGE_SIZE/2 - int(IMAGE_SCALE*y2)),cv::Scalar(0,0,255));
        }
        

        //ROS_WARN("%lf         %lf",best_frenet_path->x[1], best_frenet_path->y[1]);

        cv::imshow("Path_Visualize", VisWindow);
        cv::waitKey(1);

        if(abs(t_s - best_frenet_path->s.front()) < 10)
            break;

        fot_ic.s0 = best_frenet_path->s[1];
        fot_ic.c_speed = best_frenet_path->s_d[1];
        fot_ic.c_accel = best_frenet_path->s_dd[1];
        fot_ic.c_d = best_frenet_path->d[1];
        fot_ic.c_d_d = best_frenet_path->d_d[1];
        fot_ic.c_d_dd = best_frenet_path->d_dd[1];

        for(int control_count=0; control_count<fot_hp.dt/fot_hp.control_t; control_count++ ){
            vector<double> accel_steer = car.findAccelSteer(best_frenet_path->accel[control_count+1],best_frenet_path->c[control_count+1]);
            ROS_WARN("%lf %lf",accel_steer[0],accel_steer[1]);
            car.setPose(car.simulate(accel_steer[0], accel_steer[1], fot_hp.control_t));
        }
    }
    cv::destroyAllWindows();
    
    return 0;
}
