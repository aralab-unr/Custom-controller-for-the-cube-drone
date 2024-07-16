#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>

// Custom clamp function
template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

class PID {
public:
    PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), previous_error_(0), integral_(0) {}

    double calculate(double setpoint, double pv, double dt) {
        double error = setpoint - pv;
        double Pout = kp_ * error;
        integral_ += error * dt;
        double Iout = ki_ * integral_;
        double derivative = (error - previous_error_) / dt;
        double Dout = kd_ * derivative;
        double output = Pout + Iout + Dout;
        previous_error_ = error;
        return output;
    }

private:
    double kp_, ki_, kd_;
    double previous_error_, integral_;
};

class PIDXY {
public:
    PIDXY(double kdphi, double kpphi, double kdtheta, double kptheta, double kpx, double kdx, double kpy, double kdy) 
        : kdphi_(kdphi), kpphi_(kpphi), kdtheta_(kdtheta), kptheta_(kptheta), kpx_(kpx), kdx_(kdx), kpy_(kpy), kdy_(kdy), 
          previous_error_x(0), previous_error_phi(0), previous_error_y(0), previous_error_theta(0) {}

    std::pair<double, double> calculatexy(double setpointx, double setpointy, double pvx, double pvy, double pvphi, double pvtheta, double dt) {
        // Calculate error
        double errorx = setpointx - pvx;
        double errory = setpointy - pvy;
        double derivativex = (errorx - previous_error_x) / dt;
        double derivativey = (errory - previous_error_y) / dt;
        previous_error_x = errorx;
        previous_error_y = errory;

        // Proportional term
        double thetar =  kpx_ * errorx + kdx_ * derivativex;
        double phir =  - kpy_ * errory - kdy_ * derivativey;
        thetar = clamp(thetar, -0.1, 0.1);
        phir = clamp(phir, -0.1, 0.1);

        double errorphi = phir - pvphi;
        double errortheta = thetar - pvtheta;
        double derivativephi = (errorphi - previous_error_phi) / dt;
        double derivativetheta = (errortheta - previous_error_theta) / dt;
        previous_error_phi = errorphi;
        previous_error_theta = errortheta;

        double U2 = kpphi_ * errorphi + kdphi_ * derivativephi;
        double U3 = kptheta_ * errortheta + kdtheta_ * derivativetheta;
        U2 = clamp(U2, -0.01, 0.01);
        U3 = clamp(U3, -0.01, 0.01);

        return {U2, U3};
    }

private:
    double kpx_, kdx_, kpy_, kdy_;
    double kdphi_, kpphi_, kdtheta_, kptheta_;
    double previous_error_x, previous_error_phi;
    double previous_error_y, previous_error_theta;
};

class UAVController {
public:
    UAVController() :
        controlz_(8.0, 0, 5.75),
        controlpsi_(0.05, 0, 0.04),
        controlxy_(0.05, 0.08, 0.05, 0.08, 0.1, 0.15, 0.1, 0.15)
    {
        propvel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/prop_vel", 10);
        state_sub_ = nh_.subscribe("/gazebo/model_states", 1000, &UAVController::stateCallback, this);
        last_time_ = ros::Time::now();
    }

    void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        if (msg->pose.size() <= 1) {
            ROS_WARN("Insufficient pose data received.");
            return;
        }

        geometry_msgs::Pose state = msg->pose[1];

        ROS_INFO("I heard: [%f][%f][%f][%f][%f][%f]",
                 state.position.x, state.position.y, state.position.z,
                 state.orientation.x, state.orientation.y, state.orientation.z);

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        double xdes = 5, ydes = 5 , zdes = 5.0, psides = 0;

        double U1 = clamp(controlz_.calculate(zdes, state.position.z, dt) + 27.5, 20.0, 32.5);
        double U4 = clamp(controlpsi_.calculate(psides, state.orientation.z, dt), -0.01, 0.01);
        auto [U2, U3] = controlxy_.calculatexy(xdes, ydes, state.position.x, state.position.y, state.orientation.x, state.orientation.y, dt);

        double kt = 0.00025, kd = 0.000075, ly = 0.01845, lx= 0.0219 ;
        double w12, w22, w32, w42;

        w12 = (U2*kd*lx + U3*kd*ly + U1*kd*lx*ly + U4*kt*lx*ly) / (4 * kd * kt * lx * ly);
        w22 = (-U2*kd*lx + U3*kd*ly + U1*kd*lx*ly - U4*kt*lx*ly) / (4 * kd * kt * lx * ly);
        w32 = (U2*kd*lx - U3*kd*ly + U1*kd*lx*ly - U4*kt*lx*ly) / (4 * kd * kt * lx * ly);
        w42 = (-U2*kd*lx - U3*kd*ly + U1*kd*lx*ly + U4*kt*lx*ly) / (4 * kd * kt * lx * ly);

        w12 = std::max(0.0, w12);
        w22 = std::max(0.0, w22);
        w32 = std::max(0.0, w32);
        w42 = std::max(0.0, w42);

        double w1 = -sqrt(w12/2);
        double w2 = sqrt(w22/2);
        double w3 = sqrt(w32/2);
        double w4 = -sqrt(w42/2);

        ROS_INFO("Position: [%f][%f][%f][%f][%f][%f]", state.position.x, state.position.y, state.position.z, state.orientation.x, state.orientation.y, state.orientation.z);
        ROS_INFO("Control: [%f][%f][%f][%f]", U1, U2, U3, U4);
        std_msgs::Float64MultiArray float64_array_msg;
        float64_array_msg.data = {w1, w2, w3, w4, w1, w2, w3, w4, 0, 0};
        propvel_pub_.publish(float64_array_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher propvel_pub_;
    ros::Subscriber state_sub_;
    ros::Time last_time_;

    PID controlz_;
    PID controlpsi_;
    PIDXY controlxy_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav_controller");
    UAVController controller;
    ros::spin();
    return 0;
}
