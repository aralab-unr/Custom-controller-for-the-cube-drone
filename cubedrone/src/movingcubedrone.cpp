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
#include <cstdlib>

// Custom clamp function
template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
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

class MovingController {
public:
    MovingController() :
        controlpsi_(50.4, 0.05 , 15.4),
        controlxy_(2.5, 0, 1.4) {
        propvel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/prop_vel", 10);
        state_sub_ = nh_.subscribe("/gazebo/model_states", 1000, &MovingController::stateCallback, this);
        last_time_ = ros::Time::now();
    }

    void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        if (msg->pose.size() <= 1 || msg->twist.size() <= 1) {
            ROS_WARN("Insufficient pose or twist data received.");
            return;
        }

        geometry_msgs::Pose state = msg->pose[1];
        geometry_msgs::Twist twist = msg->twist[1];

        // Extract linear velocity
        double robotvel = sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2) + pow(twist.linear.z, 2));

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        double xdes = 7.5, ydes = 5.0;
        constexpr double pi = 3.141593;
        double ag = (ydes - state.position.y) / (xdes - state.position.x);
        double psides = atan(ag) - pi/2;
        ROS_INFO("Psi des: [%f]; Psi: [%f]", psides , 2 * state.orientation.z);
        double Fm = clamp(controlpsi_.calculate(psides, 2 * state.orientation.z, dt), -20.0, 20.0);
        double Fs;
        double w9, w10;
        double kt = 0.00025;
        double error = (xdes * xdes + ydes * ydes) - (state.position.x * state.position.x + state.position.y * state.position.y);
        double vr;
        if ( abs(psides - 2 * state.orientation.z) > 0.05) {
            vr = 0;
            Fs = 0;
            w10 = -sgn(Fm) * sqrt(abs( Fm /(2 * kt)));
            w9 = sgn(Fm) * sqrt(abs( Fm /(2 * kt)));
        } 
        else {
            vr = 1 / (1 + exp(-error)) - 0.5;
            Fm = 0;
            Fs = clamp(controlxy_.calculate(vr, robotvel, dt), -20.0, 20.0);
            w10 = sgn(Fs) * sqrt(abs(Fs/(2 * kt)));
            w9 = sgn(Fs) * sqrt(abs(Fs/(2 * kt)));
        }
        if ( abs(xdes-state.position.x) < 0.05 &&  abs(ydes-state.position.y) < 0.05) {
            w9 = 0;
            w10 =0;
        }
        ROS_INFO("Position: [%f][%f][%f][%f][%f][%f]", state.position.x, state.position.y, state.position.z, state.orientation.x, state.orientation.y, state.orientation.z);
        ROS_INFO("Control: [%f][%f][%f]", w9, w10, Fs);

        std_msgs::Float64MultiArray float64_array_msg;
        float64_array_msg.data = {0, 0, 0, 0, 0, 0, 0, 0, w9, w10};
        propvel_pub_.publish(float64_array_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher propvel_pub_;
    ros::Subscriber state_sub_;
    ros::Time last_time_;

    PID controlpsi_;
    PID controlxy_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "movingcube");
    MovingController controller;
    ros::spin();
    return 0;
}
