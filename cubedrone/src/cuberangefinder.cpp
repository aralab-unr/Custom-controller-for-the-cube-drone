#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/ModelStates.h"
#include <geometry_msgs/TransformStamped.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <iostream>
#include <chrono>
#include <limits>

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

class RangeFinder {
public:
    RangeFinder() :
        controlz_(8.0, 0, 5.75),
        controlrange_(0.75, 0, 0),
        controlroll_(0.5, 0, 0.4),
        controlpitch_(0.5, 0, 0.4),
        controlpsi_(0.05, 0, 0.04),
        zdes(1.5),  // Initial desired z position
        lastsquare_(0) 
    {
        propvel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/prop_vel", 10);
        top_scan_sub_ = nh_.subscribe("/top_scan", 10, &RangeFinder::topScanCallback, this);
        bottom_scan_sub_ = nh_.subscribe("/bottom_scan", 10, &RangeFinder::bottomScanCallback, this);
        state_sub_ = nh_.subscribe("/gazebo/model_states", 1000, &RangeFinder::stateCallback, this);
        zdes_sub_ = nh_.subscribe("/desired_z", 10, &RangeFinder::desiredZCallback, this);  // Subscriber for desired z
        zdes_pub_ = nh_.advertise<std_msgs::Float32>("/desired_z", 10);  // Publisher for desired z
        last_time_ = ros::Time::now();
        ROS_INFO("RangeFinder node initialized.");
    }

    void topScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        min_range_top = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    }

    void bottomScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        min_range_bottom = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    }

    void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        if (msg->pose.size() <= 1) {
            ROS_WARN("Insufficient pose data received.");
            return;
        }

        geometry_msgs::Pose state = msg->pose[1];
        geometry_msgs::Twist twist = msg->twist[1];

        // Extract linear velocity components
        
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();

        // Ensure dt is not zero
        if (dt == 0) {
            ROS_WARN("Time difference (dt) is zero, skipping this iteration.");
            return;
        }
        double square = sqrt(state.position.x*state.position.x + state.position.y*state.position.y);
        double vel = (square-lastsquare_)/dt;
        // Use the velocity magnitude as the robot velocity

        lastsquare_ = square; // update the last square value with the current magnitude
        last_time_ = current_time;

        double rolldes = 0, pitchdes = 0, psides = 0;

        double U1 = clamp(controlz_.calculate(zdes, state.position.z, dt) + 27.5, 20.0, 32.5);
        double U4 = clamp(controlpsi_.calculate(psides, state.orientation.z, dt), -0.01, 0.01);
        double U2 = clamp(controlroll_.calculate(rolldes, state.orientation.x, dt), -0.05, 0.05);
        double U3 = clamp(controlpitch_.calculate(pitchdes, state.orientation.y, dt), -0.05, 0.05);

        double kt = 0.00025, kd = 0.000075, ly = 0.01845, lx = 0.0219;
        double w12, w22, w32, w42;

        w12 = (U2 * kd * lx + U3 * kd * ly + U1 * kd * lx * ly + U4 * kt * lx * ly) / (4 * kd * kt * lx * ly);
        w22 = (-U2 * kd * lx + U3 * kd * ly + U1 * kd * lx * ly - U4 * kt * lx * ly) / (4 * kd * kt * lx * ly);
        w32 = (U2 * kd * lx - U3 * kd * ly + U1 * kd * lx * ly - U4 * kt * lx * ly) / (4 * kd * kt * lx * ly);
        w42 = (-U2 * kd * lx - U3 * kd * ly + U1 * kd * lx * ly + U4 * kt * lx * ly) / (4 * kd * kt * lx * ly);

        w12 = std::max(0.0, w12);
        w22 = std::max(0.0, w22);
        w32 = std::max(0.0, w32);
        w42 = std::max(0.0, w42);

        double w1 = -sqrt(w12 / 2);
        double w2 = sqrt(w22 / 2);
        double w3 = sqrt(w32 / 2);
        double w4 = -sqrt(w42 / 2);

        // Ensure min_range_top is not NaN before calculating Fs
        double rangedes = 0.153;
        if (std::isnan(min_range_top)) {
            min_range_top = rangedes; 
        }
        double error = min_range_top - rangedes;
        double Fs;
        double vr = 1 / (1 + exp(-error)) - 0.5;
        ROS_INFO("Robot veldes %f", vr);
        Fs = clamp(controlrange_.calculate(vr, vel, dt), -0.3, 0.3);
        ROS_INFO("Control law  %f", Fs);
        ROS_INFO("Range  %f", min_range_top);
        double w10 = sgn(Fs) * sqrt(abs(Fs / (2 * kt)));
        double w9 = sgn(Fs) * sqrt(abs(Fs / (2 * kt)));

        std_msgs::Float64MultiArray float64_array_msg;
        float64_array_msg.data = {w1, w2, w3, w4, w1, w2, w3, w4, w9, w10};
        propvel_pub_.publish(float64_array_msg);
    }

    void desiredZCallback(const std_msgs::Float32::ConstPtr& msg) {
        zdes = msg->data;
        ROS_INFO("Desired z set to: %f", zdes);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber top_scan_sub_;
    ros::Subscriber bottom_scan_sub_;
    ros::Publisher propvel_pub_;
    ros::Publisher zdes_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber zdes_sub_;
    ros::Time last_time_;
    float zdes;
    PID controlz_;
    PID controlrange_;
    PID controlpsi_;
    PID controlroll_;
    PID controlpitch_;
public:
    float min_range_top;
    float min_range_bottom;
    float lastsquare_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "range_finder_node");
    RangeFinder range_finder;
    ros::spin();
    return 0;
}