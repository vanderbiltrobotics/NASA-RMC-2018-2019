#ifndef MOTOR_CONTROL_TALONNODE_H
#define MOTOR_CONTROL_TALONNODE_H

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "robot_motor_control/TalonConfig.h"
#include <algorithm>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/server.h>

namespace robot_motor_control{
    class TalonNode{
    private:
        ros::NodeHandle nh;
        std::string _name;
        dynamic_reconfigure::Server<robot_motor_control::TalonConfig> server;

        std::unique_ptr<TalonSRX> talon;

        ros::Publisher tempPub;
        ros::Publisher busVoltagePub;
        ros::Publisher outputPercentPub;
        ros::Publisher outputVoltagePub;
        ros::Publisher outputCurrentPub;
        ros::Publisher posPub;
        ros::Publisher velPub;

        ros::Subscriber setPercentSub;
        ros::Subscriber setVelSub;

        ros::Time lastUpdate;
        ControlMode _controlMode;
        double _output;
        bool disabled;

    public:
        TalonNode(const ros::NodeHandle& parent, const std::string& name);

        TalonNode& operator=(const TalonNode&) = delete;

        ~TalonNode() = default;

        void reconfigure(const TalonConfig &config, uint32_t level);

        void updateConfig(const TalonConfig &config);

        void setPercentOutput(std_msgs::Float64 output);

        void setVelocity(std_msgs::Float64 output);

        void update();

        static void configureStatusPeriod(TalonSRX& talon);

    };

}

#endif //MOTOR_CONTROL_TALONNODE_H
