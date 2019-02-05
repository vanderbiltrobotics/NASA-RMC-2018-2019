#include <ros/node_handle.h>

#include "ctre/phoenix/motorcontrol/VelocityMeasPeriod.h"
#include "motor_control/TalonNode.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace motor_control {

    TalonNode::TalonNode(ros::NodeHandle nh, std::string name, const TalonConfig &config) :
            nh(nh), _name(name), server(nh), talon(new TalonSRX(config.id)),
            tempPub(nh.advertise<std_msgs::Float64>("temperature", 1)),
            busVoltagePub(nh.advertise<std_msgs::Float64>("bus_voltage", 1)),
            outputPercentPub(nh.advertise<std_msgs::Float64>("output_percent", 1)),
            outputVoltagePub(nh.advertise<std_msgs::Float64>("output_voltage", 1)),
            posPub(nh.advertise<std_msgs::Int32>("position", 1)),
            velPub(nh.advertise<std_msgs::Int32>("velocity", 1)),
            setPercentSub(nh.subscribe("set_percent_output", 1, &TalonNode::setPercentOutput, this)),
            setVelSub(nh.subscribe("set_velocity", 1, &TalonNode::setVelocity, this)),
            lastUpdate(ros::Time::now()), _controlMode(ControlMode::PercentOutput), _output(0.0){
        server.setCallback(boost::bind(&TalonNode::reconfigure, this, _1, _2));
        server.updateConfig(config);
        talon->Set(ControlMode::PercentOutput, 0);
    }

    void TalonNode::setPercentOutput(std_msgs::Float64Ptr output) {
        this->_controlMode = ControlMode::PercentOutput;
        this->_output = output->data;
        this->lastUpdate = ros::Time::now();
    }

    void TalonNode::setVelocity(std_msgs::Float64Ptr output) {
        this->_controlMode = ControlMode::Velocity;
        this->_output = output->data;
        this->lastUpdate = ros::Time::now();
    }

    void TalonNode::reconfigure(const TalonConfig &config, uint32_t level) {
        TalonSRXConfiguration c;
        talon->ConfigAllSettings(c);

        if (config.id != 0 && talon->GetDeviceID() != config.id) {
            ROS_INFO("Resetting TalonNode to new id: %d", config.id);
            talon.reset(new TalonSRX(config.id));
        }
        talon->SetInverted(config.inverted);
        talon->ConfigVoltageCompSaturation(config.peak_voltage);
        talon->EnableVoltageCompensation(true);
    }

    void TalonNode::update(){
        if(ros::Time::now()-lastUpdate > ros::Duration(0.2)){
            // Disable the Talon if we aren't getting commands
            this->_controlMode = ControlMode::PercentOutput;
            this->_output = 0.0;
            this->lastUpdate = ros::Time::now();
            ROS_INFO("Talon disabled for not receiving updates: %s", _name.c_str());
        }
        talon->Set(this->_controlMode, this->_output);

        std_msgs::Float64Ptr temperature(new std_msgs::Float64());
        temperature->data = talon->GetTemperature();
        tempPub.publish(temperature);

        std_msgs::Float64Ptr busVoltage(new std_msgs::Float64());
        busVoltage->data = talon->GetBusVoltage();
        busVoltagePub.publish(busVoltage);

        std_msgs::Float64Ptr outputPercent(new std_msgs::Float64());
        outputPercent->data = talon->GetMotorOutputPercent();
        outputPercentPub.publish(outputPercent);

        std_msgs::Float64Ptr outputVoltage(new std_msgs::Float64());
        outputVoltage->data = talon->GetMotorOutputVoltage();
        outputVoltagePub.publish(outputVoltage);

        std_msgs::Int32Ptr position(new std_msgs::Int32());
        position->data = talon->GetSelectedSensorPosition();
        posPub.publish(position);

        std_msgs::Int32Ptr velocity(new std_msgs::Int32());
        velocity->data = talon->GetSelectedSensorVelocity();
        velPub.publish(velocity);
    }

}
