#include <ros/node_handle.h>

#include "ctre/phoenix/motorcontrol/VelocityMeasPeriod.h"
#include "robot_motor_control/TalonNode.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace robot_motor_control {

    TalonNode::TalonNode(const ros::NodeHandle& parent, const std::string& name, const TalonConfig& config) :
            nh(parent), _name(name), server(nh), talon(new TalonSRX(config.id)),
            tempPub(nh.advertise<std_msgs::Float64>("temperature", 1)),
            busVoltagePub(nh.advertise<std_msgs::Float64>("bus_voltage", 1)),
            outputPercentPub(nh.advertise<std_msgs::Float64>("output_percent", 1)),
            outputVoltagePub(nh.advertise<std_msgs::Float64>("output_voltage", 1)),
            outputCurrentPub(nh.advertise<std_msgs::Float64>("output_current", 1)),
            posPub(nh.advertise<std_msgs::Int32>("position", 1)),
            velPub(nh.advertise<std_msgs::Int32>("velocity", 1)),
            setPercentSub(nh.subscribe("set_percent_output", 1, &TalonNode::setPercentOutput, this)),
            setVelSub(nh.subscribe("set_velocity", 1, &TalonNode::setVelocity, this)),
            lastUpdate(ros::Time::now()), _controlMode(ControlMode::PercentOutput), _output(0.0){
        server.setCallback(boost::bind(&TalonNode::reconfigure, this, _1, _2));
        server.updateConfig(config);
        talon->Set(ControlMode::PercentOutput, 0);
        configureStatusPeriod(*talon);
    }

    void TalonNode::setPercentOutput(std_msgs::Float64 output) {
        this->_controlMode = ControlMode::PercentOutput;
        this->_output = output.data;
        this->lastUpdate = ros::Time::now();
    }

    void TalonNode::setVelocity(std_msgs::Float64 output) {
        this->_controlMode = ControlMode::Velocity;
        this->_output = output.data;
        this->lastUpdate = ros::Time::now();
    }

    void TalonNode::reconfigure(const TalonConfig &config, uint32_t level) {
        if (config.id != 0 && talon->GetDeviceID() != config.id) {
            ROS_INFO("Resetting TalonNode to new id: %d", config.id);
            talon = std::make_unique<TalonSRX>(config.id);
            configureStatusPeriod(*talon);
        }

        TalonSRXConfiguration c;
        SlotConfiguration slot;
        slot.kP = config.P;
        slot.kI = config.I;
        slot.kD = config.D;
        slot.kF = config.F;
        c.slot0 = slot;
        c.voltageCompSaturation = config.peak_voltage;
        c.pulseWidthPeriod_EdgesPerRot = 4096;
        talon->ConfigAllSettings(c);

        talon->SelectProfileSlot(0,0);
        talon->SetInverted(config.inverted);
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

        std_msgs::Float64 temperature;
        temperature.data = talon->GetTemperature();
        tempPub.publish(temperature);

        std_msgs::Float64 busVoltage;
        busVoltage.data = talon->GetBusVoltage();
        busVoltagePub.publish(busVoltage);

        std_msgs::Float64 outputPercent;
        outputPercent.data = talon->GetMotorOutputPercent();
        outputPercentPub.publish(outputPercent);

        std_msgs::Float64 outputVoltage;
        outputVoltage.data = talon->GetMotorOutputVoltage();
        outputVoltagePub.publish(outputVoltage);

        std_msgs::Float64 outputCurrent;
        outputCurrent.data = talon->GetOutputCurrent();
        outputCurrentPub.publish(outputCurrent);

        std_msgs::Int32 position;
        position.data = talon->GetSelectedSensorPosition();
        posPub.publish(position);

        std_msgs::Int32 velocity;
        velocity.data = talon->GetSelectedSensorVelocity();
        velPub.publish(velocity);
    }

    void TalonNode::configureStatusPeriod(TalonSRX& talon){
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 20);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 20);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 200);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 200);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 200);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 200);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 200);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 200);
    }

}
