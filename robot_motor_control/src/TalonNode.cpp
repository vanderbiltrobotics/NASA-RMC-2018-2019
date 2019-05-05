#include <ros/node_handle.h>

#include "ctre/phoenix/motorcontrol/VelocityMeasPeriod.h"
#include "robot_motor_control/TalonNode.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace robot_motor_control {

    TalonNode::TalonNode(const ros::NodeHandle& parent, const std::string& name, const TalonConfig& config):
            nh(parent), _name(name), server(nh), _config(config), talon(new TalonSRX(_config.id)),
            tempPub(nh.advertise<std_msgs::Float64>("temperature", 1)),
            busVoltagePub(nh.advertise<std_msgs::Float64>("bus_voltage", 1)),
            outputPercentPub(nh.advertise<std_msgs::Float64>("output_percent", 1)),
            outputVoltagePub(nh.advertise<std_msgs::Float64>("output_voltage", 1)),
            outputCurrentPub(nh.advertise<std_msgs::Float64>("output_current", 1)),
            analogPub(nh.advertise<std_msgs::Float64>("analog_input", 1)),
            posPub(nh.advertise<std_msgs::Int32>("position", 1)),
            velPub(nh.advertise<std_msgs::Int32>("velocity", 1)),
            fwdPub(nh.advertise<std_msgs::Bool>("forward_limit", 1)),
            revPub(nh.advertise<std_msgs::Bool>("reverse_limit", 1)),
            setPercentSub(nh.subscribe("set_percent_output", 1, &TalonNode::setPercentOutput, this)),
            setVelSub(nh.subscribe("set_velocity", 1, &TalonNode::setVelocity, this)),
            lastUpdate(ros::Time::now()), _controlMode(ControlMode::PercentOutput), _output(0.0), disabled(false),
            configured(false), not_configured_warned(false){
        server.updateConfig(_config);
        server.setCallback(boost::bind(&TalonNode::reconfigure, this, _1, _2));
        talon->Set(ControlMode::PercentOutput, 0);
    }

    void TalonNode::setPercentOutput(std_msgs::Float64 output) {
        boost::mutex::scoped_lock scoped_lock(mutex);
        this->_controlMode = ControlMode::PercentOutput;
        this->_output = output.data;
        this->lastUpdate = ros::Time::now();
    }

    void TalonNode::setVelocity(std_msgs::Float64 output) {
        boost::mutex::scoped_lock scoped_lock(mutex);
        this->_controlMode = ControlMode::Velocity;
        this->_output = output.data;
        this->lastUpdate = ros::Time::now();
    }

    void TalonNode::reconfigure(const TalonConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure called on %d with id %d", talon->GetDeviceID(), config.id);
        boost::mutex::scoped_lock scoped_lock(mutex);
        this->_config = config;
        this->configured = false;
    }

    void TalonNode::configure(){
        if (talon->GetDeviceID() != _config.id) {
            ROS_INFO("Resetting TalonNode to new id: %d", _config.id);
            talon = std::make_unique<TalonSRX>(_config.id);
        }

        TalonSRXConfiguration c;
        SlotConfiguration slot;
        slot.kP = _config.P;
        slot.kI = _config.I;
        slot.kD = _config.D;
        slot.kF = _config.F;
        c.slot0 = slot;
        c.voltageCompSaturation = _config.peak_voltage;
        c.pulseWidthPeriod_EdgesPerRot = 4096;
        ErrorCode error = talon->ConfigAllSettings(c, 50);

        if(error != ErrorCode::OK){
            if(!this->not_configured_warned){
                ROS_WARN("Reconfiguring Talon %s %d failed!", _name.c_str(), talon->GetDeviceID());
                this->not_configured_warned = true;
            }
            this->configured = false;
        }else{
            configureStatusPeriod(*talon);

            if(_config.pot){
                talon->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog);
            }else{
                talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
            }

            talon->SetSensorPhase(_config.invert_sensor);
            talon->SelectProfileSlot(0,0);
            talon->SetInverted(_config.inverted);
            talon->EnableVoltageCompensation(true);

            ROS_INFO("Reconfigured Talon: %s with %d %f %f %f", _name.c_str(), talon->GetDeviceID(),
                     _config.P, _config.I, _config.D);
            this->not_configured_warned = false;
            this->configured = true;
        }
    }

    void TalonNode::update(){
        boost::mutex::scoped_lock scoped_lock(mutex);
        if(!this->configured){
            this->configure();
        }
        if(ros::Time::now()-lastUpdate > ros::Duration(0.2)){
            // Disable the Talon if we aren't getting commands
            if(!this->disabled) ROS_WARN("Talon disabled for not receiving updates: %s", _name.c_str());
            this->disabled = true;
            this->_controlMode = ControlMode::PercentOutput;
            this->_output = 0.0;
        }else{
            this->disabled = false;
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

        std_msgs::Float64 analogInput;
        analogInput.data = talon->GetSensorCollection().GetAnalogIn();
        analogPub.publish(analogInput);

        std_msgs::Bool forward_limit;
        forward_limit.data = talon->GetSensorCollection().IsFwdLimitSwitchClosed();
        fwdPub.publish(forward_limit);

        std_msgs::Bool reverse_limit;
        reverse_limit.data = talon->GetSensorCollection().IsRevLimitSwitchClosed();
        fwdPub.publish(reverse_limit);

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
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250);
        talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250);
    }

}
