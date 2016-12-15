#ifndef JABINHARDWARE_H
#define JABINHARDWARE_H

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class JabinHardware : public hardware_interface::RobotHW {
public:
	JabinHardware(ros::NodeHandle nh);

	virtual ~JabinHardware();

    void readInputs();

    void writeSpeeds(ros::Duration elapsed);

private:
	bool running_;

	ros::V_string joint_names;

    struct Joint {
        double position;
        double velocity;
        double effort;
        double velocity_command;

        Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
    } joints_[2];

    hardware_interface::JointStateInterface joint_state_interface_;

    hardware_interface::VelocityJointInterface velocity_joint_interface_;

};

#endif
