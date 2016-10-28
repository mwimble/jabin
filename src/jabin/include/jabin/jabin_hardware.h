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
};

#endif
