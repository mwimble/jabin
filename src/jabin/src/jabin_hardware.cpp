#include <boost/assign.hpp>

#include "jabin/jabin_hardware.h"

JabinHardware::JabinHardware(ros::NodeHandle nh) {
    ros::V_string joint_names =
        boost::assign::list_of("left_wheel_joint")("right_wheel_joint");

    for (size_t i = 0; i < joint_names.size(); i++) {
        hardware_interface::JointStateHandle joint_state_handle(
            joint_names[i], 
            &joints_[i].position,
            &joints_[i].velocity,
            &joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(
            joint_state_handle,
            &joints_[i].velocity_command);
        velocity_joint_interface_.registerHandle(joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    ROS_INFO("[JabinHardware::JabinHardware] constructed");
}

JabinHardware::~JabinHardware() {
	// Do nothing.
}

void JabinHardware::readInputs() {
	ROS_INFO("[JabinHardware::readInputs]");
}


void JabinHardware::writeSpeeds() {
	ROS_INFO("[JabinHardware::writeSpeeds] 0-vel: %7.3f, 0-cmd: %7.3f, 1-vel: %7.3f, 1-cmd: %7.3f", joints_[0].velocity, joints_[0].velocity_command, joints_[1].velocity, joints_[1].velocity_command);
}