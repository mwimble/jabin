#include <boost/assign.hpp>

#include "jabin/jabin_hardware.h"

JabinHardware::JabinHardware(ros::NodeHandle nh) :
	running_(true) {
    /*ros::V_string*/ joint_names =
        boost::assign::list_of("left_wheel_joint")("right_wheel_joint").convert_to_container<ros::V_string>();

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


void JabinHardware::writeSpeeds(ros::Duration elapsed) {
    if (running_) {
      for (unsigned int i = 0; i < joint_names.size(); ++i) {
        // Note that pos_[i] will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.
        joints_[i].position += joints_[i].velocity * elapsed.toSec(); // update position
        joints_[i].velocity = joints_[i].velocity_command; // might add smoothing here later
      }
    } else {
    	for (unsigned int i = 0; i < joint_names.size(); ++i) {
    		joints_[i].position = std::numeric_limits<double>::quiet_NaN();
    		joints_[i].velocity = std::numeric_limits<double>::quiet_NaN();
    	}
    }

	ROS_INFO("[JabinHardware::writeSpeeds 0] 0-pos: %7.3f 0-vel: %7.3f, 0-cmd: %7.3f",
		joints_[0].position,
		joints_[0].velocity,
		joints_[0].velocity_command);
	ROS_INFO("[JabinHardware::writeSpeeds 1] 1-pos: %7.3f 1-vel: %7.3f, 1-cmd: %7.3f", 
		joints_[1].position,
		joints_[1].velocity,
		joints_[1].velocity_command);
}