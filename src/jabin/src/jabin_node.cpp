#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <time.h>

#include "controller_manager/controller_manager.h"

#include "jabin/jabin_hardware.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "jabin_node");
    ros::NodeHandle nh;

    JabinHardware robot(nh);
    controller_manager::ControllerManager cm(&robot, nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate r(10 /*node_params.controller_loop_rate*/);

    ros::Time last_time;
    ros::Time current_time;
    ros::Duration elapsed;
    last_time = ros::Time::now();

    while (ros::ok()) {
        current_time = ros::Time::now();
        elapsed = last_time - current_time;
        last_time = current_time;
        // robot.readInputs();
        // cm.update(ros::Time::now(), elapsed);
        // robot.setParams(firmware_params);
        // robot.sendParams();
        // robot.writeSpeeds();

        r.sleep();
    }

    return 0;
}