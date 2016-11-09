#ifndef __ROTATE
#define __ROTATE

#include <ros/ros.h>
#include <ros/console.h>
//#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "line_detector/line_detector.h"
#include "strategy/StrategyContext.h"
#include "strategy/StrategyFn.h"
#include <string>

class Rotate : public StrategyFn {
private:
	static const string strategyHasntStarted;
	static const string strategySuccess;

	typedef enum {
		kROTATING_START,
		kROTATING_LEFT,
		kROTATING_RIGHT
	} STATE;
	
	STATE state;

	// Print debug info?
	bool debug_;

	// For tracking line while turning.
	bool horizontalLineFound;
	bool horizontalToLeft;
	bool horizontalToRight;
	bool lineDetectorMsgReceived;
	bool verticalLineFound;
	double verticalCurveA;
	double verticalCurveB;
	double verticalIntercept;

	// Topic to publish robot movements.
	ros::Publisher cmdVelPub_;

	// Topic name containing cmd_vel message.
	string cmdVelTopicName_;

	// Topic to publish current strategy.
	ros::Publisher currentStrategyPub_;

	double goalYaw_;

	// // Subscriber to IMU message.
	// ros::Subscriber imuSub_;

	// // Topic name containing imu message.
	// string imuTopicName_;

	// Last report yaw via odometry.
	double lastYaw_;

	// To help log strategy only when it changes.
	string lastReportedStrategy_;

	// Subscriber to line_detector message.
	ros::Subscriber lineDetectorSub_;

	// Topic name containing line_detector message.`
	string lineDetectorTopicName_;

	// ROS node handle.
	ros::NodeHandle nh_;

	// Subscriber to odom message.
	ros::Subscriber odomSub_;

	// Topic name containing odom message.
	string odomTopicName_;

	// Yaw at start of turn.
	double startYaw_;

	// // From IMU callback.
	// double yaw_;

	static StrategyContext& strategyContext;

	// For publishing strategy status.
	ros::Publisher strategyStatusPublisher_;

	Rotate();

	// // Process IMU messages.
	// void imuTopicCb(const sensor_msgs::Imu& msg);

	// Process line_detector messages.
	void lineDetectorTopicCb(const line_detector::line_detector& msg);

	// Process line_detector messages.
	void odomTopicCb(const nav_msgs::Odometry& msg);

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

public:
	StrategyFn::RESULT_T tick();

	string name();

	static Rotate& Singleton();

};

#endif