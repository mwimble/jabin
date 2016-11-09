#include <ros/ros.h>
#include <actionlib_msgs/GoalStatus.h>
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <unistd.h>

#include "strategy/GotoCrossing.h"
#include "strategy/Rotate.h"

using namespace std;

// double angle(double angle1, double angle2) {
//   double x1=cos(angle1);
//   double y1=sin(angle1);
//   double x2=cos(angle2);
//   double y2=sin(angle2);

//   double dot_product = x1*x2 + y1*y2;
//   return acos(dot_product);
// }

Rotate::Rotate() :
	debug_(false), 
	state(kROTATING_START),
	verticalLineFound(false) {

	nh_ = ros::NodeHandle("~");
	nh_.getParam("cmd_vel_topic_name", cmdVelTopicName_);
	ROS_INFO("[Rotate] PARAM cmd_vel_topic_name: %s", cmdVelTopicName_.c_str());

	nh_.getParam("debug_rotate", debug_);
	ROS_INFO("[Rotate] PARAM debug_rotate: %s", debug_ ? "TRUE" : "false");

	// nh_.getParam("imu_topic_name", imuTopicName_);
	// ROS_INFO("[Rotate] PARAM imu_topic_name: %s", imuTopicName_.c_str());

	nh_.getParam("line_detector_topic_name", lineDetectorTopicName_);
	ROS_INFO("[Rotate] PARAM line_detector_topic_name: %s", lineDetectorTopicName_.c_str());

	nh_.getParam("odom_topic_name", odomTopicName_);
	ROS_INFO("[Rotate] PARAM odomTopicName: %s", odomTopicName_.c_str());

	currentStrategyPub_ = nh_.advertise<std_msgs::String>("current_stragety", 1, true /* latched */);
	lastReportedStrategy_ = strategyHasntStarted;
	//imuSub_ = nh_.subscribe(imuTopicName_.c_str(), 1, &Rotate::imuTopicCb, this);
	odomSub_ = nh_.subscribe(odomTopicName_.c_str(), 1, &Rotate::odomTopicCb, this);
	cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>(cmdVelTopicName_.c_str(), 1);
	lineDetectorSub_ = nh_.subscribe(lineDetectorTopicName_.c_str(), 1, &Rotate::lineDetectorTopicCb, this);
	strategyStatusPublisher_ = nh_.advertise<actionlib_msgs::GoalStatus>("/strategy", 1);
}

Rotate& Rotate::Singleton() {
	static Rotate singleton_;
	return singleton_;
}

double normalizeEuler(double yaw) {
	double result = yaw;
	while (result > 360) result -= 360;
	while (result < 0) result += 360;
	return result;
}

// void Rotate::imuTopicCb(const sensor_msgs::Imu& msg) {
// 	// yaw_ = normalizeEuler(tf::getYaw(msg.orientation));
//     double roll, pitch, yaw;
//     tf::Quaternion q;
//     tf::quaternionMsgToTF(msg.orientation, q);
//     tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
//     yaw_ = normalizeEuler(57.29577951308229 * yaw);
// 	if (debug_) {
// 		ROS_INFO("[Rotate::normalizeEuler] yaw: %7.4f", yaw_);
// 	}
// }

void Rotate::odomTopicCb(const nav_msgs::Odometry& msg) {
	tf::Pose pose;
	tf::poseMsgToTF(msg.pose.pose, pose);
	lastYaw_ = tf::getYaw(pose.getRotation());
 	if (debug_) {
		ROS_INFO("[Rotate::odomTopicCb] lastYaw_: %7.4f", lastYaw_);
	}
}

string Rotate::name() {
	return string("Rotate");
}

void Rotate::lineDetectorTopicCb(const line_detector::line_detector& msg) {
	horizontalLineFound = msg.horizontalToLeft || msg.horizontalToRight;
	horizontalToLeft = msg.horizontalToLeft;
	horizontalToRight = msg.horizontalToRight;
	verticalCurveA = msg.verticalCurveA;
	verticalCurveB = msg.verticalCurveB;
	verticalIntercept = msg.cameraWidth / 2;
	verticalLineFound = msg.verticalToBottom;
	lineDetectorMsgReceived = true;

	int verticalWidth = msg.verticalUpperRightX - msg.verticalLowerLeftX;
	if (msg.horizontalToLeft && msg.horizontalToRight) {
		verticalIntercept = (verticalWidth / 2) + msg.verticalLowerLeftX;
	} else if (msg.horizontalToLeft) {
		verticalIntercept = msg.horizontalUpperRightX - 28;
	} else if (msg.horizontalToRight) {
		verticalIntercept = msg.horizontalLowerLeftX + 28;
	} else {
		verticalIntercept = (verticalWidth / 2) + msg.verticalLowerLeftX;
	}

	if (debug_) {
		ROS_INFO("[GotoCrossing::lineDetectorTopicCb] horizontalToLeft: %s, horizontalToRight: %s, (horizontalBottom: %d, horizontalLeft: %d, horizontalLength: %d), verticalToTop: %s, verticalToBottom: %s (verticalBottom: %d, verticalLeft: %d, verticalYlength: %d)",
			msg.horizontalToLeft ? "TRUE" : "false",
			msg.horizontalToRight ? "TRUE" : "false",
			msg.horizontalBottom, 
			msg.horizontalLeft, 
			msg.horizontalLength,
			msg.verticalToTop ? "TRUE" : "false", 
			msg.verticalToBottom ? "TRUE" : "false",
			msg.verticalBottom, 
			msg.verticalLeft, 
			msg.verticalYlength);
	}
}

void Rotate::publishCurrentStragety(string strategy) {
	std_msgs::String msg;
	msg.data = strategy;
	if (strategy != lastReportedStrategy_) {
		lastReportedStrategy_ = strategy;
		currentStrategyPub_.publish(msg);
		ROS_INFO("[Rotate::publishCurrentStragety] strategy: %s", strategy.c_str());
	}
}

StrategyFn::RESULT_T Rotate::tick() {
	geometry_msgs::Twist 		cmdVel;
	actionlib_msgs::GoalStatus 	goalStatus;
	bool 						keepRotating = false;
	RESULT_T 					result = FATAL;
	ostringstream 				ss;
	double 						verticalError;

	goalStatus.goal_id.stamp = ros::Time::now();
	goalStatus.goal_id.id = "Rotate";
	goalStatus.status = actionlib_msgs::GoalStatus::ACTIVE;

	if (strategyContext.needToFollowLine) {
		if (debug_) {
			ROS_INFO("[Rotate::tick] Still following line");
		}

		goalStatus.text = "Still following line";
		strategyStatusPublisher_.publish(goalStatus);
		result = SUCCESS;
		return result;
	}

	if (!strategyContext.needToRotateLeft90 && !strategyContext.needToRotateRight90) {
		if (debug_) {
			ROS_INFO("[Rotate::tick] no need to rotate");
		}

		goalStatus.text = "No need to follow line";
		strategyStatusPublisher_.publish(goalStatus);
		result = SUCCESS;
		return result;
	}

	const double degrees90 = M_PI / 2.0;

	switch (state) {
		case kROTATING_START:
			ss << "kROTATING_START ";
			startYaw_ = lastYaw_;
			if (strategyContext.needToRotateLeft90) {
				state = kROTATING_LEFT;
				result = RUNNING;
				goalYaw_ = normalizeEuler(startYaw_ + degrees90);
				if (debug_) ROS_INFO("[Rotate::tick] start of rotate left, startYaw_: %7.4f, goalYaw_: %7.4f", startYaw_, goalYaw_);
			} else if (strategyContext.needToRotateRight90) {
				state = kROTATING_RIGHT;
				result = RUNNING;
				goalYaw_ = normalizeEuler(startYaw_ - degrees90);
				if (debug_) ROS_INFO("[Rotate::tick] start of rotate right, startYaw_: %7.4f, goalYaw_: %7.4f", startYaw_, goalYaw_);
			} else {
				result = FATAL;
				if (debug_) ROS_INFO("[Rotate::tick] BAD GOAL--neither kROTATING_LEFT nor kROTATING_RIGHT");
				ss << "BAD GOAL--neither kROTATING_LEFT nor kROTATING_RIGHT";
			}

			ss << "new state: " << (state == kROTATING_LEFT ? "kROTATING_LEFT" : (state == kROTATING_LEFT ? "kROTATING_LEFT" : "UNKNOWN"));
			ss << ", startYaw_: " << startYaw_;
			ss << ", goalYaw_: " << goalYaw_;
			break;

		case kROTATING_LEFT:
			ss << "kROTATING_LEFT ";
			verticalError = abs(verticalIntercept - 160);
			keepRotating = (!verticalLineFound || (verticalError > 7)) && (lastYaw_ < goalYaw_);

			// if (!keepRotating && !verticalLineFound) {
			// 	if (startYaw_ > goalYaw_) {
			// 		keepRotating = (lastYaw_ > (goalYaw_ + degrees90)) || (lastYaw_ < goalYaw_);
			// 	} else {
			// 		keepRotating = lastYaw_ < goalYaw_;
			// 	}
			// }

			if (debug_) {
				ROS_INFO("[Rotate::tick] kROTATING_LEFT, keepRotating: %s, startYaw_: %7.4f, goalYaw_: %7.4f, lastYaw_: %7.4f, verticalLineFound: %s, verticalIntercept: %7.4f, verticalError: %7.4f",
					     keepRotating ? "TRUE" : "false",
					     startYaw_,
					     goalYaw_,
					     lastYaw_,
					     verticalLineFound ? "TRUE" : "false",
					     verticalIntercept,
					     verticalError);
			}

			if (keepRotating) {
				cmdVel.linear.x = 0.0;
				cmdVel.angular.z = 0.4;
				cmdVelPub_.publish(cmdVel);
				result = RUNNING;
				if (debug_) {
					ROS_INFO("[Rotate::tick] keep rotating left, x: %7.4f, z: %7.4f", cmdVel.linear.x, cmdVel.angular.z);
				}
			} else {
				cmdVel.linear.x = 0.0;
				cmdVel.angular.z = 0.0;
				cmdVelPub_.publish(cmdVel);
				result = SUCCESS;
				strategyContext.needToRotateLeft90 = false;
				GotoCrossing::Singleton().restartGoal();
				if (debug_) {
					ROS_INFO("[Rotate::tick] end of rotate left, startYaw_: %7.4f, lastYaw_: %7.4f", startYaw_, lastYaw_);
				}
			}

			ss << "keepRotating: " << keepRotating;
			ss << ", verticalError: " << verticalError;
			ss << ", lastYaw_: " << lastYaw_;
			ss << ", goalYaw_: " << goalYaw_;
			ss << ", verticalLineFound: " << (verticalLineFound ? "TRUE" : "false");
			ss << ", moving with x: " << cmdVel.linear.x << " and z: " << cmdVel.angular.z;
			break;

		case kROTATING_RIGHT:
			ss << "kROTATING_LEFT ";
			verticalError = abs(verticalIntercept - 160);
			keepRotating = (!verticalLineFound || (verticalError > 7)) && (lastYaw_ > goalYaw_);

			// if (!keepRotating && !verticalLineFound) {
			// 	if (startYaw_ < goalYaw_) {
			// 		keepRotating = (lastYaw_ < (goalYaw_ + degrees90)) || (lastYaw_ > goalYaw_);
			// 	} else {
			// 		keepRotating = lastYaw_ > goalYaw_;
			// 	}
			// }

			if (debug_) {
				ROS_INFO("[Rotate::tick] kROTATING_LEFT, keepRotating: %s, startYaw_: %7.4f, goalYaw_: %7.4f, lastYaw_: %7.4f, verticalLineFound: %s, verticalIntercept: %7.4f, verticalError: %7.4f",
					     keepRotating ? "TRUE" : "false",
					     startYaw_,
					     goalYaw_,
					     lastYaw_,
					     verticalLineFound ? "TRUE" : "false",
					     verticalIntercept,
					     verticalError);
			}
			if (keepRotating) {
				cmdVel.linear.x = 0.0;
				cmdVel.angular.z = -0.4;
				cmdVelPub_.publish(cmdVel);
				result = RUNNING;
				if (debug_) {
					ROS_INFO("[Rotate::tick] keep rotating right, x: %7.4f, z: %7.4f", cmdVel.linear.x, cmdVel.angular.z);
				}
			} else {
				cmdVel.linear.x = 0.0;
				cmdVel.angular.z = 0.0;
				cmdVelPub_.publish(cmdVel);
				result = SUCCESS;
				strategyContext.needToRotateRight90 = false;
				GotoCrossing::Singleton().restartGoal();
				if (debug_) {
					ROS_INFO("[Rotate::tick] end of rotate right, startYaw_: %7.4f, lastYaw_: %7.4f", startYaw_, lastYaw_);
				}
			}

			ss << "keepRotating: " << keepRotating;
			ss << ", verticalError: " << verticalError;
			ss << ", lastYaw_: " << lastYaw_;
			ss << ", goalYaw_: " << goalYaw_;
			ss << ", verticalLineFound: " << (verticalLineFound ? "TRUE" : "false");
			ss << ", moving with x: " << cmdVel.linear.x << " and z: " << cmdVel.angular.z;
			break;
			
		otherwise:
			ss << "UNKNOWN STATE";
			break;
	}

	goalStatus.text = ss.str();
	strategyStatusPublisher_.publish(goalStatus);
	return result;
}

StrategyContext& Rotate::strategyContext = StrategyContext::Singleton();

const string Rotate::strategyHasntStarted = "Rotate: Strategy hasn't started";
const string Rotate::strategySuccess = "Rotate: SUCCESS";
