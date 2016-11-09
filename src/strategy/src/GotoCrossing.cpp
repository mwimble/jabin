#include <ros/ros.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <unistd.h>

#include "strategy/GotoCrossing.h"

using namespace std;

GotoCrossing::GotoCrossing() :
	atLine_(false), 
	debug_(false), 
	horizontalLineFound(false),
	lineDetectorMsgReceived(false),
	odometryMessageReceived_(false),
	sawHorizontalLine(false),
	state(kLOOKING_FOR_HORIZONTAL_LINE_START),
	verticalLineFound(false) {

	nh_ = ros::NodeHandle("~");
	nh_.getParam("cmd_vel_topic_name", cmdVelTopicName_);
	ROS_INFO("[GotoCrossing] PARAM cmd_vel_topic_name: %s", cmdVelTopicName_.c_str());

	nh_.getParam("debug_goto_crossing", debug_);
	ROS_INFO("[GotoCrossing] PARAM debug_goto_crossing: %s", debug_ ? "TRUE" : "false");

	nh_.getParam("dontMove", dontMove_);
	ROS_INFO("[GotoCrossing] PARAM dontMove_: %s", dontMove_ ? "TRUE" : "false");

	nh_.getParam("line_detector_topic_name", lineDetectorTopicName_);
	ROS_INFO("[GotoCrossing] PARAM line_detector_topic_name: %s", lineDetectorTopicName_.c_str());

	currentStrategyPub_ = nh_.advertise<std_msgs::String>("current_stragety", 1, true /* latched */);
	lastReportedStrategy_ = strategyHasntStarted;
	lineDetectorSub_ = nh_.subscribe(lineDetectorTopicName_.c_str(), 1, &GotoCrossing::lineDetectorTopicCb, this);
	odometrySub_ = nh_.subscribe("/odom", 1, &GotoCrossing::odomTopicCb, this);
	cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>(cmdVelTopicName_.c_str(), 1);
	strategyStatusPublisher_ = nh_.advertise<actionlib_msgs::GoalStatus>("/strategy", 1);
}

GotoCrossing& GotoCrossing::Singleton() {
	static GotoCrossing singleton_;
	return singleton_;
}

void GotoCrossing::lineDetectorTopicCb(const line_detector::line_detector& msg) {
	lastLineDetectorMsg = msg;
	horizontalLineFound = msg.horizontalToLeft || msg.horizontalToRight;
	horizontalToLeft = msg.horizontalToLeft;
	horizontalToRight = msg.horizontalToRight;
	verticalLineFound = msg.verticalLowerLeftX != -1;
	verticalIntercept = msg.cameraWidth / 2;

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

	lineDetectorMsgReceived = true;
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

string GotoCrossing::name() {
	return string("GotoCrossing");
}

void GotoCrossing::odomTopicCb(const nav_msgs::Odometry& msg) {
	lastOdomMsg_ = msg;
	if (!odometryMessageReceived_) {
		startingPose_ = lastOdomMsg_.pose.pose;
	}

	odometryMessageReceived_ = true;
}

void GotoCrossing::publishCurrentStragety(string strategy) {
	std_msgs::String msg;
	msg.data = strategy;
	if (strategy != lastReportedStrategy_) {
		lastReportedStrategy_ = strategy;
		currentStrategyPub_.publish(msg);
		ROS_INFO("[GotoCrossing] strategy: %s", strategy.c_str());
	}
}

double distanceBetweenPoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
	double result = sqrt((pose1.position.x - pose2.position.x) * (pose1.position.x - pose2.position.x) +
						 (pose1.position.y - pose2.position.y) * (pose1.position.y - pose2.position.y));
	return result;
}

StrategyFn::RESULT_T GotoCrossing::tick() {
	geometry_msgs::Twist		cmdVel;
	RESULT_T 					result = FATAL;
	ostringstream 				ss;
	actionlib_msgs::GoalStatus 	goalStatus;
	double 						zCorrection = ((lastLineDetectorMsg.cameraWidth / 2.0) - verticalIntercept) / 40.0;
	if (zCorrection > 1.0) zCorrection = 1.0;
	if (zCorrection < -1.0) zCorrection = -1.0;

	goalStatus.goal_id.stamp = ros::Time::now();
	goalStatus.goal_id.id = "GotoCrossing";
	goalStatus.status = actionlib_msgs::GoalStatus::ACTIVE;

	if (!strategyContext.needToFollowLine) {
		if (debug_) {
			ROS_INFO("[GotoCrossing] no needToFollowLine");
		}

		result = SUCCESS;
		goalStatus.text = "No need to follow line";
		strategyStatusPublisher_.publish(goalStatus);
		return result;
	}

	// if (!odometryMessageReceived_) {
	// 	if (debug_) ROS_INFO("[GotoCrossing] Waiting for first odometry message");
	// 	goalStatus.text = "Waiting for first odometry message";
	// 	strategyStatusPublisher_.publish(goalStatus);
	// 	result = RUNNING;
	// 	return result;
	// }

	ss << "[x:" << lastOdomMsg_.pose.pose.position.x << ", y:" << lastOdomMsg_.pose.pose.position.y << "]";
	ss << ",state: " << getStateName(state);
	ss << ", verticalLineFound: " << verticalLineFound ? "TRUE" : "false";
	ss << ", sawHorizontalLine: " << sawHorizontalLine ? "TRUE" : "false";
	ss << ", verticalIntercept: " << verticalIntercept;
	ss << ", verticalCurveB: " << lastLineDetectorMsg.verticalCurveB;
	ss << ", zCorrection: " << zCorrection;
	goalStatus.text = ss.str();
	strategyStatusPublisher_.publish(goalStatus);
	ss.str("");
	if (debug_) {
		ROS_INFO("[GotoCrossing] state: %s, verticalLineFound: %s, sawHorizontalLine: %s, verticalIntercept: %7.4f, verticalCurveB: %7.4f, zCorrection: %7.4f",
			state == kLOOKING_FOR_HORIZONTAL_LINE_START ? "kLOOKING_FOR_HORIZONTAL_LINE_START" : "kLOOKING_FOR_HORIZONTAL_LINE_END",
			verticalLineFound ? "TRUE" : "false",
			sawHorizontalLine ? "TRUE" : "false",
		    verticalIntercept,
		    lastLineDetectorMsg.verticalCurveB,
		    zCorrection);
	}

	switch (state) {
		case kLOOKING_FOR_HORIZONTAL_LINE_START:
			ss << "kLOOKING_FOR_HORIZONTAL_LINE_START ";
			publishCurrentStragety(strategyLookingForHorizontalLineStart);
			if (horizontalLineFound) {
				state = kLOOKING_FOR_HORIZONTAL_LINE_END;
				strategyContext.needToRotateLeft90 = horizontalToLeft;
				strategyContext.needToRotateRight90 = horizontalToRight;
				sawHorizontalLine = true;
				// Move until not found horizontal line.
				cmdVel.linear.x = 0.1;
				cmdVel.angular.z = zCorrection;
				if (!dontMove_) cmdVelPub_.publish(cmdVel);
				if (debug_) ROS_INFO("GotoCrossing::tick] kLOOKING_FOR_HORIZONTAL_LINE_START found start, cmdVel x: %7.3f, z: %7.3f", cmdVel.linear.x, cmdVel.angular.z);
				ss << "and horizontalLineFound, new kLOOKING_FOR_HORIZONTAL_LINE_END";
				ss << ", moving with x: " << cmdVel.linear.x << " and z: " << cmdVel.angular.z;
				ss << ", needToRotateLeft90: " << strategyContext.needToRotateLeft90 ? "TRUE" : "false";
				ss << ", needToRotateRight90: " << strategyContext.needToRotateRight90 ? "TRUE" : "false";
				result = RUNNING;
			} else {
				// Move until found horizontal line.
				geometry_msgs::Twist cmdVel;
				cmdVel.linear.x = 0.1;
				cmdVel.angular.z = zCorrection;
				if (!dontMove_) cmdVelPub_.publish(cmdVel);
				if (debug_) ROS_INFO("GotoCrossing::tick] kLOOKING_FOR_HORIZONTAL_LINE_START looking, cmdVel x: %7.3f, z: %7.3f", cmdVel.linear.x, cmdVel.angular.z);
				ss << "verticalLineFound but no horizontalLineFound";
				ss << ", moving with x: " << cmdVel.linear.x << " and z: " << cmdVel.angular.z;
				result = RUNNING;
			}

			break;

		case kLOOKING_FOR_HORIZONTAL_LINE_END:
			ss << "kLOOKING_FOR_HORIZONTAL_LINE_END ";
			publishCurrentStragety(strategyLookingForHorizontalLineEnd);
			if (horizontalLineFound) {
				// Move until no longer see horizontal line.
				cmdVel.linear.x = 0.1;
				cmdVel.angular.z = 0.0;
				if (!dontMove_) cmdVelPub_.publish(cmdVel);
				if (debug_) ROS_INFO("GotoCrossing::tick] kLOOKING_FOR_HORIZONTAL_LINE_END looking for end of horizontal line, cmdVel x: %7.3f, z: %7.3f", cmdVel.linear.x, cmdVel.angular.z);
				strategyContext.needToRotateLeft90 |= horizontalToLeft;
				strategyContext.needToRotateRight90 |= horizontalToRight;
				ss << " horizontalLineFound";
				ss << ", moving with x: " << cmdVel.linear.x << " and z: " << cmdVel.angular.z;
				ss << ", needToRotateLeft90: " << strategyContext.needToRotateLeft90 ? "TRUE" : "false";
				ss << ", needToRotateRight90: " << strategyContext.needToRotateRight90 ? "TRUE" : "false";
				result = RUNNING;
			} else {
				// Time to center over line.
				endingPose_ = lastOdomMsg_.pose.pose;
				distanceTraveledToLineEnd_ = distanceBetweenPoses(startingPose_, endingPose_);
				if (debug_) ROS_INFO("[GotoCrossing] kLOOKING_FOR_HORIZONTAL_LINE_END found end of horizontal line, distance traveled to line end: %7.4f", distanceTraveledToLineEnd_);
				state = kMOVING_TO_CENTERING_POSITION;
				ss << "NOT horizontalLineFound so kMOVING_TO_CENTERING_POSITION next";
				ss << ", distanceTraveledToLineEnd_: " << distanceTraveledToLineEnd_;
				result = RUNNING;
			}

			break;

		case kMOVING_TO_CENTERING_POSITION:
			ss << "kMOVING_TO_CENTERING_POSITION ";
			publishCurrentStragety(strategyMovingToCenteringPosition);
			double totalPostTraveledDistance = distanceBetweenPoses(endingPose_, lastOdomMsg_.pose.pose);
			if (debug_) {
				ROS_INFO("[GotoCrossing] looking for center, totalPostTraveledDistance: %7.4f, distanceTraveledToLineEnd_: %7.4f",
						 totalPostTraveledDistance,
						 distanceTraveledToLineEnd_);
			}

			if (totalPostTraveledDistance  < 0.0921) {
				// Need to go further to find centering position.
				cmdVel.linear.x = 0.1;
				cmdVel.angular.z = 0.0;
				if (!dontMove_) cmdVelPub_.publish(cmdVel);
				if (debug_) ROS_INFO("GotoCrossing::tick] kMOVING_TO_CENTERING_POSITION move further, cmdVel x: %7.3f, z: %7.3f", cmdVel.linear.x, cmdVel.angular.z);
				ss << "not traveled far enough yet";
				ss << ", totalPostTraveledDistance: " << totalPostTraveledDistance;
				ss << ", moving with x: " << cmdVel.linear.x << " and z: " << cmdVel.angular.z;
				result = RUNNING;
			} else {
				publishCurrentStragety(strategySuccess);

				// Stop.
				cmdVel.linear.x = 0.0;
				cmdVel.angular.z = 0.0;
				if (!dontMove_) cmdVelPub_.publish(cmdVel);
				if (debug_) ROS_INFO("GotoCrossing::tick] kMOVING_TO_CENTERING_POSITION stop, cmdVel x: %7.3f, z: %7.3f", cmdVel.linear.x, cmdVel.angular.z);
				ss << "completed centering move, stopping";
				result = SUCCESS;
				strategyContext.needToFollowLine = false;
			}

			break;

		otherwise:
			ss << "UNKNOWN STATE";
			break;
	}

	goalStatus.text = ss.str();
	strategyStatusPublisher_.publish(goalStatus);
	return result;
}

StrategyContext& GotoCrossing::strategyContext = StrategyContext::Singleton();

const string GotoCrossing::strategyHasntStarted = "GotoCrossing: Strategy hasn't started";
const string GotoCrossing::strategyLookingForHorizontalLineEnd = "GotoCrossing: Looking for horizontal line end";
const string GotoCrossing::strategyLookingForHorizontalLineStart = "GotoCrossing: Looking for horizontal line start";
const string GotoCrossing::strategyMovingToCenteringPosition = "GotoCrossing: Moving to centering position";
const string GotoCrossing::strategySuccess = "GotoCrossing: SUCCESS";
