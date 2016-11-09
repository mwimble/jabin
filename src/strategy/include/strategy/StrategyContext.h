#ifndef __STRATEGY_CONTEXT
#define __STRATEGY_CONTEXT

#include <sys/time.h>
#include <geometry_msgs/Twist.h>

class StrategyContext {
private:
	// Singleton pattern.
	StrategyContext() :
		atGoal(false),
		needToFollowLine(true),
		needToRotateLeft90(false),
		needToRotateRight90(false) {};
	StrategyContext(StrategyContext const&) {};
	StrategyContext& operator=(StrategyContext const&) {}

public:
	static StrategyContext& Singleton();

	bool atGoal;								// Goal is satisfied.

	bool needToFollowLine;						// Need to follow line.

	bool needToRotateLeft90;					// Need to rotate 180 degrees left.

	bool needToRotateRight90;					// Need to rotate 180 degrees right.

};

#endif
