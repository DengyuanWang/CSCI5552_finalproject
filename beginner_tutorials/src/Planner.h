#include <vector>
#include <string>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"


class Planner{
	public: bool is_finished(){return false;};
	public: geometry_msgs::Twist do_planning(){
		geometry_msgs::Twist msg;
		return msg;
	};
};

