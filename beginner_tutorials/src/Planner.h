#include "header.h"


class Planner{
	public: bool is_finished(){return false;};
	public: geometry_msgs::Twist do_planning(){
		geometry_msgs::Twist msg;
		return msg;
	};
};

