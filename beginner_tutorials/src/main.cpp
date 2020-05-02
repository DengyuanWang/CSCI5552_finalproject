#include <vector>
#include <string>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"


void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
// subscribe to the content in topic /Odom
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;
	ROS_INFO("x: %f, y: %f",x,y);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"beginner_tutorials");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
	//10 is the queue size

	ros::Subscriber sub = nh.subscribe("odom", 10 , OdomCallback);


	//Sets up the random number generator
     	srand(time(0));

     	//Sets the loop to publish at a rate of 10Hz
     	ros::Rate rate(10);

        while(ros::ok()) {
            //Declares the message to be sent
          geometry_msgs::Twist msg;
           //Random x value between -2 and 2
          msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
           //Random y value between -3 and 3
          msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
           //Publish the message
          pub.publish(msg);
	  ros::spinOnce();
          //Delays untill it is time to send another message
          rate.sleep();
        }
	return 0;
}
