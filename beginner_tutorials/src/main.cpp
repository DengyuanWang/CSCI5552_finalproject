#include <vector>
#include <string>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "SLAM.h"
#include "Planner.h"

#include "beginner_tutorials/integrated_msg.h"

using namespace std;
class ROSinterface{
	string cmd_topic;
	ros::NodeHandle node_handle;
	ros::Publisher pub_cmd;
	ros::Subscriber sub_odom,sub_laserscan;
	SLAM slam_handle;
	Planner planner_handle;
	beginner_tutorials::integrated_msg integrated_msg;
	geometry_msgs::Twist cmd_msg;
	
	 void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	// subscribe to the content in topic /Odom
		if(integrated_msg.tag==0){
			integrated_msg.tag=1;
		}else{
			integrated_msg.tag=2;
			integrated_msg.delta_t = msg->header.stamp.toSec()-integrated_msg.time_stamp.toSec();
			integrated_msg.time_stamp = msg->header.stamp;
			if(integrated_msg.delta_t>0){
				integrated_msg.odom = *msg;
			}
			
		}
	} 
	 void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
		// subscribe to the content in topic /scan
		if(integrated_msg.tag==2 && integrated_msg.delta_t>0){
			integrated_msg.layserScan = *msg;
		}
	} 
	public: bool init_start_ros(bool debug_tag){
		integrated_msg.tag = 0;
		integrated_msg.delta_t=-1;
		//Sets up the random number generator
     		srand(time(0));

		pub_cmd = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
		//10 is the queue size
		sub_odom = node_handle.subscribe("odom", 10 , &ROSinterface::OdomCallback,this);
		sub_laserscan = node_handle.subscribe("scan", 10 , &ROSinterface::LaserScanCallback,this);

		return start_ros_loop(debug_tag);

	}

	bool start_ros_loop(bool debug_tag){
		//Sets the loop to publish at a rate of 10Hz
	     	ros::Rate rate(100);
		integrated_msg.time_stamp = ros::Time::now();
		while(ros::ok()) {
		    	
			ros::spinOnce();
			
			if(integrated_msg.tag==2 && integrated_msg.delta_t>0){
				cout<<"delta_t:"<<integrated_msg.delta_t<<" tag:"<<integrated_msg.tag<<endl;
				slam_handle.do_SLAM();
				//Declares the message to be sent
				geometry_msgs::Twist msg = planner_handle.do_planning();
				if(planner_handle.is_finished())
				{
					return true;	
				}
				if(debug_tag){
					//Random x value between -2 and 2
					msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
					//Random y value between -3 and 3
					msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
					//Publish the message
				}
				else{
				
				}

				pub_cmd.publish(msg);
			}
			

			//Delays untill it is time to send another message
			rate.sleep();
		}
		return true;
	}
};




int main(int argc, char** argv){
	
	ros::init(argc,argv,"beginner_tutorials");
	ROSinterface ros_api;
	bool debug_tag = true;
	ros_api.init_start_ros(debug_tag);

     	

	return 0;
}
