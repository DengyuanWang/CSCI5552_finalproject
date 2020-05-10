
#include "SLAM.cpp"
#include "Planner.h"


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
	queue<visualization_msgs::Marker> Markers;
	ros::Publisher marker_pub;

	tf::TransformBroadcaster br;
	tf::Transform transform;
	int cur_marker_id;

	void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	// subscribe to the content in topic /Odom
		if(integrated_msg.tag==0){
			integrated_msg.tag=1;
		}else{
			integrated_msg.tag=2;
			integrated_msg.delta_t = msg->header.stamp.toSec()-integrated_msg.time_stamp.toSec();
			integrated_msg.time_stamp = msg->header.stamp;
			if(integrated_msg.delta_t>0){
				//ROS_INFO("--odom write--");
				integrated_msg.odom = *msg;

				integrated_msg.u_v = msg->twist.twist.linear.x;
				integrated_msg.u_w = msg->twist.twist.angular.z;
				//gen_marker(msg->pose.pose.position.x,msg->pose.pose.position.y);
			}
			
		}
	} 
	void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
		// subscribe to the content in topic /scan
		if(integrated_msg.tag==2 && integrated_msg.delta_t>0){
			//ROS_INFO("--scan write--");
			integrated_msg.layserScan = *msg;
		}
	} 
	public: bool init_start_ros(bool debug_tag){

		
		transform.setOrigin( tf::Vector3(0.0,0.0, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0,0);
		transform.setRotation(q);
		

		slam_handle.init_SLAM();
		cur_marker_id = 0;
		integrated_msg.tag = 0;
		integrated_msg.delta_t=-1;
		//Sets up the random number generator
     		srand(time(0));

		pub_cmd = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
		//10 is the queue size
		sub_odom = node_handle.subscribe("odom", 10 , &ROSinterface::OdomCallback,this);
		sub_laserscan = node_handle.subscribe("scan", 10 , &ROSinterface::LaserScanCallback,this);
		//visualization for debug
		marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);

		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));
		return start_ros_loop(debug_tag);

	}
	
	bool gen_marker(float x_axis,float y_axis,int id){
		uint32_t shape = visualization_msgs::Marker::CUBE;
		visualization_msgs::Marker marker;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "world";
		marker.header.stamp = ros::Time::now();
		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "basic_shapes";
		marker.id = id;
		//cur_marker_id +=1;
		// Set the marker type.  
		//Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = shape;

		// Set the marker action.  
		//Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  
		//This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = x_axis;
		marker.pose.position.y = y_axis;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 1.0;
		marker.pose.orientation.y = 1.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();
		Markers.push(marker);
		return true;
	}
	
	bool show_marker(){
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));
		if (marker_pub.getNumSubscribers() < 1){
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			return false;
		}else{
			 while (!Markers.empty()) { 
				marker_pub.publish(Markers.front());
				Markers.pop(); 
			    } 
			return true;
		}		
	}
	bool start_ros_loop(bool debug_tag){
		//Sets the loop to publish at a rate of 10Hz
	     	ros::Rate rate(100);
		integrated_msg.time_stamp = ros::Time::now();
		while(ros::ok()) {
		    	
			ros::spinOnce();
			
			if(integrated_msg.tag==2 && integrated_msg.delta_t>0){
				cout<<"delta_t:"<<integrated_msg.delta_t<<" tag:"<<integrated_msg.tag<<endl;
				slam_handle.integrated_msg = integrated_msg;
				slam_handle.do_SLAM();

				cout<<"landmark size is"<<(slam_handle.x_hat_t_glob.size()-3)/2.0<<endl;
				for(int i=0;i<(slam_handle.x_hat_t_glob.size()-3)/2.0;i++){
					float x = slam_handle.x_hat_t_glob[3+2*i],y = slam_handle.x_hat_t_glob[3+2*i+1];
					gen_marker(x,y,i);
				}
				//Declares the message to be sent
				geometry_msgs::Twist msg = planner_handle.do_planning();
				if(planner_handle.is_finished())
				{
					return true;	
				}
				if(debug_tag){
					//Random x value between -2 and 2
					msg.linear.x=0;
					msg.linear.y=0;
					msg.linear.z=0;
					//Random y value between -3 and 3
					msg.angular.x=0;
					msg.angular.y=0;
					msg.angular.z=0.2;
					//Publish the message
				}
				else{
				
				}
				show_marker();
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
