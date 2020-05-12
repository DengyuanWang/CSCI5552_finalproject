
#include "SLAM.cpp"
#include "Planner.h"


using namespace std;
class ROSinterface{
	string cmd_topic;
	ros::NodeHandle node_handle;
	ros::Publisher pub_cmd;

	ros::Subscriber sub_odom,sub_laserscan,sub_pcloud;

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

	void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
		// subscribe to the content in topic /scan
		if(integrated_msg.tag==2 && integrated_msg.delta_t>0){
			//ROS_INFO("--scan write--");
			integrated_msg.pcloud = *msg;
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
		sub_odom = node_handle.subscribe("odom", 100 , &ROSinterface::OdomCallback,this);
		sub_laserscan = node_handle.subscribe("scan", 100 , &ROSinterface::LaserScanCallback,this);

		sub_pcloud = node_handle.subscribe("my_cloud", 100 , &ROSinterface::PointCloudCallback,this);

		//visualization for debug
		marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));

		ros::Duration(2).sleep(); 
		LaserScanToPointCloud filter(node_handle);
		

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


	bool gen_points(Eigen::VectorXd x_hat_t){

		visualization_msgs::Marker points;
		points.header.frame_id = "/world";
		points.header.stamp = ros::Time::now();
		points.ns = "points_and_lines";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;
		points.id = 0;
		points.type = visualization_msgs::Marker::POINTS;
		points.scale.x = 0.2;points.scale.y = 0.2;
		points.color.b = 1.0f;points.color.a = 1.0;

		//= integrated_msg.pcloud.points;
		for(int i=0;i<(x_hat_t.size()-3)/2.0;i++){
			geometry_msgs::Point p;
			float x =x_hat_t[3+2*i],
				y =x_hat_t[3+2*i+1];
			p.x = x;p.y=y;p.z=0;
			points.points.push_back(p);
		}
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));
		if (marker_pub.getNumSubscribers() < 1){
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			return false;
		}
		marker_pub.publish(points);
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
	     	ros::Rate rate(70);
		integrated_msg.time_stamp = ros::Time::now();
		while(ros::ok()) {
		    	
			ros::spinOnce();
			
			if(integrated_msg.tag==2 && integrated_msg.delta_t>0){
				cout<<"delta_t:"<<integrated_msg.delta_t<<" tag:"<<integrated_msg.tag<<endl;
				slam_handle.integrated_msg = integrated_msg;
				slam_handle.do_SLAM();

				cout<<"landmark size is"<<(slam_handle.x_hat_t_glob.size()-3)/2.0<<endl;

				if(slam_handle.count==0)
					gen_points(slam_handle.x_hat_t_glob);

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
					msg.linear.x=1*double(rand())/double(RAND_MAX)-0.5;
					//Random y value between -3 and 3
					msg.angular.z=2*double(rand())/double(RAND_MAX)-1;
				}
				else{
				
				}
				show_marker();

				//gen_points();

				//pub_cmd.publish(msg);
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
