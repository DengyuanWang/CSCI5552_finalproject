#include "header.h"
using namespace std;

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"


#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "/scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(1));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",10);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
/*
    bool tag = true;
    sensor_msgs::PointCloud cloud;

   	tag = false;
	try
	{
	projector_.transformLaserScanToPointCloud(
	  "world",*scan_in, cloud,listener_);
	}
	catch (tf::TransformException& e)
	{
	std::cout << e.what();
	tag = true;
	}
  

    
    // Do something with cloud.

    scan_pub_.publish(cloud);
*/
if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/world",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/world",*scan_in,
          cloud,listener_);
  scan_pub_.publish(cloud);
  // Do something with cloud.
  }
};
