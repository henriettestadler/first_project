#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"


//Global 
nav_msgs::Odometry latest_odom;  // Store latest odometry
ros::Time last_time;
ros::Publisher odom_pub;

void computeOdometry(const geometry_msgs::PointStamped::ConstPtr& msg) {

	//Extract time and coordianted
    ros::Time current_time = msg->header.stamp;
    double steer_deg = msg->point.x;
    double speed = msg->point.y;
	if (last_time.isZero()) {
		last_time = current_time;
		return;
	}

	//Odometry: Pose and Twist (with Covariance)
	// Fill odometry: header
    latest_odom.header.stamp = current_time;
    latest_odom.header.frame_id = "odom";
	latest_odom.header.frame_id = msg->header.frame_id;
	
	//double dt = (current_time - last_time).toSec();

	// Fill odometry: pose
    latest_odom.pose.pose.position.x = 0.0;
    latest_odom.pose.pose.position.y = 0.0;
    latest_odom.pose.pose.position.z = 0.0;
    latest_odom.pose.pose.orientation.x = 0.0;
    latest_odom.pose.pose.orientation.y = 0.0;
    latest_odom.pose.pose.orientation.z = 0.0;
    latest_odom.pose.pose.orientation.w = 1.0;

	// Fill odometry: twist
	latest_odom.twist.twist.linear.x = speed;
	latest_odom.twist.twist.linear.y = 0.0;
	latest_odom.twist.twist.linear.z = 0.0;
	latest_odom.twist.twist.angular.x = 0.0;
	latest_odom.twist.twist.angular.y = 0.0;
	latest_odom.twist.twist.angular.z = 0.0;

	// Publish to /odom
	odom_pub.publish(latest_odom);
}
//Subscribing and receiving PointStamp
void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	//Compute Odometry of received PointStamp
    computeOdometry(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/speedsteer", 10, speedsteerCallback);
	odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);

	ros::spin();

	return 0;
}