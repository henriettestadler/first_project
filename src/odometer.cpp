#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

/*
INPUT: 
- topic name: /speedsteer
	- type: geometry_msgs/PointStamped -> point (xyz)
		- x: steer at the steering wheel (deg)
		- y: speed (km/h)

OUTPUT:
- topic: /odom
	- type: nav_msgs/Odometry -> pose (position, orientation) & twist (linear, angular)
        - coordinate system: x to front; y to left; z up (from driver's view)
- tf: odom-vehicle
	- rear wheels baseline: 130 cm
	- distance from front to rear wheels: 176.5 cm
    - steering factor: 32
*/ 

class OdometerNode {
public:
    OdometerNode() : x_(0.0), y_(0.0), theta_(0.0) {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // Load parameters from launch file
        pnh.getParam("wheelbase", wheelbase_); //m
        pnh.getParam("steering_factor", steering_factor_);

        // Subscriber to /speedsteer
        sub_ = nh.subscribe("/speedsteer", 10, &OdometerNode::callback, this);

        // Publisher for /odom
        pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);

        last_time_ = ros::Time::now();
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        ros::Time current_time = msg->header.stamp;

        // Extract values from msg
        double speed_kmh = msg->point.y; //kmh
        double steer_deg = msg->point.x; //deg

        // Convert speed from km/h to m/s
        double v_ms = speed_kmh / 3.6;

        // Convert steer to radians and apply steering factor
        double steer_rad = steer_deg * M_PI / 180.0; //rad
        double delta = steer_rad / steering_factor_;

        // Compute angular velocity
        double omega = v_ms * tan(delta) / wheelbase_;

        // Time difference
        double dt = (current_time - last_time_).toSec();

        // Integrate to get new position
        x_ += v_ms * cos(theta_) * dt;
        y_ += v_ms * sin(theta_) * dt;
        theta_ += omega * dt;

        // Broadcast tf
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x_, y_, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta_);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, current_time, "odom", "vehicle"));

        // Publish Odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "vehicle";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);

        odom.twist.twist.linear.x = v_ms;
        odom.twist.twist.angular.z = omega;

        pub_.publish(odom);

        last_time_ = current_time;
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double x_, y_, theta_;
    ros::Time last_time_;
    double wheelbase_;
    double steering_factor_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometer");
    OdometerNode odometer;
    ros::spin();
    return 0;
}
