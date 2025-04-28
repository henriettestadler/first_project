#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <cmath>

class GpsOdometer {
public:
    GpsOdometer() : has_reference_(false), has_last_position_(false) {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // Get reference GPS coordinates from launch file or parameter server
        pnh.getParam("lat_r", lat_ref_);
        pnh.getParam("lon_r", lon_ref_);
        pnh.getParam("alt_r", alt_ref_);

        // Precompute reference ECEF
        gpsToECEF(lat_ref_, lon_ref_, alt_ref_, x_ref_, y_ref_, z_ref_);
        has_reference_ = true;

        sub_ = nh.subscribe("/swiftnav/front/gps_pose", 10, &GpsOdometer::callback, this);
        pub_ = nh.advertise<nav_msgs::Odometry>("/gps_odom", 10);
    }

    void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (!has_reference_) return;

        // Convert current GPS to ECEF
        double x, y, z;
        gpsToECEF(msg->latitude, msg->longitude, msg->altitude, x, y, z);

        // Convert ECEF to ENU
        double enu_x, enu_y, enu_z;
        ecefToENU(x, y, z, enu_x, enu_y, enu_z);

        // Compute heading
        double heading = 0.0;
        if (has_last_position_) {
            double dx = enu_x - last_enu_x_;
            double dy = enu_y - last_enu_y_;
        if (dx != 0.0 || dy != 0.0) {
            heading = std::atan2(dy, dx);
            } else {
            heading = last_heading_; // re-use
        }
        }
        last_enu_x_ = enu_x;
        last_enu_y_ = enu_y;
        last_heading_ = heading;
        has_last_position_ = true;

        // Broadcast TF
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(enu_x, enu_y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, heading);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "gps"));

        // Publish Odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "gps";

        odom.pose.pose.position.x = enu_x;
        odom.pose.pose.position.y = enu_y;
        odom.pose.pose.position.z = enu_z;

        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);

        pub_.publish(odom);
}

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    double lat_ref_, lon_ref_, alt_ref_;
    double x_ref_, y_ref_, z_ref_;
    bool has_reference_;

    double last_enu_x_, last_enu_y_, last_heading_;
    bool has_last_position_;

    // WGS84 ellipsoid constants
    const double a = 6378137.0;
    const double b = 6356752.0;
    const double e_sq = (a * a - b * b) / (a * a);

    void gpsToECEF(double lat, double lon, double alt, double& x, double& y, double& z) {
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        double N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));
        x = (N + alt) * cos(lat_rad) * cos(lon_rad);
        y = (N + alt) * cos(lat_rad) * sin(lon_rad);
        z = ((1 - e_sq) * N + alt) * sin(lat_rad);
        }

    void ecefToENU(double x, double y, double z, double& enu_x, double& enu_y, double& enu_z) {
        double lat_rad = lat_ref_ * M_PI / 180.0;
        double lon_rad = lon_ref_ * M_PI / 180.0;

        double dx = x - x_ref_;
        double dy = y - y_ref_;
        double dz = z - z_ref_;

        enu_x = -sin(lon_rad) * dx + cos(lon_rad) * dy;
        enu_y = -sin(lat_rad) * cos(lon_rad) * dx - sin(lat_rad) * sin(lon_rad) * dy + cos(lat_rad) * dz;
        enu_z = cos(lat_rad) * cos(lon_rad) * dx + cos(lat_rad) * sin(lon_rad) * dy + sin(lat_rad) * dz;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    GpsOdometer gps_odom;
    ros::spin();
    return 0;
}
