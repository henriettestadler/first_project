#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <first_project/sector_times.h>
#include <cmath>

/*
INPUT: 
- topic name: /swiftnav/front/gps_pose
	- type: sensor_msgs/NavSatFix
- topic name: /speedsteer
    - type: geometry_msgs/PointStamped 

OUTPUT:
- topic: /sector_times
	- type: first_project/sector_times (custom message, create .msg file)

SECTOR TIMES / END POINTS
- 1 - end: at 142sec 
        latitude: 45.630107929622135
        longitude: 9.2904209270337
        altitude: 238.8514007567726
- 2 -end at 262sec
        latitude: 45.623327601927656
        longitude: 9.28682809188195
        altitude: 231.77824942388858
- 3 - end (using the very first GPS data points)
        latitude: 45.618932386592405
        longitude: 9.281178887031235
        altitude: 229.04906147731415
*/ 

class SectorTimesNode {
public:
    SectorTimesNode() : last_sector_(1), distance_total_(0.0), speed_sum_(0.0), speed_count_(0), has_reference_(false) {
        ros::NodeHandle nh;

        gps_sub_ = nh.subscribe("/swiftnav/front/gps_pose", 10, &SectorTimesNode::gpsCallback, this);
        speed_sub_ = nh.subscribe("/speedsteer", 10, &SectorTimesNode::speedCallback, this);
        sector_pub_ = nh.advertise<first_project::sector_times>("/sector_times", 10);
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (!has_reference_) {
            ref_lat_ = msg->latitude;
            ref_lon_ = msg->longitude;
            ref_alt_ = msg->altitude;
            gpsToECEF(ref_lat_, ref_lon_, ref_alt_, ref_x_, ref_y_, ref_z_);
            has_reference_ = true;
            last_time_ = msg->header.stamp;
            sector_start_time_ = msg->header.stamp;
            
            initializeSectors();
            return;
        }

        double x, y, z;
        double ecef_x, ecef_y, ecef_z;
        gpsToECEF(msg->latitude, msg->longitude, msg->altitude, ecef_x, ecef_y, ecef_z);
        ecefToENU(ecef_x, ecef_y, ecef_z, x, y, z);

        double dx = x - last_x_;
        double dy = y - last_y_;
        double segment_dist = std::sqrt(dx * dx + dy * dy);
        distance_total_ += segment_dist;

        ros::Time current_time = msg->header.stamp;
        last_time_ = current_time;

        int next_sector = (last_sector_ % 3) + 1;
        
        if (distance2D(x, y, sector_start_enu_[next_sector - 1].x, sector_start_enu_[next_sector - 1].y) < switch_radius_) {
            // Sector change
            last_sector_ = next_sector;
            sector_start_time_ = current_time;
            speed_sum_ = 0.0;
            speed_count_ = 0;

            if (last_sector_ == 1) {
                distance_total_ = 0.0;
            }
        }

        first_project::sector_times out_msg;
        out_msg.current_sector = last_sector_;
        out_msg.current_sector_time = (last_time_ - sector_start_time_).toSec();
        out_msg.current_sector_mean_speed = (speed_count_ > 0) ? speed_sum_ / speed_count_ : 0.0f;
        sector_pub_.publish(out_msg);

        last_x_ = x;
        last_y_ = y;
    }

    void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double speed = msg->point.y / 3.6; // km/h to m/s
        speed_sum_ += speed;
        speed_count_++;
    }

private:
    ros::Subscriber gps_sub_;
    ros::Subscriber speed_sub_;
    ros::Publisher sector_pub_;
    //ros::Timer timer_;

    // Reference GPS and ECEF
    double ref_lat_, ref_lon_, ref_alt_;
    double ref_x_, ref_y_, ref_z_;
    bool has_reference_;

    // Last ENU position
    double last_x_ = 0.0, last_y_ = 0.0;
    ros::Time last_time_;
    ros::Time sector_start_time_;

    int last_sector_;
    double distance_total_;
    double speed_sum_;
    int speed_count_;

    struct ENU {
        double x, y, z;
    };

    ENU sector_start_enu_[3];

    const double switch_radius_ = 5.0;

    void initializeSectors() {
        
        double sx1, sy1, sz1, sx2, sy2, sz2, sx3, sy3, sz3;

        gpsToECEF(45.618932386592405, 9.281178887031235, 229.04906147731415, sx1, sy1, sz1);
        ecefToENU(sx1, sy1, sz1, sector_start_enu_[0].x, sector_start_enu_[0].y, sector_start_enu_[0].z); // Sector 1

        gpsToECEF(45.630107929622135, 9.2904209270337, 238.8514007567726, sx2, sy2, sz2);
        ecefToENU(sx2, sy2, sz2, sector_start_enu_[1].x, sector_start_enu_[1].y, sector_start_enu_[1].z); // Sector 2

        gpsToECEF(45.623327601927656, 9.28682809188195, 231.77824942388858, sx3, sy3, sz3);
        ecefToENU(sx3, sy3, sz3, sector_start_enu_[2].x, sector_start_enu_[2].y, sector_start_enu_[2].z); // Sector 3
    }

    double distance2D(double x1, double y1, double x2, double y2) {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    void gpsToECEF(double lat, double lon, double alt, double& x, double& y, double& z) {
        const double a = 6378137.0;
        const double b = 6356752.0;
        const double e_sq = (a * a - b * b) / (a * a);

        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        double N = a / std::sqrt(1 - e_sq * std::sin(lat_rad) * std::sin(lat_rad));
        x = (N + alt) * std::cos(lat_rad) * std::cos(lon_rad);
        y = (N + alt) * std::cos(lat_rad) * std::sin(lon_rad);
        z = ((1 - e_sq) * N + alt) * std::sin(lat_rad);
    }

    void ecefToENU(double x, double y, double z, double& enu_x, double& enu_y, double& enu_z) {
        double lat_rad = ref_lat_ * M_PI / 180.0;
        double lon_rad = ref_lon_ * M_PI / 180.0;

        double dx = x - ref_x_;
        double dy = y - ref_y_;
        double dz = z - ref_z_;

        enu_x = -sin(lon_rad) * dx + cos(lon_rad) * dy;
        enu_y = -sin(lat_rad) * cos(lon_rad) * dx - sin(lat_rad) * sin(lon_rad) * dy + cos(lat_rad) * dz;
        enu_z = cos(lat_rad) * cos(lon_rad) * dx + cos(lat_rad) * sin(lon_rad) * dy + sin(lat_rad) * dz;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_times");
    SectorTimesNode node;
    ros::spin();
    return 0;
}
