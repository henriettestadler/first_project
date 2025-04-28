#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <first_project/sector_times.h>
#include <cmath>

class SectorTimesNode {
public:
SectorTimesNode() : last_sector_(-1), distance_total_(0.0), speed_sum_(0.0), speed_count_(0) {
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
return;
}

// Convert current GPS to ENU
double x, y, z;
double ecef_x, ecef_y, ecef_z;
gpsToECEF(msg->latitude, msg->longitude, msg->altitude, ecef_x, ecef_y, ecef_z);
ecefToENU(ecef_x, ecef_y, ecef_z, x, y, z);

// Compute distance from last GPS
double dx = x - last_x_;
double dy = y - last_y_;
double segment_dist = sqrt(dx * dx + dy * dy);
distance_total_ += segment_dist;

int current_sector = static_cast<int>(distance_total_ / sector_length_);
ros::Time current_time = msg->header.stamp;

if (current_sector != last_sector_) {
// Sector changed, publish info for previous
if (last_sector_ >= 0) {
first_project::sector_times msg;
msg.current_sector = last_sector_;
msg.current_sector_time = (current_time - sector_start_time_).toSec();
msg.current_sector_mean_speed = (speed_count_ > 0) ? speed_sum_ / speed_count_ : 0.0f;
sector_pub_.publish(msg);
}

// Reset for new sector
sector_start_time_ = current_time;
speed_sum_ = 0.0;
speed_count_ = 0;
last_sector_ = current_sector;
}

last_x_ = x;
last_y_ = y;
last_time_ = current_time;
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

// Reference GPS
double ref_lat_, ref_lon_, ref_alt_;
double ref_x_, ref_y_, ref_z_;
bool has_reference_ = false;

// ENU last position
double last_x_ = 0.0, last_y_ = 0.0;
ros::Time last_time_;
ros::Time sector_start_time_;

int last_sector_;
double distance_total_;
double speed_sum_;
int speed_count_;
const double sector_length_ = 50.0; // meters

// WGS84 constants
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
