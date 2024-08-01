#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

class CloudToScan : public rclcpp::Node {
public:
  CloudToScan() : Node("cloud_to_scan") {
    this->declare_parameter("scan_topic", "/front_camera/scan");
    this->declare_parameter("cloud_topic", "/front_camera/points");
    this->declare_parameter("frequency", 30);
    this->declare_parameter("x_max", 20.0);
    this->declare_parameter("x_min", 0.7);
    this->declare_parameter("y_max", 100.);
    this->declare_parameter("y_min", -100.);
    this->declare_parameter("z_max", 0.0);
    this->declare_parameter("z_min", -0.3);
    this->declare_parameter("angle_max", 0.7);
    this->declare_parameter("angle_min", -0.7);
    this->declare_parameter("rays_number", 300);
    this->declare_parameter("frame_id", "front_camera_link");

    xMax = this->get_parameter("x_max").as_double();
    xMin = this->get_parameter("x_min").as_double();
    yMax = this->get_parameter("y_max").as_double();
    yMin = this->get_parameter("y_min").as_double();
    zMax = this->get_parameter("z_max").as_double();
    zMin = this->get_parameter("z_min").as_double();
    angleMax = this->get_parameter("angle_max").as_double();
    angleMin = this->get_parameter("angle_min").as_double();
    raysNum = this->get_parameter("rays_number").as_int();
    frameId = this->get_parameter("frame_id").as_string();
    frequency = this->get_parameter("frequency").as_int();
    scan_topic = this->get_parameter("scan_topic").as_string();
    cloud_topic = this->get_parameter("cloud_topic").as_string();

    angleIncrement = (abs(angleMin) + abs(angleMax)) / raysNum;

    pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->pc_callback(msg);
        });

    scan_pub =
        this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);

    scan_timer =
        this->create_wall_timer(std::chrono::milliseconds(1000 / frequency),
                                std::bind(&CloudToScan::timer_callback, this));
  }

private:
  /*----------------------------------*/
  /*            PARAMS                */
  /*----------------------------------*/
  double xMax;
  double xMin;
  double yMax;
  double yMin;
  double zMax;
  double zMin;
  double angleMax;
  double angleMin;
  double angleIncrement;
  int raysNum;
  int frequency;
  std::string frameId;
  std::string scan_topic;
  std::string cloud_topic;

  /*----------------------------------*/
  /*        SUBS, PUBS, TIMERS        */
  /*----------------------------------*/
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::TimerBase::SharedPtr scan_timer;

  /*----------------------------------*/
  /*          OTHER VARIABLES         */
  /*----------------------------------*/
  sensor_msgs::msg::PointCloud2 pcData;

  /*----------------------------------*/
  /*            METHODS               */
  /*----------------------------------*/

  void timer_callback() {
    if (pcData.data.size() == 0) {
      return;
    }
    std::vector<float> new_data(raysNum);
    const size_t number_of_points = pcData.height * pcData.width;
    sensor_msgs::PointCloud2Iterator<float> iter_x(pcData, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pcData, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pcData, "z");
    for (size_t i = 0; i < number_of_points;
         ++i, ++iter_x, ++iter_y, ++iter_z) {
      double x = *iter_x;
      double y = *iter_y;
      double z = *iter_z;
      if (x > xMin && x < xMax && y > yMin && y < yMax && z > zMin &&
          z < zMax) {
        double rho = std::sqrt(pow(x, 2.0) + pow(y, 2.0));
        double phi = std::atan(y / x);
        int ind = static_cast<int>((phi - angleMin) / angleIncrement) + 1;
        if (ind >= 0 && ind < raysNum && new_data[ind] == 0) {
          new_data[ind] = rho;
        }
      }
    }

    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = this->get_clock()->now();
    scan.header.frame_id = frameId;
    scan.angle_min = angleMin;
    scan.angle_max = angleMax;
    scan.angle_increment = angleIncrement;
    scan.range_min = xMin;
    scan.range_max = xMax;
    scan.ranges = new_data;
    scan_pub->publish(scan);
    // RCLCPP_INFO(this->get_logger(), "I published)");
  }

  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcData = *msg;
    // RCLCPP_INFO(this->get_logger(), "I heard");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudToScan>());
  rclcpp::shutdown();
  return 0;
}