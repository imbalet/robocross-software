#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <sstream>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

std::vector<double> euler_from_quaternion(double x, double y, double z,
                                          double w) {
  double const PI = 3.1415926535;
  std::vector<double> euler(3);
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  double roll = std::atan2(sinr_cosp, cosr_cosp);
  euler[0] = roll;

  double sinp = std::sqrt(1 + 2 * (w * y - x * z));
  double cosp = std::sqrt(1 - 2 * (w * y - x * z));
  double pitch = 2 * std::atan2(sinp, cosp) - PI / 2;
  euler[1] = pitch;

  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  euler[2] = yaw;

  return euler;
}

class Mapping : public rclcpp::Node {
public:
  Mapping() : Node("mapping") {

    this->declare_parameter("frequency", 30);
    this->declare_parameter("front_scan_topic", "/front_camera/scan");
    this->declare_parameter("rear_scan_topic", "/rear_camera/scan");
    this->declare_parameter("front_scan_position",
                            std::vector<double>{2.2, 0.0, 0.0});
    this->declare_parameter("rear_scan_position",
                            std::vector<double>{-2.2, 0.0, 0.0});
    this->declare_parameter("map_topic", "/map");
    this->declare_parameter("map_resolution", 0.1);
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("map_infiltration_radius", 2.5);
    this->declare_parameter("map_size", 40);
    this->declare_parameter("local_map_size", 40);
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("robot_base_frame", "chassis");
    this->declare_parameter("goal_topic", "/goal_pose");
    this->declare_parameter("path_topic", "/path");
    this->declare_parameter("path_base_frame", "map");
    this->declare_parameter("path_collision_radius", 1.5);
    this->declare_parameter("goal_radius", 2.0);
    this->declare_parameter("steering_value", 0.35);
    this->declare_parameter("path_discrete", 1.5);
    this->declare_parameter("timeout", 0.2);

    int frequency = this->get_parameter("frequency").as_int();
    std::string front_scan_topic =
        this->get_parameter("front_scan_topic").as_string();
    std::string rear_scan_topic =
        this->get_parameter("rear_scan_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string map_topic = this->get_parameter("map_topic").as_string();
    std::string goal_topic = this->get_parameter("goal_topic").as_string();
    std::string path_topic = this->get_parameter("path_topic").as_string();
    robotBaseFrame = this->get_parameter("robot_base_frame").as_string();
    mapFrame = this->get_parameter("map_frame").as_string();
    mapSize = this->get_parameter("map_size").as_int();
    localMapSize = this->get_parameter("local_map_size").as_int();
    mapRes = this->get_parameter("map_resolution").as_double();
    robotColRadius = this->get_parameter("map_infiltration_radius").as_double();
    frontScanPos = this->get_parameter("front_scan_position").as_double_array();
    rearScanPos = this->get_parameter("rear_scan_position").as_double_array();
    pathBaseFrame = this->get_parameter("path_base_frame").as_string();
    pathCollisionRad = this->get_parameter("path_collision_radius").as_double();
    goalRad = this->get_parameter("goal_radius").as_double();
    steeringVal = this->get_parameter("steering_value").as_double();
    pathDiscrete = this->get_parameter("path_discrete").as_double();
    timeout = this->get_parameter("timeout").as_double();

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    mainTimer =
        this->create_wall_timer(std::chrono::milliseconds(1000 / frequency),
                                std::bind(&Mapping::timer_callback, this));
    frontScanSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        front_scan_topic, 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          this->front_scan_callback(msg);
        });

    rearScanSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rear_scan_topic, 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          this->rear_scan_callback(msg);
        });

    odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          this->odom_callback(msg);
        });

    mapPub =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, 10);

    mapArray.resize(std::pow(mapSize / mapRes, 2), FREE_CELL);
    robotPos.resize(2, 0);
  }

private:
  const unsigned char FREE_CELL = 0;
  const unsigned char UNKNOWN_CELL = 255;
  const unsigned char OBSTACLE_CELL = 100;

  std::string robotBaseFrame;
  std::string mapFrame;
  int mapSize;
  int localMapSize;
  double mapRes;
  double robotColRadius;
  std::vector<double> frontScanPos;
  std::vector<double> rearScanPos;
  std::vector<double> robotPos;
  std::vector<unsigned char> mapArray;
  std::string pathBaseFrame;
  double pathCollisionRad;
  double robotYaw;
  double goalRad;
  double steeringVal;
  double pathDiscrete;
  double timeout;

  rclcpp::TimerBase::SharedPtr mainTimer;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr frontScanSub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rearScanSub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  nav_msgs::msg::Odometry odom_data = nav_msgs::msg::Odometry();
  sensor_msgs::msg::LaserScan front_scan_data = sensor_msgs::msg::LaserScan();
  sensor_msgs::msg::LaserScan rear_scan_data = sensor_msgs::msg::LaserScan();

  std::vector<std::vector<double>>
  robot_to_global(std::vector<double> robot_pos, double robot_orientation,
                  std::vector<double> lidar_pos, std::vector<float> distanties,
                  std::vector<double> angles) {
    std::vector<double> lidar_offset_x(angles.size());
    std::vector<double> lidar_offset_y(angles.size());

    for (size_t i = 0; i < angles.size(); ++i) {
      //первод из полярных в декартовы и применение смещения относительно робота
      lidar_offset_x[i] = distanties[i] * cos(angles[i]) + lidar_pos[0];
      lidar_offset_y[i] = distanties[i] * sin(angles[i]) + lidar_pos[1];
    }
    //для матрицы поворота
    double cos_theta = cos(robot_orientation);
    double sin_theta = sin(robot_orientation);

    std::vector<std::vector<double>> global_coords(angles.size(),
                                                   std::vector<double>(2));

    //применяет матрицу поворота и смещение
    //сразу пишет в матрицу вида [[x0, y0],[x1, y1]]
    for (size_t i = 0; i < angles.size(); ++i) {
      global_coords[i][0] = lidar_offset_x[i] * cos_theta -
                            lidar_offset_y[i] * sin_theta + robot_pos[0];

      global_coords[i][1] = lidar_offset_x[i] * sin_theta +
                            lidar_offset_y[i] * cos_theta + robot_pos[1];
    }
    return global_coords;
  }

  void timer_callback() {
    publish_tf();
    scan_to_local_map();
    publish_map();
  }

  void scan_to_local_map() {
    if (front_scan_data.ranges.size() > 0) {
      int map_center = static_cast<int>(mapSize / (2.0 * mapRes));
      std::vector<double> angles(front_scan_data.ranges.size());
      for (size_t i = 0; i < angles.size(); ++i) {
        angles[i] =
            front_scan_data.angle_min + i * front_scan_data.angle_increment;
      }

      auto coords = robot_to_global(robotPos, robotYaw, frontScanPos,
                                    front_scan_data.ranges, angles);

      int base_x = static_cast<int>(map_center + robotPos[0] / mapRes +
                                    frontScanPos[0] / mapRes);
      int base_y = static_cast<int>(map_center + robotPos[1] / mapRes +
                                    frontScanPos[1] / mapRes);

      auto src_point = cv::Point(base_x, base_y);

      cv::Mat image(mapSize / mapRes, mapSize / mapRes, CV_8UC1,
                    mapArray.data());
      std::vector<cv::Point> points;
      for (auto coord : coords) {
        auto x = map_center + static_cast<int>(coord[0] / mapRes);
        auto y = map_center + static_cast<int>(coord[1] / mapRes);
        auto dst_point = cv::Point(x, y);
        if (static_cast<size_t>(std::max(std::abs(x), std::abs(y))) <
            (mapSize / mapRes)) {
          cv::line(image, src_point, dst_point, FREE_CELL, 1);
          points.push_back(dst_point);
          // cv::circle(image, dst_point, robotColRadius / mapRes, 70, -1);
        }
      }
      // for (auto coord : coords) {
      //   auto x = map_center + static_cast<int>(coord[0] / mapRes);
      //   auto y = map_center + static_cast<int>(coord[1] / mapRes);
      //   mapArray[y * (mapSize / mapRes) + x] = OBSTACLE_CELL;
      // }
      for (cv::Point point : points) {
        cv::circle(image, point, robotColRadius / mapRes, 70, -1);
      }
      for (cv::Point point : points) {
        mapArray[point.y * (mapSize / mapRes) + point.x] = OBSTACLE_CELL;
      }
    }
  }

  void front_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    front_scan_data = *msg;
  }
  void rear_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    rear_scan_data = *msg;
  }
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_data = *msg;
    robotPos[0] = msg->pose.pose.position.x;
    robotPos[1] = msg->pose.pose.position.y;
    auto q = msg->pose.pose.orientation;
    robotYaw = euler_from_quaternion(q.x, q.y, q.z, q.w)[2];
  }

  void publish_tf() {
    auto msg = geometry_msgs::msg::TransformStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = mapFrame;
    msg.child_frame_id = robotBaseFrame;
    msg.transform.translation.x = odom_data.pose.pose.position.x;
    msg.transform.translation.y = odom_data.pose.pose.position.y;
    msg.transform.translation.z = odom_data.pose.pose.position.z + 1.065;
    msg.transform.rotation = odom_data.pose.pose.orientation;
    tf_broadcaster->sendTransform(msg);
  }

  void publish_map() {

    // std::vector<signed char> data(
    //     reinterpret_cast<signed char *>(mapArray.data()), mapArray.size());

    // for (size_t i = 0; i < mapArray.size(); ++i) {
    //   if (mapArray[i] > 127) {
    //     data[i] = 100;
    //   } else {
    //     data[i] = static_cast<signed char>(mapArray[i]);
    //   }
    // }

    std::vector<signed char> data(mapArray.begin(), mapArray.end());

    auto map = nav_msgs::msg::OccupancyGrid();
    map.data = data;
    map.header.stamp = this->get_clock()->now();
    map.header.frame_id = mapFrame;
    map.info.width = mapSize / mapRes;
    map.info.height = mapSize / mapRes;
    map.info.resolution = mapRes;
    map.info.origin.position.x = -mapSize / 2.0;
    map.info.origin.position.y = -mapSize / 2.0;
    mapPub->publish(map);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mapping>());
  rclcpp::shutdown();
  return 0;
}
