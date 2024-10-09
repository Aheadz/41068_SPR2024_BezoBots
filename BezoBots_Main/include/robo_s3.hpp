#ifndef ROS2_S3_NODE_HPP_
#define ROS2_S3_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <opencv2/opencv.hpp>
#include <cmath>

class Sprint3 : public rclcpp::Node
{

public:
    /**
     * @brief Construct a new Sprint 3 object
     *
     */
    Sprint3();

    /**
     * @brief Destroy the Sprint 3 object
     *
     */
    ~Sprint3();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr &grid);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void check_newPoints(std :; vector<std::pair<double, double>> circle_points);
    std::vector<std::pair<double, double>> detectCircleClusters(std::vector<std::pair<double, double>> points);
    void detectArc(const std::vector<std::pair<double, double>> &cluster, std::vector<std::pair<double, double>> &circle_centers);
    bool calculateCircleFromThreePoints(const std::pair<double, double> &p1,
                                        const std::pair<double, double> &p2,
                                        const std::pair<double, double> &p3,
                                        double &radius,
                                        std::pair<double, double> &center);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void drawOnMap(double x, double y);
    void convertToMapFrame(std::vector<std::pair<double, double>> &points);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    geometry_msgs::msg::Pose current_pose_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    cv::Mat map_image_;
    bool map_received_;
    const double cylinder_diameter_;

    std::vector<std::pair<double, double>> detected_circles_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif