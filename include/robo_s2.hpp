#ifndef ROS2_S2_NODE_HPP_
#define ROS2_S2_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv4/opencv2/opencv.hpp>

class Sprint2 : public rclcpp::Node
{

public:
    Sprint2();
private:
    void move_turtlebot(double move_f_b, double turn_l_r);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void calculateYawChange();
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints);
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_turtlebot_;
    
    cv::Mat first_image_, second_image_;
    
    
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;
    double relative_orientaion_ = 0.0;

};

#endif