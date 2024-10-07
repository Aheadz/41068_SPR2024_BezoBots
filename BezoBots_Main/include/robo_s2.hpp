#ifndef ROS2_S2_NODE_HPP_
#define ROS2_S2_NODE_HPP_


#include <random>
#include <chrono>
#include <cmath>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>


class Sprint2 : public rclcpp::Node
{

public:
    Sprint2();
private:
    void timer_callback();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    
    void publish_scan_matching_pose();
    void publish_cmd_vel();
    void calculate_and_publish_rmse_gt_amcl();
    void calculate_and_publish_rmse_gt_scanmatching_();

    bool check_for_obstacles(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void extract_and_process_images();
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg);
    void estimate_and_correct_rotation();
    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints);
    void rotate_robot(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void stop_robot();
    double normalize_angle(double angle);
    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid);
    cv::Mat extract_map_section();
    cv::Mat apply_canny_edge_detection(const cv::Mat &image);
    cv::Mat laser_scan_to_image(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    double calculate_rmse(double gt_val, double estimated_val);

    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scan_matching_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rmse_pub_gt_amcl_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rmse_pub_gt_scanmatching_;
    rclcpp::TimerBase::SharedPtr timer_;


    // Mutex for synchronizing access to images between callbacks and display
    std::mutex image_mutex_;
    
    double linear_speed_;
    double angular_speed_;
    double distance_;
    std::string direction_;
    bool initialized_;
    bool previous_scan_available_;
    bool match_found_;

    double initial_pose_x_, initial_pose_y_, initial_pose_theta_;
    double current_pose_x_, current_pose_y_, current_pose_theta_;
    std::optional<geometry_msgs::msg::Point> amcl_pose_;
    std::optional<geometry_msgs::msg::Point> ground_truth_pose_;
    std::optional<geometry_msgs::msg::Point> scan_matching_pose_;

    rclcpp::Time last_time_;

    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    cv::Mat scan_image_, edge_map_;
    cv::Mat map_section_; // Declare map_section_ for storing Image A

    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;

    const std::string WINDOW1 = "Map Image";

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> noise_dist_;
    nav_msgs::msg::OccupancyGrid map_; // Store the map data
};

#endif