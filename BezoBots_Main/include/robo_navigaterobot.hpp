#ifndef NAVIGATE_ROBOT_HPP
#define NAVIGATE_ROBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"            // Include Odometry message
#include "gazebo_msgs/srv/set_entity_state.hpp" // Include the SetEntityState service
#include "nav_msgs/msg/occupancy_grid.hpp"      // Include OccupancyGrid for map data
#include <opencv2/opencv.hpp>                   // Include OpenCV
#include <cmath>
#include <thread>
#include <mutex>
#include <future>

class NavigateRobot : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigateRobot();
    void sendGoal(int goal_index);
    void moveShelf(const std::string &shelf_name, double x, double y, double z);

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;     // Subscription to odometry data
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;             // Subscription to planned path
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;     // Subscription to map data
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr shelf_service_client_; // Service client for moving the shelf

    // Goal variables
    bool initial_pose_set_;
    int goal_index_;
    // Robot position
    double robot_start_x_, robot_start_y_, robot_start_z;
    double robot_current_x_, robot_current_y_, robot_current_z_;
    // Shelf Position
    double shelf_x, shelf_y, shelf_z;
    // Goals for Shelf 14
    //  Declare poses using geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose pose1, pose2, pose3, pose4;
    //Goals for Shelf 4
    geometry_msgs::msg::Pose pose5, pose6, pose7, pose8;
    // Declare a separate thread variable as part of the class, to handle the shelf movement
    std::thread shelf_movement_thread_;
    std::mutex shelf_movement_mutex_; // Mutex to protect shared variables
    std::future<void> shelf_movement_future_;
    bool keep_moving_shelf_ = false; // A flag to control the shelf movement thread
    const double tolerance = 0.3;    // Tolerance to place the shelf
    // Total distance
    double total_distance_;

    // OpenCV related variables
    cv::Mat map_image_; // OpenCV image for real-time path visualization
    int map_width_;
    int map_height_;
    double scale_; // Scaling factor for converting coordinates to pixels

    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg); // Odometry callback
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);         // Planned path callback
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg); // Map callback
    double calculateTotalDistanceToGoal(double goal_x, double goal_y);
    void drawPlannedPath(const nav_msgs::msg::Path::SharedPtr path);
};

#endif // NAVIGATE_ROBOT_HPP
