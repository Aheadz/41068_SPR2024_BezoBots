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
#include "tf2/impl/utils.h"
#include <cmath>
#include <vector>

/**
 * @class Sprint3
 * @brief A ROS2 node for processing laser scan, odometry, and occupancy grid data to detect and map cylindrical objects.
 *
 * This node subscribes to the laser scan, occupancy grid, and odometry topics to detect cylinder-like objects (circles) which are 30cm in diameter
 * in the project's environment. Detected circles are then transformed into the map frame and visualized on the occupancy grid (with the position).
 */
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
    /**
     * @brief Callback for occupancy grid map data
     * @param msg Shared pointer to the occupancy grid message
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief Converts occupancy grid to OpenCV image
     * The occupancy grid is visualized as an image where free space, obstacles
     * and unknown areas are representedby different colors
     * @param grid Shared pointer to the occupancy grid message
     * @return OpenCV image representing the grid map.
     */
    cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr &grid);
    
    /**
     * @brief Callback for processing laser scan data
     * The function is used to detect potential cylinder-like objects in the environment 
     * by clustering laser scan points and checking for circular patterns
     * @param msg Shared pointer to the laser scan message
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * @brief Checks for newly detected circles
     * This function compares newly detected circle points with existing circles
     * adding new circles if they do not match any previously detected ones into the grid map
     * @param circle_points Vector of newly detected circle centers
     */
    void checknewPoints(std::vector<std::pair<double, double>> circle_points);
    
    /**
     * @brief Detects clusters of points that form circular patterns
     * This function groups nearby points from the laser scan into clusters and checks for circular patterns indicating cylindrical objects
     * @param points A vector of 2D points from the laser scan
     */
    std::vector<std::pair<double, double>> detectCircleClusters(std::vector<std::pair<double, double>> points);
    
    /**
     * @brief Detects circular arcs from a cluster of points
     * Analyzes a cluster of laser scan points to check if they form a circular arc, which indicates a cylinder
     * @param cluster A vector of points forming a cluster
     * @param circle_centers A reference to the vector where detected circle centers will be stored
     */
    void detectArc(const std::vector<std::pair<double, double>> &cluster, std::vector<std::pair<double, double>> &circle_centers);
    
    /**
     * @brief Calculates the circle parameters from three points
     *
     * Given three points, computes the center and radius of the circle passing through them
     * Returns false if the points are collinear.
     * 
     * @param p1 First point.
     * @param p2 Second point.
     * @param p3 Third point.
     * @param radius Reference to store the computed radius of the circle.
     * @param center Reference to store the computed center of the circle.
     * @return True if the points form a valid circle, false otherwise.
     */
    bool calculateCircleFromThreePoints(const std::pair<double, double> &p1,
                                        const std::pair<double, double> &p2,
                                        const std::pair<double, double> &p3,
                                        double &radius,
                                        std::pair<double, double> &center);
    
    /**
     * @brief Callback for receiving odometry data
     *
     * Uses the odometry data to transform the robot's pose from the "odom" frame to the "map" frame, allowing
     * detected circles to be accurately positioned on the map.
     * 
     * @param msg Shared pointer to the odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief Draws detected cylinders on the map image
     *
     * Draws green circles on the OpenCV image representing the map, marking the positions of detected cylinders.
     * 
     * @param x X coordinate of the detected cylinder in the map frame.
     * @param y Y coordinate of the detected cylinder in the map frame.
     */
    void drawOnMap(double x, double y);

    /**
     * @brief Converts detected points from robot frame to map frame
     *
     * Uses the robot's current pose and transformation data to convert detected circle points from the robot's
     * local frame into the map frame.
     * 
     * @param points Reference to a vector of points that will be transformed into the map frame.
     */
    void convertToMapFrame(std::vector<std::pair<double, double>> &points);

    //Member variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;  ///< Subscriber for receiving laser scan data
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;  ///< Subscriber for receiving occupancy grid map data
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;      ///< Subscriber for receiving odometry data

    geometry_msgs::msg::Pose current_pose_;                                         ///< Robot's current pose in the map frame
    nav_msgs::msg::OccupancyGrid map_msg_;                                          ///< Latest occupancy grid map data
    cv::Mat map_image_;                                                             ///< OpenCV image representing the occupancy grid
    bool map_received_;                                                             ///< Flag indicating if the map has been received
    const double cylinder_diameter_;                                                ///< Diameter of the detected cylinders

    std::vector<std::pair<double, double>> detected_circles_;                       ///< Vector of detected cylinder centers

    tf2_ros::Buffer tf_buffer_;                                                     ///< Buffer for storing transformations
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                       ///< Listener for receiving transformation data
};

#endif