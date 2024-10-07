#ifndef ROS2_S3_NODE_HPP_
#define ROS2_S3_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


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

};

#endif