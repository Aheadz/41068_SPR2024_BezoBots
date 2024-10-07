#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/utils.h"

#include "robo_s3.hpp"

Sprint3::Sprint3() : Node("robo_s3") {

}

Sprint3::~Sprint3() {

}