#include "rclcpp/rclcpp.hpp"
#include <cstdlib>
#include "robo_s2.hpp"
#include "robo_s3.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //auto node = std::make_shared<Ros2Lab2Node>(n);
  //auto node = std::make_shared<Sprint2>();
  auto node = std::make_shared<Sprint3>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
