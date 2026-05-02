#include <memory>
#include "fleet_manager/fleet_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fleet_manager::FleetManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}