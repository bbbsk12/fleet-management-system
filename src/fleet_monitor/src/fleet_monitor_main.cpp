#include <memory>
#include "fleet_monitor/fleet_monitor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fleet_monitor::FleetMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}