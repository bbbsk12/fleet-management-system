#include "fleet_manager/traffic_manager.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <queue>
#include <set>

namespace fleet_manager
{
TrafficManager::TrafficManager(rclcpp::Node * node)
: node_(node)
{
}

}  // namespace fleet_manager
