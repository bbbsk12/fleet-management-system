#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"

namespace fleet_manager
{

// ==================== Services ====================

void FleetManagerNode::handle_submit_task(
  const std::shared_ptr<fleet_msgs::srv::SubmitTask::Request> req,
  std::shared_ptr<fleet_msgs::srv::SubmitTask::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  res->task_id = scheduler_->submit_task(
    req->waypoint_id, req->priority, req->robot_id,
    req->task_type, req->site_code);
  res->success = !res->task_id.empty();
  res->message = res->success ? "submitted" : "failed";
  if (res->success) {
    auto ti = scheduler_->get_task_info(res->task_id);
    if (!ti.task_id.empty()) task_pub_->publish(ti);
  }
}

void FleetManagerNode::handle_cancel_task(
  const std::shared_ptr<fleet_msgs::srv::CancelTask::Request> req,
  std::shared_ptr<fleet_msgs::srv::CancelTask::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto task = scheduler_->get_task_info(req->task_id);
  if (task.task_id.empty()) { res->success = false; res->message = "not found"; return; }

  scheduler_->cancel_task(req->task_id);
  fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(req->task_id);
  task_pub_->publish(ti);

  // stop robot if it's executing this task
  for (auto & [rid, ni] : navs_) {
    if (!ni || ni->current_task_id != req->task_id) continue;
    cancel_goals(ni);
    stop_robot(rid, 20);
    ni->current_task_id.clear();
    ni->route.clear();
    ni->route_index = 0;
    ni->retry_count = 0;
    ni->chassis_task_sent = false;
    ni->chassis_handshake_ok = false;
    occupancy_->release_reservations(rid);
  }

  res->success = true;
  res->message = "cancelled";
}

void FleetManagerNode::handle_get_robot_status(
  const std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Request> req,
  std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (req->robot_id.empty()) {
    for (const auto & [_, st] : robots_) res->robots.push_back(st);
    res->success = true;
  } else {
    auto it = robots_.find(req->robot_id);
    if (it != robots_.end()) { res->robots.push_back(it->second); res->success = true; }
    else { res->success = false; res->message = "not found"; }
  }
}

void FleetManagerNode::handle_load_traffic_map(
  const std::shared_ptr<fleet_msgs::srv::LoadTrafficMap::Request> req,
  std::shared_ptr<fleet_msgs::srv::LoadTrafficMap::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  res->success = traffic_->load_map(req->file_path);
  res->message = res->success ? "loaded" : "failed";
  if (res->success) {
    occupancy_->set_topology(
      traffic_->get_adjacency_map(),
      [this](const std::string & id) { return traffic_->get_waypoint_pose(id); },
      [this](const std::string & id) { return traffic_->get_waypoint_radius(id); });
    traffic_->validate_waypoint_spacing(waypoint_radius_ * 2.0);
  }
}

void FleetManagerNode::handle_save_traffic_map(
  const std::shared_ptr<fleet_msgs::srv::SaveTrafficMap::Request> req,
  std::shared_ptr<fleet_msgs::srv::SaveTrafficMap::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  res->success = traffic_->save_map(req->file_path);
  res->message = res->success ? "saved" : "failed";
}

void FleetManagerNode::handle_remove_robot(
  const std::shared_ptr<fleet_msgs::srv::RemoveRobot::Request> req,
  std::shared_ptr<fleet_msgs::srv::RemoveRobot::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  const std::string & rid = req->robot_id;
  if (rid.empty()) { res->success = false; res->message = "empty id"; return; }

  // cancel all tasks
  for (const auto & t : scheduler_->get_all_tasks()) {
    if (t.assigned_robot_id == rid && t.status != "completed" &&
        t.status != "failed" && t.status != "cancelled") {
      scheduler_->fail_task(t.task_id, "robot removed");
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
      task_pub_->publish(ti);
    }
  }

  auto ni = get_or_create_nav(rid);
  if (ni) {
    cancel_goals(ni);
    ni->current_task_id.clear();
    ni->route.clear();
    ni->route_index = 0;
    ni->chassis_task_sent = false;
    ni->chassis_handshake_ok = false;
  }

  occupancy_->clear_robot(rid);
  robots_.erase(rid);
  navs_.erase(rid);
  stop_robot(rid, 5);
  removed_.insert(rid);

  PersistLogger::log_warn("robot.removed", rid, "",
    "removed from fleet", __FILE__, __LINE__, __func__);

  res->success = true;
  res->message = "removed";
}

void FleetManagerNode::publish_traffic_fleet_status()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (!has_fleet_) return;

  fleet_msgs::msg::FleetStatus out;
  out.timestamp = this->now();
  out.pending_tasks = last_fleet_.pending_tasks;
  out.active_tasks = last_fleet_.active_tasks;
  out.system_status = last_fleet_.system_status;

  for (auto & [id, st] : robots_) {
    if (is_robot_executing(id)) st.nav_status = "executing";

    // update planned route from nav state
    auto ni = navs_.find(id);
    if (ni != navs_.end() && ni->second) {
      st.planned_route = ni->second->route;
      if (!ni->second->current_task_id.empty())
        st.current_task_id = ni->second->current_task_id;
    }

    out.robots.push_back(st);
  }

  traffic_pub_->publish(out);
}

}  // namespace fleet_manager
