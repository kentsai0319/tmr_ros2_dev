#include "tmr_driver/tmr_ros2_tmsct.h"


TmSctRos2::TmSctRos2(const rclcpp::NodeOptions &options, tmrl::driver::Driver &iface, bool is_fake)
  : Node("tmr_sct", options)
  , iface_(iface)
  , state_(iface.state)
  , is_fake_(is_fake)
{
  jns_.reserve(tmrl::driver::RobotState::DOF);
  jns_.push_back("joint_1");
  jns_.push_back("joint_2");
  jns_.push_back("joint_3");
  jns_.push_back("joint_4");
  jns_.push_back("joint_5");
  jns_.push_back("joint_6");

  if (!is_fake_) {
    pm_.sct_pub = create_publisher<tmr_msgs::msg::TmsctResponse>("tmsct_response", 1);
    pm_.sta_pub = create_publisher<tmr_msgs::msg::TmstaResponse>("tmsta_response", 1);
  }
  if (!is_fake_) {
    auto ros_ok = [](){ return rclcpp::ok(); };

    iface_.tmsct.set_is_ok_predicate(ros_ok);
    iface_.tmsct.set_tmsct_callback(std::bind(&TmSctRos2::tmsct_cb, this, std::placeholders::_1));
    iface_.tmsct.set_tmsta_callback(std::bind(&TmSctRos2::tmsta_cb, this, std::placeholders::_1));
    iface_.tmsct.start();
  }

  if (!is_fake_) {
    connect_tmr_srv_ = create_service<tmr_msgs::srv::ConnectTMR>(
      "tmr/connect_tmsct", std::bind(&TmSctRos2::connect_tmsct, this,
      std::placeholders::_1, std::placeholders::_2));

    send_script_srv_ = create_service<tmr_msgs::srv::SendScript>(
      "tmr/send_script", std::bind(&TmSctRos2::send_script, this,
      std::placeholders::_1, std::placeholders::_2));
    set_event_srv_ = create_service<tmr_msgs::srv::SetEvent>(
      "tmr/set_event", std::bind(&TmSctRos2::set_event, this,
      std::placeholders::_1, std::placeholders::_2));
    set_io_srv_ = create_service<tmr_msgs::srv::SetIO>(
      "tmr/set_io", std::bind(&TmSctRos2::set_io, this,
      std::placeholders::_1, std::placeholders::_2));
    set_positions_srv_ = create_service<tmr_msgs::srv::SetPositions>(
      "tmr/set_positions", std::bind(&TmSctRos2::set_positions, this,
      std::placeholders::_1, std::placeholders::_2));

    ask_sta_srv_ = create_service<tmr_msgs::srv::AskSta>(
      "tmr/ask_sta", std::bind(&TmSctRos2::ask_sta, this,
      std::placeholders::_1, std::placeholders::_2));
  }

  as_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "tmr_arm_controller/follow_joint_trajectory",
    std::bind(&TmSctRos2::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TmSctRos2::handle_cancel, this, std::placeholders::_1),
    std::bind(&TmSctRos2::handle_accepted, this, std::placeholders::_1)
  );
  goal_id_.clear();
}
TmSctRos2::~TmSctRos2()
{
  std::cout << "TM_ROS: TMSCT halt\n";

  if (is_fake_) return;

  sta_updated_ = true;
  sta_cv_.notify_all();

  if (iface_.tmsct.client().is_connected()) {
    iface_.set_script_exit();
  }
  iface_.tmsct.stop();
}

void TmSctRos2::tmsct_cb(const tmrl::comm::TmsctPacket &pack)
{
  SctMsg &pm = pm_;
  std::unique_lock<std::mutex> lck(sct_mtx_);
  
  pm.sct_msg.id = pack.id();
  pm.sct_msg.script = pack.script();

  sct_updated_ = true;
  lck.unlock();
  sct_cv_.notify_all();

  if (pack.has_error()) {
    tmrl_ERROR_STREAM("$TMSCT: err: " << pm.sct_msg.id << ", " << pm.sct_msg.script);
  }
  else {
    tmrl_INFO_STREAM("$TMSCT: res: " << pm.sct_msg.id << ", " << pm.sct_msg.script);
  }

  pm.sct_msg.header.stamp = rclcpp::Node::now();
  pm.sct_pub->publish(pm.sct_msg);
}

void TmSctRos2::tmsta_cb(const tmrl::comm::TmstaPacket &pack)
{
  SctMsg &pm = pm_;
  std::unique_lock<std::mutex> lck(sta_mtx_);
  
  pm.sta_msg.subcmd = pack.subcmd();
  pm.sta_msg.subdata = pack.subdata();

  sta_updated_ = true;
  lck.unlock();
  sta_cv_.notify_all();

  tmrl_INFO_STREAM("$TMSTA: res: " << pm.sta_msg.subcmd << ", " << pm.sta_msg.subdata);

  pm.sta_msg.header.stamp = rclcpp::Node::now();
  pm.sta_pub->publish(pm.sta_msg);
}

bool TmSctRos2::connect_tmsct(
  const std::shared_ptr<tmr_msgs::srv::ConnectTMR::Request> req,
  std::shared_ptr<tmr_msgs::srv::ConnectTMR::Response> res)
{
  bool rb = true;
  int t_o = (int)(1000.0 * req->timeout);
  int t_v = (int)(1000.0 * req->timeval);

  tmrl::comm::ClientThread *client = &iface_.tmsct;

  if (req->connect) {
    tmrl_INFO_STREAM("TM_ROS: (re)connect(" << t_o << ") TMSCT");
    client->stop();
    rb = client->start(t_o);
  }
  if (req->reconnect) {
    client->set_reconnect_timeout(req->timeout);
    client->set_reconnect_timeval(req->timeval);
    tmrl_INFO_STREAM("TM_ROS: set TMSCT reconnect timeout " << t_o << "ms, timeval " << t_v << "ms");
  }
  else {
    // no reconnect
    client->set_reconnect_timeval(-1);
    tmrl_INFO_STREAM("TM_ROS: set TMSCT NOT reconnect");
  }
  res->ok = rb;
  return rb;
}

bool TmSctRos2::send_script(
  const std::shared_ptr<tmr_msgs::srv::SendScript::Request> req,
  std::shared_ptr<tmr_msgs::srv::SendScript::Response> res)
{
  bool rb = iface_.tmsct.send_script(req->id, req->script);
  res->ok = rb;
  return rb;
}
bool TmSctRos2::set_event(
  const std::shared_ptr<tmr_msgs::srv::SetEvent::Request> req,
  std::shared_ptr<tmr_msgs::srv::SetEvent::Response> res)
{
  bool rb = false;
  std::string content;
  switch (req->func) {
  case tmr_msgs::srv::SetEvent_Request::EXIT:
    rb = iface_.set_script_exit();
    break;
  case tmr_msgs::srv::SetEvent_Request::TAG:
    rb = iface_.set_tag((int)(req->arg0), (int)(req->arg1));
    break;
  case tmr_msgs::srv::SetEvent_Request::WAIT_TAG:
    rb = iface_.set_wait_tag((int)(req->arg0), (int)(req->arg1));
    break;
  case tmr_msgs::srv::SetEvent_Request::STOP:
    rb = iface_.set_stop();
    break;
  case tmr_msgs::srv::SetEvent_Request::PAUSE:
    rb = iface_.set_pause();
    break;
  case tmr_msgs::srv::SetEvent_Request::RESUME:
    rb = iface_.set_resume();
    break;
  }
  res->ok = rb;
  return rb;
}
bool TmSctRos2::set_io(
  const std::shared_ptr<tmr_msgs::srv::SetIO::Request> req,
  std::shared_ptr<tmr_msgs::srv::SetIO::Response> res)
{
  bool rb = iface_.set_io(tmrl::driver::IOModule(req->module), tmrl::driver::IOType(req->type), int(req->pin), req->state);
  res->ok = rb;
  return rb;
}
bool TmSctRos2::set_positions(
  const std::shared_ptr<tmr_msgs::srv::SetPositions::Request> req,
  std::shared_ptr<tmr_msgs::srv::SetPositions::Response> res)
{
  bool rb = false;
  const size_t dof = tmrl::driver::RobotState::DOF;

  if (req->positions.size() != dof) {
    return rb;
  }
  switch(req->motion_type) {
  case tmr_msgs::srv::SetPositions_Request::PTP_J:
    rb = iface_.set_joint_pos_PTP(tmrl::to_arrayd<dof>(req->positions),
      (int)(req->velocity), req->acc_time, req->blend_percentage, req->fine_goal);
    break;
  case tmr_msgs::srv::SetPositions_Request::PTP_T:
    rb = iface_.set_tool_pose_PTP(tmrl::to_arrayd<dof>(req->positions),
      (int)(req->velocity), req->acc_time, req->blend_percentage, req->fine_goal);
    break;
  case tmr_msgs::srv::SetPositions_Request::LINE_T:
    rb = iface_.set_tool_pose_Line(tmrl::to_arrayd<dof>(req->positions),
      req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
    break;
  }
  res->ok = rb;
  return rb;
}

bool TmSctRos2::ask_sta(
  const std::shared_ptr<tmr_msgs::srv::AskSta::Request> req,
  std::shared_ptr<tmr_msgs::srv::AskSta::Response> res)
{
  SctMsg &pm = pm_;
  bool rb = false;

  std::unique_lock<std::mutex> lck(sta_mtx_);
  sta_updated_ = false;
  lck.unlock();

  rb = iface_.tmsct.send_sta_request(req->subcmd, req->subdata);

  lck.lock();
  if (rb && req->wait_time > 0.0) {
    if (!sta_updated_) {
      sta_cv_.wait_for(lck, std::chrono::duration<double>(req->wait_time));
    }
    if (!sta_updated_) {
      rb = false;
    }
    res->subcmd = pm.sta_msg.subcmd;
    res->subdata = pm.sta_msg.subdata;
  }
  sta_updated_ = false;

  res->ok = rb;
  return rb;
}


rclcpp_action::GoalResponse TmSctRos2::handle_goal(const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  auto goal_id = rclcpp_action::to_string(uuid);
  RCLCPP_INFO_STREAM(this->get_logger(), "Received new action goal " << goal_id);

  if (has_goal_) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!is_fake_) {
    if (!iface_.tmsvr.client().is_connected()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!iface_.tmsct.client().is_connected()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (state_.has_error()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (!has_points(goal->trajectory)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  //for (auto &jn : goal->trajectory.joint_names) { tmr_DEBUG_STREAM(jn); }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse TmSctRos2::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  auto goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  RCLCPP_INFO_STREAM(this->get_logger(), "Got request to cancel goal " << goal_id);

  std::unique_lock<std::mutex> lck(as_mtx_);
  if (goal_id_.compare(goal_id) == 0 && has_goal_) {
    has_goal_ = false;
    iface_.stop_pvt_traj();
  }
  lck.unlock();

  return rclcpp_action::CancelResponse::ACCEPT;
}
void TmSctRos2::handle_accepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  std::unique_lock<std::mutex> lck(as_mtx_);
  goal_id_ = rclcpp_action::to_string(goal_handle->get_goal_id());
  has_goal_ = true;
  lck.unlock();

  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&TmSctRos2::execute_traj, this, std::placeholders::_1), goal_handle}.detach();
  //std::thread{std::bind(&TmSctRos2::execute_traj_feedback, this, std::placeholders::_1), goal_handle}.detach();
}

void TmSctRos2::execute_traj(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  tmrl_INFO_STREAM("TM_ROS: trajectory thread begin");

  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  //actually, no need to reorder
  //std::vector<trajectory_msgs::msg::JointTrajectoryPoint> traj_points;
  //reorder_traj_joints(traj_points, goal_handle->get_goal()->trajectory);
  auto &traj_points = goal_handle->get_goal()->trajectory.points;

  if (!is_positions_match(traj_points.front(), 0.01)) {
    result->error_code = result->PATH_TOLERANCE_VIOLATED;
    result->error_string = "Start point doesn't match current pose";
    RCLCPP_WARN_STREAM(this->get_logger(), result->error_string);

    //goal_handle->abort(result);
  }

  auto pvts = get_pvt_traj(traj_points, 0.025);
  //tmrl_INFO_STREAM("TM_ROS: traj. total time:=" << pvts->total_time);

  if (!goal_handle->is_executing()) {
    goal_handle->execute();
    tmrl_INFO_STREAM("goal_handle->execute()");
  }

  if (!is_fake_) {
    iface_.run_pvt_traj(*pvts);
  }
  else {
    iface_.fake_run_pvt_traj(*pvts);
  }
  if (rclcpp::ok()) {
    if (!is_positions_match(traj_points.back(), 0.01)) {
      result->error_code = result->GOAL_TOLERANCE_VIOLATED;
      result->error_string = "Current pose doesn't match Goal point";
      RCLCPP_WARN_STREAM(this->get_logger(), result->error_string);
    }
    else {
      result->error_code = result->SUCCESSFUL;
      result->error_string = "Goal reached, success!";
      RCLCPP_INFO_STREAM(this->get_logger(), result->error_string);
    }
    goal_handle->succeed(result);
  }

  std::unique_lock<std::mutex> lck(as_mtx_);
  goal_id_.clear();
  has_goal_ = false;
  lck.unlock();

  tmrl_INFO_STREAM("TM_ROS: trajectory thread end");
}
void TmSctRos2::execute_traj_feedback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  tmrl_INFO_STREAM("TM_ROS: feedback thread begin");

  auto goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
  feedback->joint_names = jns_;
  feedback->header.frame_id = "base";
  int time_count = 0;

  //std::this_thread::sleep_for(std::chrono::milliseconds(10));

  while (has_goal_) {
    bool same_goal = false;
    {
      std::lock_guard<std::mutex> lck(as_mtx_);
      if (goal_id.compare(goal_id_) == 0) {
        same_goal = true;
      }
    }
    if (!same_goal) {

      break;
    }

    // Feedback
    //if (time_count == 0) {
      tmrl::driver::RobotState::Ulock lck(state_.mtx);
      feedback->actual.positions = tmrl::to_vectorXd(state_.joint_angle());
      feedback->actual.velocities = tmrl::to_vectorXd(state_.joint_speed());
      lck.unlock();

      feedback->header.stamp = rclcpp::Node::now();
      feedback->desired.positions = feedback->actual.positions;
      goal_handle->publish_feedback(feedback);
    //}

    // Handle Excution
    //if (time_count % 100 == 0) {

    //}

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    time_count = (time_count + 1) % 1000;
  }
  tmrl_INFO_STREAM("TM_ROS: feedback thread end");
}

void TmSctRos2::reorder_traj_joints(
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &new_traj_points,
  const trajectory_msgs::msg::JointTrajectory& traj)
{
  /* Reorders trajectory - destructive */
  std::vector<size_t> mapping;
  mapping.resize(jns_.size(), jns_.size());
  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    for (size_t j = 0; j < jns_.size(); ++j) {
      if (traj.joint_names[i] == jns_[j])
        mapping[j] = i;
    }
  }
  new_traj_points.clear();
  for (unsigned int i = 0; i < traj.points.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint new_point;
    for (unsigned int j = 0; j < traj.points[i].positions.size(); ++j) {
      new_point.positions.push_back(traj.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(traj.points[i].velocities[mapping[j]]);
      if (traj.points[i].accelerations.size() != 0)
        new_point.accelerations.push_back(traj.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = traj.points[i].time_from_start;
    new_traj_points.push_back(new_point);
  }
}
bool TmSctRos2::has_points(const trajectory_msgs::msg::JointTrajectory &traj)
{
  if (traj.points.size() == 0) return false;
  for (auto &point : traj.points) {
    if (point.positions.size() != traj.joint_names.size() ||
      point.velocities.size() != traj.joint_names.size()) return false;
  }
  return true;
}
bool TmSctRos2::is_traj_finite(const trajectory_msgs::msg::JointTrajectory &traj)
{
  for (auto &point : traj.points) {
    for (auto &p : point.positions) {
      if (!std::isfinite(p)) return false;
    }
    for (auto &v : point.velocities) {
      if (!std::isfinite(v)) return false;
    }
  }
  return true;
}
bool TmSctRos2::is_positions_match(
  const trajectory_msgs::msg::JointTrajectoryPoint &point, double eps)
{
  tmrl::driver::RobotState::Ulock lck(state_.mtx);
  auto q_act = state_.joint_angle();
  lck.unlock();

  if (point.positions.size() != q_act.size()) {
    return false;
  }
  for (size_t i = 0; i < q_act.size(); ++i) {
    if (fabs(point.positions[i] - q_act[i]) > eps)
      return false;
  }
  return true;
}
void TmSctRos2::set_pvt_traj(
  tmrl::driver::PvtTraj &pvts, const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin)
{
  size_t i = 0, i_1 = 0, i_2 = 0;
  int skip_count = 0;
  tmrl::driver::PvtPoint point;

  if (traj_points.size() == 0) return;

  pvts.mode = tmrl::driver::PvtMode::Joint;

  auto sec = [](const builtin_interfaces::msg::Duration& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
  };

  // first point
  if (sec(traj_points[i].time_from_start) != 0.0) {
    tmrl_WARN_STREAM("TM_ROS: Traj.: first point should be the current position, with time_from_start set to 0.0");
    point.time = sec(traj_points[i].time_from_start);
    point.positions = traj_points[i].positions;
    point.velocities = traj_points[i].velocities;
    pvts.points.push_back(point);
  }
  for (i = 1; i < traj_points.size() - 1; ++i) {
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    if (point.time >= Tmin) {
      i_2 = i_1;
      i_1 = i;
      point.positions = traj_points[i].positions;
      point.velocities = traj_points[i].velocities;
      pvts.points.push_back(point);
    }
    else {
      ++skip_count;
    }
  }
  if (skip_count > 0) {
    tmrl_WARN_STREAM("TM_ROS: Traj.: skip " << skip_count << " points");
  }
  // last point
  if (traj_points.size() > 1) {
    i =  traj_points.size() - 1;
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    point.positions = traj_points[i].positions;
    point.velocities = traj_points[i].velocities;
    if (point.time >= Tmin) {
      pvts.points.push_back(point);
    }
    else {
      point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_2].time_from_start);
      pvts.points.back() = point;
      ++skip_count;
      tmrl_WARN_STREAM("TM_ROS: Traj.: skip 1 more last point");
    }
  }
  pvts.total_time = sec(traj_points.back().time_from_start);
}
std::shared_ptr<tmrl::driver::PvtTraj> TmSctRos2::get_pvt_traj(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin)
{
  std::shared_ptr<tmrl::driver::PvtTraj> pvts = std::make_shared<tmrl::driver::PvtTraj>();
  set_pvt_traj(*pvts, traj_points, Tmin);
  return pvts;
}
