#include "tmr_driver/tmr_ros2_tmsvr.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


TmSvrRos2::TmSvrRos2(const rclcpp::NodeOptions &options, tmrl::driver::Driver &iface, bool is_fake, bool stick_play)
  : Node("tmr_svr", options)
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

  pm_.joint_msg.name = jns_;

  pm_.fbs_pub = create_publisher<tmr_msgs::msg::FeedbackState>("feedback_states", 1);
  pm_.joint_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  pm_.tool_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("tool_pose", 1);

  if (!is_fake_) {
    pm_.svr_pub = create_publisher<tmr_msgs::msg::TmsvrResponse>("tmsvr_response", 1);
  }
  if (!is_fake_) {
    auto ros_ok = [](){ return rclcpp::ok(); };

    iface_.tmsvr.set_is_ok_predicate(ros_ok);
    iface_.tmsvr.set_feedback_callback(std::bind(&TmSvrRos2::feedback_cb, this, std::placeholders::_1));
    iface_.tmsvr.set_response_callback(std::bind(&TmSvrRos2::tmsvr_cb, this, std::placeholders::_1));
    iface_.tmsvr.set_read_callback    (std::bind(&TmSvrRos2::tmsvr_cb, this, std::placeholders::_1));
    bool rb = iface_.tmsvr.start();
    if (rb && stick_play) {
      iface_.send_stick_play();
    }
  }
  else {
    tmrl::vector6d zeros{0};
    state_.set_joint_states(zeros, zeros, zeros);
    fake_fb_thd_ = std::thread(std::bind(&TmSvrRos2::fake_feedback, this));
  }

  if (!is_fake_) {
    connect_tmr_srv_ = create_service<tmr_msgs::srv::ConnectTMR>(
      "tmr/connect_tmsvr", std::bind(&TmSvrRos2::connect_tmsvr, this,
      std::placeholders::_1, std::placeholders::_2));

    write_item_srv_ = create_service<tmr_msgs::srv::WriteItem>(
      "tmr/write_item", std::bind(&TmSvrRos2::write_item, this,
      std::placeholders::_1, std::placeholders::_2));
    ask_item_srv_ = create_service<tmr_msgs::srv::AskItem>(
      "tmr/ask_item", std::bind(&TmSvrRos2::ask_item, this,
      std::placeholders::_1, std::placeholders::_2));
  }
}
TmSvrRos2::~TmSvrRos2()
{
  std::cout << "TM_ROS: TMSVR halt\n";

  if (fake_fb_thd_.joinable()) {
    fake_fb_thd_.join();
  }

  if (is_fake_) return;

  svr_updated_ = true;
  svr_cv_.notify_all();

  if (iface_.tmsvr.client().is_connected()) {
    //
  }
  iface_.tmsvr.stop();
}

void TmSvrRos2::fake_feedback()
{
  SvrMsg &pm = pm_;
  tmrl::driver::RobotState &rs = state_;

  tmrl_INFO_STREAM("TM_ROS: fake publisher thread begin");

  while (rclcpp::ok()) {
    tmrl::driver::RobotState::Ulock lck(rs.mtx);
    pm.fbs_msg.joint_pos = tmrl::to_vectorXd(rs.joint_angle());
    pm.fbs_msg.joint_vel = tmrl::to_vectorXd(rs.joint_speed());
    pm.fbs_msg.joint_tor = tmrl::to_vectorXd(rs.joint_torque());
    lck.unlock();

    pm.fbs_msg.header.stamp = rclcpp::Node::now();
    pm.fbs_pub->publish(pm.fbs_msg);

    // Publish joint state
    pm.joint_msg.position = pm.fbs_msg.joint_pos;
    pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
    pm.joint_msg.effort = pm.fbs_msg.joint_tor;
    pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
    pm.joint_pub->publish(pm.joint_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
  std::cout << "TM_ROS: fake publisher thread end\n";
}

void TmSvrRos2::feedback_cb(const tmrl::driver::RobotState &rs)
{
  SvrMsg &pm = pm_;

  // Publish feedback state

  pm.fbs_msg.is_svr_connected = iface_.tmsvr.client().is_connected();
  pm.fbs_msg.is_sct_connected = iface_.tmsct.client().is_connected();

  pm.fbs_msg.joint_pos = tmrl::to_vectorXd(rs.joint_angle());
  pm.fbs_msg.joint_vel = tmrl::to_vectorXd(rs.joint_speed());
  pm.fbs_msg.joint_tor = tmrl::to_vectorXd(rs.joint_torque());
  //pm.fbs_msg.flange_pose = tmrl::to_vectorXd(rs.flange_pose()); 
  pm.fbs_msg.tool_pose = tmrl::to_vectorXd(rs.tool_pose());
  pm.fbs_msg.tcp_speed = tmrl::to_vectorXd(rs.tcp_speed_vec());
  pm.fbs_msg.tcp_force = tmrl::to_vectorXd(rs.tcp_force_vec());
  pm.fbs_msg.robot_link = rs.is_linked();
  pm.fbs_msg.robot_error = rs.has_error();
  pm.fbs_msg.project_run = rs.is_project_running();
  pm.fbs_msg.project_pause = rs.is_project_paused();
  pm.fbs_msg.safetyguard_a = rs.is_safeguard_A();
  pm.fbs_msg.e_stop = rs.is_EStop();
  pm.fbs_msg.camera_light = rs.camera_light();
  pm.fbs_msg.error_code = rs.error_code();
  pm.fbs_msg.project_speed = rs.project_speed();
  pm.fbs_msg.ma_mode = rs.ma_mode();
  pm.fbs_msg.robot_light = rs.robot_light();
  pm.fbs_msg.cb_digital_output = tmrl::to_vectorX(rs.ctrller_DO());
  pm.fbs_msg.cb_digital_input = tmrl::to_vectorX(rs.ctrller_DI());
  pm.fbs_msg.cb_analog_output = tmrl::to_vectorX(rs.ctrller_AO());
  pm.fbs_msg.cb_analog_input = tmrl::to_vectorX(rs.ctrller_AI());
  pm.fbs_msg.ee_digital_output = tmrl::to_vectorX(rs.ee_DO());
  pm.fbs_msg.ee_digital_input = tmrl::to_vectorX(rs.ee_DI());
  //pm.fbs_msg.ee_analog_output = tmrl::to_vectorX(rs.ee_AO());
  pm.fbs_msg.ee_analog_input = tmrl::to_vectorX(rs.ee_AI());
  pm.fbs_msg.error_content = rs.error_content();

  pm.fbs_msg.header.stamp = rclcpp::Node::now();
  pm.fbs_pub->publish(pm.fbs_msg);

  // Publish joint state
  pm.joint_msg.position = pm.fbs_msg.joint_pos;
  pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
  pm.joint_msg.effort = pm.fbs_msg.joint_tor;
  pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
  pm.joint_pub->publish(pm.joint_msg);

  // Publish tool pose
  auto &pose = pm.fbs_msg.tool_pose;
  tf2::Quaternion quat;
  quat.setRPY(pose[3], pose[4], pose[5]);
  tf2::Transform Tbt{ quat, tf2::Vector3(pose[0], pose[1], pose[2]) };
  pm.tool_pose_msg.header.stamp = pm.joint_msg.header.stamp;
  pm.tool_pose_pub->publish(pm.tool_pose_msg);

}
void TmSvrRos2::tmsvr_cb(const tmrl::comm::TmsvrPacket &pack)
{
  SvrMsg &pm = pm_;

  std::unique_lock<std::mutex> lck(svr_mtx_);
  pm.svr_msg.id = pack.transaction_id();
  pm.svr_msg.mode = (int)(pack.mode());
  pm.svr_msg.content = pack.content();
  pm.svr_msg.error_code = (int)(pack.errcode());

  svr_updated_ = true;
  lck.unlock();
  svr_cv_.notify_all();

  tmrl_INFO_STREAM("$TMSVR: " << pm.svr_msg.id
    << ", " << (int)(pm.svr_msg.mode) << ", " << pm.svr_msg.content);

  pm.svr_msg.header.stamp = rclcpp::Node::now();
  pm.svr_pub->publish(pm.svr_msg);
}

bool TmSvrRos2::connect_tmsvr(
  const std::shared_ptr<tmr_msgs::srv::ConnectTMR::Request> req,
  std::shared_ptr<tmr_msgs::srv::ConnectTMR::Response> res)
{
  bool rb = true;
  int t_o = (int)(1000.0 * req->timeout);
  int t_v = (int)(1000.0 * req->timeval);

  tmrl::comm::ClientThread *client = &iface_.tmsvr;

  if (req->connect) {
    tmrl_INFO_STREAM("TM_ROS: (re)connect(" << t_o << ") TMSVR");
    client->stop();
    rb = client->start(t_o);
  }
  if (req->reconnect) {
    client->set_reconnect_timeout(req->timeout);
    client->set_reconnect_timeval(req->timeval);
    tmrl_INFO_STREAM("TM_ROS: set TMSVR reconnect timeout " << t_o << "ms, timeval " << t_v << "ms");
  }
  else {
    // no reconnect
    client->set_reconnect_timeval(-1);
    tmrl_INFO_STREAM("TM_ROS: set TMSVR NOT reconnect");
  }
  res->ok = rb;
  return rb;
}

bool TmSvrRos2::write_item(
  const std::shared_ptr<tmr_msgs::srv::WriteItem::Request> req,
  std::shared_ptr<tmr_msgs::srv::WriteItem::Response> res)
{
  bool rb;
  std::string content = req->item + "=" + req->value;
  rb = iface_.tmsvr.send_content(req->id, content);
  res->ok = rb;
  return rb;
}
bool TmSvrRos2::ask_item(
  const std::shared_ptr<tmr_msgs::srv::AskItem::Request> req,
  std::shared_ptr<tmr_msgs::srv::AskItem::Response> res)
{
  SvrMsg &pm = pm_;
  bool rb = false;

  std::unique_lock<std::mutex> lck(svr_mtx_);
  svr_updated_ = false;
  lck.unlock();

  rb = iface_.tmsvr.send_content(req->id, req->item, tmrl::comm::TmsvrPacket::Mode::READ_STRING);

  lck.lock();
  if (rb && req->wait_time > 0.0) {
    if (!svr_updated_) {
      svr_cv_.wait_for(lck, std::chrono::duration<double>(req->wait_time));
    }
    if (!svr_updated_) {
      rb = false;
    }
    res->id = pm.svr_msg.id;
    res->value = pm.svr_msg.content;
  }
  svr_updated_ = false;

  res->ok = rb;
  return rb;
}
