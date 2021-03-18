#include "tmrl/driver/driver.h"
#include "tmrl/utils/logger.h"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "tmr_msgs/msg/feedback_state.hpp"
#include "tmr_msgs/msg/tmsvr_response.hpp"

#include "tmr_msgs/srv/connect_tmr.hpp"
#include "tmr_msgs/srv/write_item.hpp"
#include "tmr_msgs/srv/ask_item.hpp"


class TmSvrRos2 : public rclcpp::Node
{
public:
  explicit TmSvrRos2(const rclcpp::NodeOptions &options, tmrl::driver::Driver &iface, bool is_fake, bool stick_play = false);
  ~TmSvrRos2();

  bool connect_tmsvr(
    const std::shared_ptr<tmr_msgs::srv::ConnectTMR::Request> req,
    std::shared_ptr<tmr_msgs::srv::ConnectTMR::Response> res);

  bool write_item(
    const std::shared_ptr<tmr_msgs::srv::WriteItem::Request> req,
    std::shared_ptr<tmr_msgs::srv::WriteItem::Response> res);
  bool ask_item(
    const std::shared_ptr<tmr_msgs::srv::AskItem::Request> req,
    std::shared_ptr<tmr_msgs::srv::AskItem::Response> res);

private:
  void fake_feedback();

  void feedback_cb(const tmrl::driver::RobotState &rs);
  void tmsvr_cb(const tmrl::comm::TmsvrPacket &pack);

protected:
  tmrl::driver::Driver &iface_;
  tmrl::driver::RobotState &state_;

  const bool is_fake_;

  std::vector<std::string> jns_;

  std::thread fake_fb_thd_;

  struct SvrMsg {
    rclcpp::Publisher<tmr_msgs::msg::FeedbackState>::SharedPtr fbs_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_pub;
    rclcpp::Publisher<tmr_msgs::msg::TmsvrResponse>::SharedPtr svr_pub;

    tmr_msgs::msg::FeedbackState fbs_msg;
    sensor_msgs::msg::JointState joint_msg;
    geometry_msgs::msg::PoseStamped tool_pose_msg;
    tmr_msgs::msg::TmsvrResponse svr_msg;
  } pm_;

  std::mutex svr_mtx_;
  std::condition_variable svr_cv_;
  bool svr_updated_ {false};

  rclcpp::Service<tmr_msgs::srv::ConnectTMR>::SharedPtr connect_tmr_srv_;

  rclcpp::Service<tmr_msgs::srv::WriteItem>::SharedPtr write_item_srv_;
  rclcpp::Service<tmr_msgs::srv::AskItem>::SharedPtr ask_item_srv_;
};
