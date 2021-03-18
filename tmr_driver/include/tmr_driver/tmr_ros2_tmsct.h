#include "tmrl/driver/driver.h"
#include "tmrl/utils/logger.h"

#include <rclcpp/rclcpp.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "tmr_msgs/msg/tmsct_response.hpp"
#include "tmr_msgs/msg/tmsta_response.hpp"

#include "tmr_msgs/srv/connect_tmr.hpp"
#include "tmr_msgs/srv/write_item.hpp"
#include "tmr_msgs/srv/ask_item.hpp"
#include "tmr_msgs/srv/send_script.hpp"
#include "tmr_msgs/srv/set_event.hpp"
#include "tmr_msgs/srv/set_io.hpp"
//#include "tmr_msgs/srv/set_payload.hpp"
#include "tmr_msgs/srv/set_positions.hpp"
#include "tmr_msgs/srv/ask_sta.hpp"


class TmSctRos2 : public rclcpp::Node
{
public:
  explicit TmSctRos2(const rclcpp::NodeOptions &options, tmrl::driver::Driver &iface, bool is_fake);
  ~TmSctRos2();

  bool connect_tmsct(
    const std::shared_ptr<tmr_msgs::srv::ConnectTMR::Request> req,
    std::shared_ptr<tmr_msgs::srv::ConnectTMR::Response> res);

  bool send_script(
    const std::shared_ptr<tmr_msgs::srv::SendScript::Request> req,
    std::shared_ptr<tmr_msgs::srv::SendScript::Response> res);
  bool set_event(
    const std::shared_ptr<tmr_msgs::srv::SetEvent::Request> req,
    std::shared_ptr<tmr_msgs::srv::SetEvent::Response> res);
  bool set_io(
    const std::shared_ptr<tmr_msgs::srv::SetIO::Request> req,
    std::shared_ptr<tmr_msgs::srv::SetIO::Response> res);
  bool set_positions(
    const std::shared_ptr<tmr_msgs::srv::SetPositions::Request> req,
    std::shared_ptr<tmr_msgs::srv::SetPositions::Response> res);

  bool ask_sta(
    const std::shared_ptr<tmr_msgs::srv::AskSta::Request> req,
    std::shared_ptr<tmr_msgs::srv::AskSta::Response> res);

private:
  void tmsct_cb(const tmrl::comm::TmsctPacket &pack);
  void tmsta_cb(const tmrl::comm::TmstaPacket &pack);

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
  );
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  );
  void handle_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  );
  void execute_traj(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  );
  void execute_traj_feedback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
  );
  void reorder_traj_joints(
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &new_traj_points,
    const trajectory_msgs::msg::JointTrajectory &traj
  );
  bool has_points(const trajectory_msgs::msg::JointTrajectory &traj);
  bool is_traj_finite(const trajectory_msgs::msg::JointTrajectory &traj);
  bool is_positions_match(const trajectory_msgs::msg::JointTrajectoryPoint &point, double eps = 0.01);
  void set_pvt_traj(
    tmrl::driver::PvtTraj &pvts, const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin = 0.1
  );
  std::shared_ptr<tmrl::driver::PvtTraj> get_pvt_traj(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin = 0.1
  );

protected:
  tmrl::driver::Driver &iface_;
  tmrl::driver::RobotState &state_;

  const bool is_fake_;

  std::vector<std::string> jns_;

  struct SctMsg {
    rclcpp::Publisher<tmr_msgs::msg::TmsctResponse>::SharedPtr sct_pub;
    rclcpp::Publisher<tmr_msgs::msg::TmstaResponse>::SharedPtr sta_pub;

    tmr_msgs::msg::TmsctResponse sct_msg;
    tmr_msgs::msg::TmstaResponse sta_msg;
  } pm_;

  std::mutex sct_mtx_;
  std::condition_variable sct_cv_;
  bool sct_updated_ {false};

  std::mutex sta_mtx_;
  std::condition_variable sta_cv_;
  bool sta_updated_ {false};

  rclcpp::Service<tmr_msgs::srv::ConnectTMR>::SharedPtr connect_tmr_srv_;

  rclcpp::Service<tmr_msgs::srv::SendScript>::SharedPtr send_script_srv_;
  rclcpp::Service<tmr_msgs::srv::SetEvent>::SharedPtr set_event_srv_;
  rclcpp::Service<tmr_msgs::srv::SetIO>::SharedPtr set_io_srv_;
  rclcpp::Service<tmr_msgs::srv::SetPositions>::SharedPtr set_positions_srv_;

  rclcpp::Service<tmr_msgs::srv::AskSta>::SharedPtr ask_sta_srv_;

  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr as_;
  std::mutex as_mtx_;
  std::string goal_id_;
  bool has_goal_ {false};
};
