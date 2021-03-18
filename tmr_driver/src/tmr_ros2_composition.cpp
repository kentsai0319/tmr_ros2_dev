#include "tmr_driver/tmr_ros2_tmsvr.h"
#include "tmr_driver/tmr_ros2_tmsct.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  bool is_fake = true;
  std::string host;
  if (argc > 1) {
    host = argv[1];
    if (host.find("robot_ip_address:=") != std::string::npos) {
      host.replace(host.begin(), host.begin() + 18, "");
      is_fake = false;
    }
    else if (host.find("robot_ip:=") != std::string::npos) {
      host.replace(host.begin(), host.begin() + 10, "");
      is_fake = false;
    }
    else if (host.find("ip:=") != std::string::npos) {
      host.replace(host.begin(), host.begin() + 4, "");
      is_fake = false;
    }
  }
  if (is_fake) {
    std::cout << "No robot_ip, is_fake: true\n";
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  tmrl::driver::TmsvrClient tmsvr(host);
  tmrl::driver::TmsctClient tmsct(host);
  tmrl::driver::Driver iface(tmsvr, tmsct);

  auto tm_svr = std::make_shared<TmSvrRos2>(options, iface, is_fake);
  exec.add_node(tm_svr);
  auto tm_sct = std::make_shared<TmSctRos2>(options, iface, is_fake);
  exec.add_node(tm_sct);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}