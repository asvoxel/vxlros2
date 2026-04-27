#include <rclcpp/rclcpp.hpp>
#include "vxl_camera/vxl_camera_lifecycle_node.hpp"

// Standalone executable for the lifecycle node.
//
// By default it auto-transitions through CONFIGURE → ACTIVATE so users get the same
// "just works" experience as the non-lifecycle node. Pass `--no-auto-activate` to
// stay in UNCONFIGURED and let an external manager (e.g. nav2_lifecycle_manager)
// drive the transitions.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  bool auto_activate = true;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--no-auto-activate") {auto_activate = false;}
  }

  auto node = std::make_shared<vxl_camera::VxlCameraLifecycleNode>(rclcpp::NodeOptions());

  if (auto_activate) {
    auto cfg = node->configure();
    if (cfg.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "Auto-configure failed");
      rclcpp::shutdown();
      return 1;
    }
    auto act = node->activate();
    if (act.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "Auto-activate failed");
      rclcpp::shutdown();
      return 2;
    }
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
