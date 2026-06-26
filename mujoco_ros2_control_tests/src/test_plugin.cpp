#include <pluginlib/class_list_macros.hpp>
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace external_plugin_test
{

class TestPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr, const mjModel*, mjData*) override
  {
    return true;
  }
  void update(const mjModel*, mjData*) override
  {
  }
  void cleanup() override
  {
  }
};

}  // namespace external_plugin_test

PLUGINLIB_EXPORT_CLASS(external_plugin_test::TestPlugin, mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
