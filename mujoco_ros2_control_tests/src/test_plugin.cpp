#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace external_plugin_test
{

class TestPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel*, mjData*) override
  {
    // Latched so a test subscribing after a reset still observes the count.
    world_reset_count_pub_ =
        node->create_publisher<std_msgs::msg::UInt32>("world_reset_count", rclcpp::QoS(1).transient_local());
    return true;
  }
  void update(const mjModel*, mjData*) override
  {
  }
  void on_reset(mjData*) override
  {
    std_msgs::msg::UInt32 msg;
    msg.data = ++world_reset_count_;
    world_reset_count_pub_->publish(msg);
  }
  void cleanup() override
  {
    world_reset_count_pub_.reset();
  }

private:
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr world_reset_count_pub_;
  uint32_t world_reset_count_{ 0 };
};

}  // namespace external_plugin_test

PLUGINLIB_EXPORT_CLASS(external_plugin_test::TestPlugin, mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
