#include "mujoco_demo_extension/demo_plugin.hpp"

namespace mujoco::plugin::demo
{

struct StaticRegister
{
  StaticRegister()
  {
    DemoPlugin::RegisterPlugin();
  }
};

static StaticRegister _register;

}  // namespace mujoco::plugin::demo
