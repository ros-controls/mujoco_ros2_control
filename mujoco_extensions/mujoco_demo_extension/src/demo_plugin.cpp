#include "mujoco_demo_extension/demo_plugin.hpp"

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include <cstdio>

namespace mujoco::plugin::demo
{

DemoPlugin* DemoPlugin::Create(const mjModel* m, mjData* d, int instance)
{
  return new DemoPlugin(m, d, instance);
}

DemoPlugin::DemoPlugin(const mjModel* m, mjData* d, int instance)
{
  // Find the sensor associated with this plugin instance
  for (int id = 0; id < m->nsensor; ++id)
  {
    if (m->sensor_type[id] == mjSENS_PLUGIN && m->sensor_plugin[id] == instance)
    {
      sensor_id_ = id;
      break;
    }
  }
  sensor_address_ = m->sensor_adr[sensor_id_];
}

void DemoPlugin::Reset(const mjModel* m, int instance)
{
  // Nothing to reset
}

void DemoPlugin::Compute(const mjModel* m, mjData* d, int instance)
{
  d->sensordata[sensor_address_] = 0.0;
}

void DemoPlugin::Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance)
{
}

void DemoPlugin::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.plugin.demo";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  // No config attributes
  plugin.nattribute = 0;
  plugin.attributes = nullptr;

  // No plugin state
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  // Single scalar output
  plugin.nsensordata = +[](const mjModel* m, int instance, int sensor_id) { return 1; };

  // Runs after forces are computed
  plugin.needstage = mjSTAGE_ACC;

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto* p = DemoPlugin::Create(m, d, instance);
    if (!p)
    {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(p);
    return 0;
  };

  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<DemoPlugin*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data, int instance) {
    auto* p = reinterpret_cast<DemoPlugin*>(plugin_data);
    if (p)
      p->Reset(m, instance);
  };

  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
    auto* p = reinterpret_cast<DemoPlugin*>(d->plugin_data[instance]);
    if (p)
      p->Compute(m, d, instance);
  };

  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
    auto* p = reinterpret_cast<DemoPlugin*>(d->plugin_data[instance]);
    if (p)
      p->Visualize(m, d, opt, scn, instance);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::demo
