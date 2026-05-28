#pragma once

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::demo
{

/**
 * A minimal do-nothing MuJoCo sensor plugin.
 *
 * This serves as a starting point for developing new MuJoCo extensions.
 * The plugin registers as a sensor, outputs a single zero value, and
 * does no meaningful computation.
 */
class DemoPlugin
{
public:
  static DemoPlugin* Create(const mjModel* m, mjData* d, int instance);

  DemoPlugin(const mjModel* m, mjData* d, int instance);

  void Reset(const mjModel* m, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance);

  static void RegisterPlugin();

private:
  int sensor_id_;
  int sensor_address_;
};

}  // namespace mujoco::plugin::demo
