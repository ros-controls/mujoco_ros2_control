# Generate Moveable Object MJCFs

Moveable objects that are not modeled as part of the environment need to be incorporated into the MuJoCo scene as separate MJCFs.
This process is very similar to the process for [generating a robot MJCF](./TOOLS.md).

1.  If not already done in an environments/mockups package:
    1.  Create STL models for the moveable objects.  See [tips for modeling environment objects](./MODELING_ENVIRONMENTS.md) for additional helpful information.
    2.  Create URDFs for the moveable objects.  The moveable objects will not require a special URDF using the [MuJoCo `ros2_control` hardware interface plugin](../README.md#plugin), since the robot is expected to interact with and move these objects.  Instead, conversion can be performed directly on the environment/mockups URDFs or these URDFs can be xacro included into the top-level MuJoCo URDF (as is done with the [cargo back in the CLR demo](https://github.com/NASA-JSC-Robotics/chonkur_l_raile/blob/jazzy-devel/clr_mujoco_config/urdf/clr_mujoco_xacro.urdf#L19)).
2.  Create input(s) for the conversion process; each moveable object can be converted separately or as part of a top-level MuJoCo inputs file.  Most important is decomposing any object sub-parts the robot will need to interact with.  This decomposition ensures the conves hulls for collision checking on these interactable sub-parts are accurate.
    1.  It can often be useful to see what level of precision has been used to decompose interactable sub-parts in past applications, specifically the [`threshold` attribute](./TOOLS.md#processed-inputs-attribute-reference).
3.  Run the conversion script.
4.  Test the conversion process has been completed properly.
