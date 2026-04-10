# Generate Object MJCFs

Objects that are not modeled within the standard URDF process (for example, moveable objects not modeled as part of the environment) need to be incorporated into the MuJoCo scene.
They can be integrated as:

- Separate MJCFs, if not using on-the-fly MJCF conversion;
or
- Included into a top-level URDF for on-the-fly MJCF conversion.

Regardless of the approach for integrating these objects, the process is ultimately very similar to that for [generating a robot MJCF](./TOOLS.md).

1.  If not already done in a description package:
    1.  Create STL models for the objects.
    See [tips for modeling objects](./MODELING_TIPS.md) for additional helpful information.
    2.  Create URDFs for the objects.
    Some objects may not require a special URDF using the [MuJoCo `ros2_control` hardware interface plugin](../README.md#plugin), since the robot is expected to interact with and move these objects.
    Instead, conversion can be performed directly on the environment/mockups URDFs or these URDFs can be xacro included into the top-level MuJoCo URDF.
2.  Create input(s) for the conversion process; each object can be converted separately or as part of a top-level MuJoCo inputs file.
Most important is decomposing any object sub-parts the robot will need to interact with.
This decomposition ensures the conves hulls for collision checking on these interactable sub-parts are accurate.
    1.  It can often be useful to see what level of precision has been used to decompose interactable sub-parts in past applications, specifically the [`threshold` attribute](./TOOLS.md#processed-inputs-attribute-reference).
3.  Run the conversion script.
4.  Test the conversion process has been completed properly:
    ```bash
    ros2 run mujoco_vendor simulate <Converted Description File>
