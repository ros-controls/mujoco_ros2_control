# A sparse set of MuJoCo joint states: only the joints and bodies listed here are specified.
# This is not a complete simulation state -- it carries no `ctrl`, `act`, `time`, or sensor data.

# Single-DOF (hinge/slide) joint states, keyed by MuJoCo joint name and given as parallel
# `name`/`position`/`velocity` arrays. `position` and `velocity` must each be empty or the same
# length as `name`, so a given field is set for every listed joint or for none of them.
# `effort` is not supported and must be empty; `header` is unused.
# Ball and other multi-DOF joints are not supported: one scalar per name cannot express them.
sensor_msgs/JointState joint_states

# Free joint states, keyed by the name of the body each free joint drives (not by joint name).
# See FreeJointState.msg for per-entry fields.
FreeJointState[] free_joints
