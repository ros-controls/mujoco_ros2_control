# A sparse set of MuJoCo joint states: only the joints and bodies listed here are specified.
# This is not a complete simulation state -- it carries no `ctrl`, `act`, `time`, or sensor data.

# Single-DOF (hinge/slide) joint states, keyed by MuJoCo joint name and given as parallel
# `name`/`position`/`velocity`/`effort` arrays. `header` are not used by this message; MuJoCo
# ball and other multi-DOF joints are not supported here, since one scalar per name cannot
# express them. This message itself does not enforce field constraints (e.g. array lengths,
# which fields must be empty) -- those are defined by whichever service or topic consumes it;
# see ResetWorld.srv for the rules its `state_overrides` field enforces.
sensor_msgs/JointState joint_states

# Free joint states, keyed by the name of the body each free joint drives (not by joint name).
# See FreeJointState.msg for per-entry fields.
FreeJointState[] free_joints
