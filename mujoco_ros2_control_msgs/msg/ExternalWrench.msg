# Apply an external wrench to a named body/link in the MuJoCo simulation
# for a given duration.

# Standard ROS header (stamp, frame_id)
std_msgs/Header             header

# Wrench expressed in the world frame.
#   frame_id should be the name of the link/body the wrench is applied to (must match the MJCF body name).
#   force  [N]   – linear force  (x, y, z)
#   torque [N·m] – angular moment (x, y, z)
geometry_msgs/WrenchStamped        wrench

# Point of force application expressed in the body's local frame
# (relative to the body frame origin, in metres).
# A zero vector applies the force at the body frame origin.
geometry_msgs/Point         application_point

# How long the wrench remains active.
# A zero duration applies the wrench for a single simulation step.
builtin_interfaces/Duration duration

# Optional ramp-down window at the end of the duration over which the wrench
# magnitude is linearly scaled from 1.0 to 0.0.  Must be <= duration.
# A zero value (default) produces the original abrupt cut-off behaviour.
builtin_interfaces/Duration ramp_down_duration
