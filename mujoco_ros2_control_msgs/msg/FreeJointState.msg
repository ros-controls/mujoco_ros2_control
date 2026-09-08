# A single entry in a SetFreeJointState request: the desired pose and velocity for one
# MuJoCo free-joint object, identified by the name of the body it drives.
#
# `pose.header.frame_id` and `twist.header.frame_id` are resolved independently: empty means
# the world frame, otherwise the name of another MuJoCo body to interpret relative to (its
# current world pose for `pose`, its current world orientation for `twist`). `header.stamp` is
# not used.
#
# Unset orientation defaults to identity and unset twist defaults to zero, so a plain reset
# only needs `name` and `pose.pose`.

# Name of the MuJoCo body driven by the target free joint.
string name

# Desired pose to write into the object's qpos.
geometry_msgs/PoseStamped pose

# Desired velocity to write into the object's qvel. The reference body's own velocity is not
# added when `twist.header.frame_id` is set.
geometry_msgs/TwistStamped twist
