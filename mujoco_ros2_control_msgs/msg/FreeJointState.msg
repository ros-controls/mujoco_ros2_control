# A single entry in a SetFreeJointState request: the desired pose and velocity for one
# MuJoCo free-joint object, identified by the name of the body it drives.
#
# `pose` and `twist` each carry their own reference frame via their Header's `frame_id`,
# resolved independently: an empty `frame_id` means the world frame; a non-empty `frame_id`
# names another MuJoCo body, and the pose/twist is interpreted relative to that body's current
# world pose/orientation. This lets the two differ -- e.g. a pose relative to a gripper link
# but a twist in the world frame. `header.stamp` is not used.
#
# An unset orientation defaults to identity (w=1, per the Quaternion.msg default) and an unset
# twist defaults to all-zero, so the common "reset to a pose, at rest" case only needs `name`
# and `pose.pose`.

# Name of the MuJoCo body driven by the target free joint.
string name

# Desired pose to write into the object's qpos. `pose.header.frame_id` selects the reference
# frame: empty for world, or another MuJoCo body's name to compose onto that body's current
# world pose (i.e. world_pose = ref_world_pose * pose.pose).
geometry_msgs/PoseStamped pose

# Desired velocity to write into the object's qvel. `twist.header.frame_id` selects the
# reference frame: empty for world, or another MuJoCo body's name to rotate the velocity into
# the world frame using that body's current world orientation (the reference body's own
# velocity is not added). Leave at the default (all-zero) to bring the object to rest at the
# new pose.
geometry_msgs/TwistStamped twist
