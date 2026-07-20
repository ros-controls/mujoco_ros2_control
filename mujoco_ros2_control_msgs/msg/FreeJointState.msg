# A single entry in a SetFreeJointState request: the desired pose and velocity for one
# MuJoCo free-joint object, identified by the name of the body it drives.
#
# An unset orientation defaults to identity (w=1, per the Quaternion.msg default)
# and an unset twist defaults to all-zero, so the common "reset to a pose, at
# rest" case only needs to fill in `name` and `pose`.

# Name of the MuJoCo body driven by the target free joint.
string name

# Desired pose to write into the object's qpos, interpreted relative to `reference_frame`
# (or the world frame if `reference_frame` is empty).
geometry_msgs/Pose pose

# Desired velocity to write into the object's qvel, interpreted relative to `reference_frame`
# (or the world frame if `reference_frame` is empty). Leave at the default (all-zero) to bring
# the object to rest at the new pose.
geometry_msgs/Twist twist

# Optional reference frame. If empty (default), `pose` and `twist` are interpreted in the
# world frame. If set to the name of another MuJoCo body:
#   - `pose` is interpreted relative to that body's current world pose
#     (i.e. world_pose = ref_world_pose * pose).
#   - `twist` is rotated into the world frame by that body's current world orientation
#     (the reference body's own velocity is not added).
string reference_frame
