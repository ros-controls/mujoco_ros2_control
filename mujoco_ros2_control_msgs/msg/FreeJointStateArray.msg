# Snapshot of the pose and velocity of one or more MuJoCo free-joint bodies, as published by
# the FreeJointStatePublisherPlugin.

# Time the snapshot was taken and, in `frame_id`, the reference frame every entry's
# `pose.header.frame_id` / `twist.header.frame_id` is expressed in: empty means the world frame,
# otherwise the name of the MuJoCo body the poses/twists are relative to.
std_msgs/Header header

# One entry per published free-joint body.
FreeJointState[] free_joints
