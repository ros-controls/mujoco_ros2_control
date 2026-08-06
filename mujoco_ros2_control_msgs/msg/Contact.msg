# Selected information from one mjContact entry in mjData.contact.
# The message preserves mjData.contact order; id is the corresponding array index.
# Position and contact-frame axes are expressed in MuJoCo world coordinates.

# Index in mjData.contact for this contact.
int32 id

# Contact descriptors set by mj_collideXXX.
# Geom ids are -1 for flex contacts; names are derived from geom ids.
int32[2] geom_ids
string[2] geom_names

# Owning body ids and names, derived from geom ids.
# They are -1 and empty when the participant is a flex contact.
int32[2] body_ids
string[2] body_names

# Flex ids; -1 for geom contacts.
int32[2] flex_ids

# Element ids; -1 for geom contacts or flex vertices.
int32[2] elem_ids

# Vertex ids; -1 for geom contacts or flex elements.
int32[2] vert_ids

# Distance between nearest points; negative values indicate penetration.
float64 distance

# Contact point position: midpoint between the two geoms.
geometry_msgs/Point position

# Contact frame. The normal points from participant 0 to participant 1.
# tangent_1 and tangent_2 are the remaining contact-frame axes.
geometry_msgs/Vector3 normal
geometry_msgs/Vector3 tangent_1
geometry_msgs/Vector3 tangent_2

# Contact space dimensionality: 1, 3, 4, or 6.
int32 dimension

# Exclusion state set by mj_setContact or mj_instantiateContact:
# 0 include, 1 in gap, 2 fused, 3 no dofs, 4 passive.
int32 exclude

# Contact parameters set by mj_collideGeoms.
# include_margin is the margin used for force generation.
float64 include_margin

# Friction coefficients in the order tangent1, tangent2, spin, roll1, roll2.
float64[5] friction

# Constraint solver reference parameters for the normal direction.
# Array length follows the linked MuJoCo build's mjNREF constant.
float64[] solref

# Constraint solver reference parameters for friction directions.
# Array length follows the linked MuJoCo build's mjNREF constant.
float64[] solref_friction

# Constraint solver impedance parameters.
# Array length follows the linked MuJoCo build's mjNIMP constant.
float64[] solimp
