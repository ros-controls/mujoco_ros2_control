# Stamped array of MuJoCo constraint violations.
# Each entry corresponds to one active equality constraint.
# The violation is the maximum absolute position-level residual (efc_pos)
# across all constraint rows belonging to that constraint.

builtin_interfaces/Time stamp

# Name of each constraint.
# If the MJCF <equality> element has a name attribute, that name is used.
# Otherwise the name is synthesised from the connected objects:
#   joint constraints  -> "<joint1>_<joint2>_constraint"
#   tendon constraints -> "<tendon1>_<tendon2>_constraint"
#   weld constraints   -> "<body1>_<body2>_weld_constraint"
#   connect constraints-> "<body1>_<body2>_connect_constraint"
string[] names

# Constraint type string: "joint", "tendon", "weld", "connect", or "unknown"
string[] types

# Maximum absolute position-level residual (efc_pos) for each constraint.
# Values close to zero mean the constraint is well satisfied.
float64[] violations
