# Array of external wrenches to apply simultaneously to one or more bodies
# in the MuJoCo simulation.  Publish this message to apply multiple wrenches
# in a single update cycle; each entry is processed independently.

# List of wrenches to apply; see ExternalWrench.msg for per-entry fields.
ExternalWrench[] external_wrenches
