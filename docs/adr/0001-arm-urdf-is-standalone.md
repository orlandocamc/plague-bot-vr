# Arm URDF is_standalone arg for unified robot composition

`plaguebot.urdf.xacro` originally rooted the PROTON Arm to a `<link name="world"/>` via a fixed `virtual_joint`, making it a self-contained standalone model. To mount the arm on the Mobile Base's Top Plate without duplicating all six arm links in a new file, we added a single `is_standalone` xacro arg (default: `true`) that conditionally wraps the world link and virtual joint. When `plaguebot_robot.urdf.xacro` includes the arm with `is_standalone:=false`, it omits the world root and instead connects `base_link` to `top_plate_link` via `arm_mount_joint`. No mesh collision geometry was changed.

**Considered alternatives**: Duplicate all arm links and joints in `plaguebot_robot` without touching `plaguebot.urdf.xacro`. Rejected because it creates a maintenance fork — any geometry change to the arm would need to be applied in two places.

**Constraint**: Mesh collision geometry in `plaguebot.urdf.xacro` must never be modified — it is calibrated to the physical robot for digital twin fidelity.
