# Wheel joint naming inversion in diff_drive_controller

The Mobile Base URDF names the wheels `front_left_wheel_joint` and `front_right_wheel_joint`, but due to the physical chassis design, the joint the URDF calls "front_right" is mounted on the physical left side of the robot (and vice versa). Rather than rename the URDF joints — which would break the real-robot hardware interface and any saved configurations — `base_controllers.yaml` intentionally places the URDF-named "right" joints in `left_wheel_names` and the URDF-named "left" joints in `right_wheel_names`.

This means the controller YAML looks like a bug but is not. The comment in `base_controllers.yaml` documents this inversion explicitly.

**In simulation**: all four wheels are driven by the diff_drive_controller (both front and rear pairs appear in `left_wheel_names`/`right_wheel_names`). **On the real robot**: only the front pair receives command interfaces from `plaguebot_firmware`; rear wheels are passive and follow mechanically.
