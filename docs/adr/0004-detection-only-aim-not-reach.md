# Robot is detection-only: the mission aims the camera, it does not reach the plant

The original plan had the robot carry a sprayer and use the arm to act on a
detected pest. **That is no longer the case: the plaguebot is detection-only.**
Its sole job is to inspect tomato plants for pests/diseases with the D435 depth
camera on the arm. The arm must never touch the plant or fruit.

## Consequence for the Mission's IK_POSITION step

SPEC §6.2 defined an `IK_POSITION` step that uses MoveIt `/compute_ik` to move the
gripper to the 3D detection point. Two problems:

1. **Geometry.** The PROTON arm reaches a ~0.5 m sphere around `base_link`, but the
   D435 is mounted on the wrist and looks *outward* at the plant row ~0.6–0.75 m
   away in the corridor. The only IK-reachable points sit ~at the wrist itself, so
   `/compute_ik` returns `NO_IK_SOLUTION (-31)` for every real detection.
2. **Scope.** With no sprayer, there is no reason to reach the plant at all.

So `IK_POSITION` is redefined as **`AIM`**: a camera *look-at*. The step rotates
the base yaw (`joint_1`) and wrist pitch (`joint_5`) by the detection's angular
offset in the camera (`d435_link`) frame, centering the pest in the D435 view for
a clean close-up. Aiming is always feasible — it removes the reach problem.

## Consequence for the stack

The mission no longer needs MoveIt: `AIM` is pure joint-space aiming, no
`/compute_ik`, no `move_group`. This also retires the standalone-vs-unified URDF
integration concern for the mission path. `plaguebot_moveit` stays in the repo for
offline arm planning / Setup Assistant work, but `mission.launch.py` no longer
brings up `move_group` (the `use_moveit` arg is removed).

This supersedes the `IK_POSITION` definition in SPEC §6.2.
