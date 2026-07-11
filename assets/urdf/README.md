# URDF assets

## Kinematics-only (in git)

[`mycobot_280_m5_kinematics.urdf`](mycobot_280_m5_kinematics.urdf) — joint chain only
(`g_base` → `joint6_flange`). Used by Phase 1 FK unit tests and CI (no meshes).

## Full vendor package

```bash
./scripts/download_mycobot_ros2.sh
```

That clones or symlinks [elephantrobotics/mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2)
into `third_party/mycobot_ros2/` (preferred path:
`mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf`).

On this DGX Spark workspace, a sibling checkout at
`/workspaces/isaac_ros-dev/src/mycobot_ros2` is symlinked when present.
