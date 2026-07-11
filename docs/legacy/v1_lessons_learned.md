# Lessons from spark_isaac_mycobot_demo (v1)

This fork replaces v1's **direct PPO→joint** learned-IK architecture with
**classical IK + bounded residual**. Carry forward operational lessons only.

## What worked in v1

- Host ↔ container delegation via `nsenter` (`scripts/host/spark_host_exec.sh`)
- Isaac Lab install/verify scripts on DGX Spark
- Explicit units and joint-name conventions from `mycobot_ros2`
- Verified coarse reach policy @ **25 mm** (not claimed 1 mm on hardware)

## What to avoid repeating

- Competing training recipes / conflicting docs
- Claiming sub-mm accuracy without hardware verification
- Allowing learned commands to bypass validation
- Running Isaac Sim / Isaac Lab inside the Isaac ROS container
- Unattended multi-stage Isaac reloads without a watchdog

## Architecture contrast

| v1 (demo) | v2 (this fork) |
|-----------|----------------|
| Policy *is* the IK | Classical IK is base; learning adds `Δq` |
| PPO joint deltas | Supervised residual → SAC residual |
| Prohibited classical IK | **Required** classical IK |
