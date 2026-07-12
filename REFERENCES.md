# References

Curated reference material for **`spark_isaac_mycobot_v2`** (Residual Adaptive IK for MyCobot 280).
Originally collected for `spark_isaac_mycobot_demo`; retained for hardware, ROS 2, Isaac Sim/Lab,
and RL background. Prefer classical IK + residual learning references in [spec.md](spec.md);
ignore v1-specific paths such as `spark_verify_pkg` unless you are reading legacy notes.

When Phase 1 (or later phase) implementation libraries change, update this file **and** the
README “Phase N libraries” section in the same change set (see `spec.md` § Documentation Maintenance).

---

## Phase 1 implementation libraries

| Library | Docs / source | How Phase 1 uses it |
|---------|---------------|---------------------|
| [NumPy](https://numpy.org/doc/stable/) | Arrays, linalg | FK / DLS / stratified sampling / metrics |
| [PyYAML](https://pyyaml.org/) | Config load | `configs/robot/*.yaml`, `configs/ik/*.yaml` |
| [pytest](https://docs.pytest.org/en/stable/) | Fixtures, asserts | `tests/test_*.py` Phase 1 contracts (including even workspace coverage + 160 °/s motion cap) |
| [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html) | Kit / Articulation | Host metrics+viz (`isaac_sim/run_ik_viz.py`) |
| [pxr / USD](https://openusd.org/) | Stage / materials | Target sphere + lighting |
| [mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2) | Vendor URDF/meshes | Prepared via `isaac_sim/urdf_utils.py` for rendered Phase 1 |

## Phase 2 implementation libraries

| Library | Docs / source | How Phase 2 uses it |
|---------|---------------|---------------------|
| [NumPy](https://numpy.org/doc/stable/) | Arrays, linalg | Capsule–sphere collision; joint-path sampling (CI default) |
| [PyYAML](https://pyyaml.org/) | Config load | `configs/planning/collision.yaml` |
| [pytest](https://docs.pytest.org/en/stable/) | Fixtures, asserts | `tests/test_phase2_geometry.py` |
| [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html) | Kit viz logging | `PATH_OK` / `PATH_COLLISION` during host animation |
| [cuRobo](https://curobo.org/) | Apache-2.0 GPU planner | Host `MotionGen` collision-free trajectories (fail-closed) |
| [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html) | ROS 2 cuRobo-based motion | Future hardware / multi-waypoint recovery option |
| [MoveIt 2](https://moveit.picknik.ai/) | ROS 2 planning (OMPL, etc.) | Named states, approach/retreat, pipeline retries |
| [OMPL](https://ompl.kavrakilab.org/) | Sampling-based planning | Underpins many MoveIt planners |

Vendor reach / speed reference: [myCobot 280 specs](https://www.elephantrobotics.com/en/mycobot-280-m5-new-specificatons-en/) (280 mm, 160 °/s) → `configs/robot/workspace.yaml`.  
v1 even-coverage reference: `spark_isaac_mycobot_demo/isaac_lab/mdp_core.py` (cylindrical annulus + stratified demo bins).

---

## 1. MyCobot 280 hardware (Elephant Robotics)

| Reference | Why it matters here |
|---|---|
| [myCobot 280 Pi — product specifications](https://www.elephantrobotics.com/en/mycobot-280-pi-specifications-en/) | Authoritative spec sheet: 6 DOF, 250 g payload, **280 mm working radius**, ±0.5 mm repeatability, joint ranges ±165° (J6 ±175°), 160°/s max joint speed. These numbers drive `SafetyBoundaryConfig` (joint limits 2.879793 rad = 165°, workspace ±0.28 m) and `WorkspaceConstraints.workspace_span_m = 0.56` (2 × reach). |
| [myCobot 280 Pi — official GitBook documentation](https://docs.elephantrobotics.com/docs/mycobot_280_pi_en/) | Full user/developer manual: assembly, myStudio firmware flashing, Python/ROS development environment on the arm's built-in Raspberry Pi 4B. |
| [myCobot 280 M5 2023 — specifications](https://www.elephantrobotics.com/en/mycobot-280-m5-2023-specificatons-en/) | The M5 variant's spec page, listing explicit ROS 2 support; useful when choosing which 280 variant to buy for this project. |
| [elephantrobotics/mycobot_ros2 (GitHub)](https://github.com/elephantrobotics/mycobot_ros2) | Vendor ROS 2 packages with the URDF and meshes that Live Phase 1 ingests into Isaac Sim. The joint names in `articulation_model.cpp` / `articulation_model_py.py` (`joint1`…`joint6`) mirror this URDF. |
| [elephantrobotics/pymycobot (GitHub)](https://github.com/elephantrobotics/pymycobot) | The vendor Python serial library. Its framing (0xFE 0xFE header, command ids such as `send_angles`, angles as int16 tenths of a degree) is what `pymycobot_serial_encoder.cpp` and `edge_deployment_node._encode_serial_payload` emulate. |
| [pymycobot API documentation](https://docs.elephantrobotics.com/docs/gitbook-en/7-ApplicationBasePython/) | Command-level API reference (`send_angles`, `get_angles`, speed/limit semantics) for wiring the Phase 3 driver to the physical arm. |

## 2. ROS 2 Jazzy (core concepts used in `spark_verify_pkg`)

| Reference | Why it matters here |
|---|---|
| [ROS 2 Jazzy documentation (home)](https://docs.ros.org/en/jazzy/index.html) | Top-level docs for the exact distro this project targets (Ubuntu 24.04 / Jazzy Jalisco). |
| [ROS 2 concepts: nodes, topics, services, parameters](https://docs.ros.org/en/jazzy/Concepts/Basic.html) | The pub/sub and parameter model every node in `spark_verify_nodes/` and `src/` is built on. |
| [About Quality of Service (QoS) settings](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html) | Explains the depth-10 KEEP_LAST profiles used by every publisher/subscription, durability (why a one-shot publish can be lost — see `joint_command_dispatcher`), and reliability. |
| [About interfaces (.msg definition syntax)](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html) | Field types, fixed vs unbounded arrays — the rules behind the six custom messages in `msg/`. |
| [rclpy API documentation](https://docs.ros.org/en/jazzy/p/rclpy/) | Python client library API (Node, timers, spin) used by all Python nodes. |
| [rclcpp API documentation](https://docs.ros.org/en/jazzy/p/rclcpp/) | C++ client library API used by `mock_articulation_bridge` and `pymycobot_driver`. |
| [tf2 tutorials (Jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html) | The transform system: broadcasters (used by the articulation bridge) and Buffer/TransformListener lookups (used by the Phase 1 test). |
| [Launch system tutorials (Jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html) | Python launch files, `IncludeLaunchDescription` composition — the pattern layering `phase4 → phase3 → phase2 → phase1` launch files. |
| [ament_cmake user documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html) | The build-system extensions used throughout `CMakeLists.txt` (`ament_target_dependencies`, `ament_package`, export macros). |
| [colcon documentation](https://colcon.readthedocs.io/en/released/) | The workspace build/test tool: `colcon build`, `colcon test`, `colcon test-result --all`. |
| [ros2/launch_testing (GitHub)](https://github.com/ros2/launch/tree/rolling/launch_testing) | The framework behind all four `test_phase*_integration.py` suites: `generate_test_description()`, `ReadyToTest()`, active tests against live processes. |
| [ROS_DOMAIN_ID concept](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html) | Why each integration test pins a unique domain id (41–44): DDS discovery isolation between test suites. |
| [sensor_msgs/Image message reference](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | The `encoding` / `step` / `data` semantics the mock cameras and the vision tracker rely on. |
| [sensor_msgs/JointState message reference](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/JointState.html) | Parallel name/position arrays and the match-by-name rule implemented in `ArticulationModel::set_joint_positions`. |

## 3. NVIDIA Isaac Sim / Isaac Lab / Isaac ROS

| Reference | Why it matters here |
|---|---|
| [Isaac Sim documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html) | The simulator targeted by Live Phase 1 (URDF import, cameras, ROS 2 bridge). |
| [Isaac Sim ROS 2 tutorials](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/index.html) | How Isaac Sim exchanges JointState/TF/camera topics with ROS 2 — the live counterpart of `mock_articulation_bridge` and `mock_camera_publisher`. |
| [Isaac Sim robot simulation snippets (Articulation API)](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/robots_simulation.html) | The Articulation API named in `.cursorrules`: wrapping a robot prim and applying DOF position targets directly to the simulated robot rather than emulating controllers on the CPU. |
| [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/main/index.html) | The RL framework for Live Phase 2; its manager-based environments consume exactly the observation/reward decomposition mocked in `mycobot_mdp.py` and `reward_function.py`. |
| [Isaac Lab: creating a manager-based RL environment](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/03_envs/create_manager_rl_env.html) | Tutorial mapping observation terms / reward terms / action terms to code — the structure `MyCobotPickPlaceMDP` is shaped to slot into. |
| [Isaac ROS NITROS concepts](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html) | NVIDIA Isaac Transport for ROS: type adaptation and zero-copy transport. `NitrosFrameHandle.msg` mocks this descriptor pattern for Phase 1 verification. |
| [Isaac ROS documentation (home)](https://nvidia-isaac-ros.github.io/index.html) | GPU-accelerated perception packages; the production replacement for the threshold-based `block_vision_tracker`. |

## 4. Reinforcement learning background

| Reference | Why it matters here |
|---|---|
| [Sutton & Barto, *Reinforcement Learning: An Introduction* (2nd ed., free)](http://incompleteideas.net/book/the-book-2nd.html) | The standard text for the MDP formalism (`mycobot_mdp.py`) — states, actions, rewards, discounting. |
| [Ng, Harada & Russell (1999), *Policy invariance under reward transformations*](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/NgHaradaRussell-shaping-ICML1999.pdf) | The theory behind reward shaping — why the dense tracking/alignment/lift terms in `reward_function.py` guide learning without changing the optimal policy (when potential-based). |
| [Schulman et al. (2017), *Proximal Policy Optimization*](https://arxiv.org/abs/1707.06347) | PPO is the default algorithm in Isaac Lab's RL workflows and the likely trainer for Live Phase 2. |
| [OpenAI Spinning Up in Deep RL](https://spinningup.openai.com/en/latest/) | Approachable introduction to policy-gradient methods and RL practicalities. |

## 5. Model export and edge deployment (Phases 3–4)

| Reference | Why it matters here |
|---|---|
| [ONNX (Open Neural Network Exchange)](https://onnx.ai/) | The portable model format Phase 3 exports policies into; `mock_onnx_policy.py` fakes exactly this inference boundary. |
| [ONNX Runtime documentation](https://onnxruntime.ai/docs/) | Reference runtime for executing `.onnx` files; the one-line substitution point in `onnx_inference_node.py`. |
| [PyTorch → ONNX export guide](https://pytorch.org/docs/stable/onnx.html) | How trained Isaac Lab (PyTorch) policy weights become `.onnx` files. |
| [Raspberry Pi AI HAT+ documentation](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html) | Official docs for the AI Hat: Hailo-8/8L NPU (13/26 TOPS), Pi 5 integration — the target of `edge_deployment_node.py`. |
| [Raspberry Pi AI software guide](https://www.raspberrypi.com/documentation/computers/ai.html) | Installing `hailo-all`, running models on the NPU — the deployment steps Live Phase 4 automates. |
| [Hailo Developer Zone](https://hailo.ai/developer-zone/) | Dataflow Compiler and Model Zoo for converting ONNX models into Hailo `.hef` binaries that run on the AI Hat. |
| [Raspberry Pi Camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html) | Sensor details (e.g. Camera Module 2's IMX219: 3.68 mm sensor width — the default in `WorkspaceConstraints.sensor_width_mm`). |
| [Angle of view / pinhole camera geometry (Wikipedia)](https://en.wikipedia.org/wiki/Angle_of_view) | The `FOV = 2·atan(w / 2f)` relation implemented and unit-tested in `camera_lens_advisor.py`. |
| [ros-drivers/usb_cam (GitHub)](https://github.com/ros-drivers/usb_cam) | A standard V4L2 USB camera driver for ROS 2 — the live replacement for `mock_usb_camera_hil.py`. |

## 6. Testing tools

| Reference | Why it matters here |
|---|---|
| [GoogleTest primer](https://google.github.io/googletest/primer.html) | `TEST`, `EXPECT_*` vs `ASSERT_*`, `EXPECT_NEAR` — used by the three C++ suites in `test/`. |
| [pytest documentation](https://docs.pytest.org/en/stable/) | Fixtures and `@pytest.mark.parametrize`, used by `test_camera_lens_advisor.py` and `test_mock_onnx_policy.py`. |
| [ament_lint / ament_lint_auto (GitHub)](https://github.com/ament/ament_lint) | The linters (flake8, pep257, cpplint, uncrustify, xmllint, copyright) that run as part of `colcon test` via `ament_lint_auto_find_test_dependencies()`. |
| [ROS 2 testing overview (Jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html) | How unit, launch, and lint tests fit together in a ROS 2 package. |

## 7. Platform

| Reference | Why it matters here |
|---|---|
| [NVIDIA DGX Spark](https://www.nvidia.com/en-us/products/workstations/dgx-spark/) | The development platform named in `spec.md` (Ubuntu 24.04, DGX OS) hosting Isaac Sim/Lab and the ROS 2 Jazzy container. |
| [Isaac ROS getting started](https://nvidia-isaac-ros.github.io/getting_started/index.html) | Setting up the `isaac_ros-dev` containerized workspace referenced by the "Run Phase N manually" commands in `README.md`. |
