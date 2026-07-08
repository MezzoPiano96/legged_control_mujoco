# legged_control (MuJoCo fork)

A fork of [qiayuanliao/legged_control](https://github.com/qiayuanliao/legged_control)
(NMPC + WBC control stack for legged robots, built on
[OCS2](https://github.com/leggedrobotics/ocs2) and ros-control). All credit for
the control framework itself — NMPC, WBC, state estimation — goes to the
original authors; see their repo for the full writeup, math, and citations if
you use this academically. This fork only adds one thing: **a MuJoCo backend
for the Unitree A1**, alongside the original Gazebo one.

## What's added

`legged_mujoco/` — a new package (not in the upstream repo) implementing a
`hardware_interface::RobotHW` (`LeggedHWMujoco`) that runs the exact same
controller stack against MuJoCo instead of Gazebo, plus a standalone debug
viewer. Everything else in this repo is unmodified upstream code.

## Reproducing this on MuJoCo

1. **Get the OCS2 dependencies** (same as upstream — clone
   [`ocs2`](https://github.com/leggedrobotics/ocs2),
   [`pinocchio`](https://github.com/leggedrobotics/pinocchio),
   [`hpp-fcl`](https://github.com/leggedrobotics/hpp-fcl), and
   [`ocs2_robotic_assets`](https://github.com/leggedrobotics/ocs2_robotic_assets)
   as siblings of this repo in the same catkin workspace `src/`).
2. **Build the environment**: either follow
   [`docker/README.md`](docker/README.md) (the containerized setup this was
   built and tested in — recommended, includes the MuJoCo SDK install), or
   install the MuJoCo C++ SDK natively and point `legged_mujoco/CMakeLists.txt`
   at it.
3. **Build the packages**:
   ```bash
   catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
   catkin build legged_controllers legged_unitree_description legged_mujoco
   ```
4. **Run it** — full instructions, the gotchas (e.g. the robot won't stand up
   until you publish one `/cmd_vel`), and fall-recovery steps are in
   [`legged_mujoco/TROUBLESHOOTING.md`](legged_mujoco/TROUBLESHOOTING.md).

## License

BSD-3-Clause, same as upstream — see [`LICENSE`](LICENSE).
