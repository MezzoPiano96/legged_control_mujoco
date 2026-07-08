# legged_mujoco: running it and known gotchas

## Running the sim

```bash
source /home/catkin_ws/devel/setup.bash
export ROBOT_TYPE=a1
roslaunch legged_mujoco mujoco_sim.launch
```

This brings up MuJoCo (headless physics + optional viewer, see below) and a
prompt to type a gait (`stance`, `trot`, `pace`, ...). Leave that terminal
open — you switch gaits by typing into it.

In another terminal, load and start the controller:

```bash
source /opt/ros/noetic/setup.bash
rosservice call /controller_manager/switch_controller \
  "{start_controllers: ['controllers/legged_cheater_controller'], stop_controllers: [], strictness: 2, start_asap: false, timeout: 0.0}"
```

### The robot won't stand up

`LeggedController::starting()` initializes the MPC target trajectory with a
zeroed state (target height = 0); only a `/cmd_vel` (or
`/move_base_simple/goal`) message ever pushes the real target height
(`comHeight` from `task.info`, 0.3 m for the A1) into the controller. If you
start the controller and never publish `/cmd_vel`, the robot just sits at
~0.06 m looking like it failed to stand — this is expected, not a bug. Fix:
publish one `/cmd_vel` (all-zero is fine):

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Optional viewer

```bash
python3 scripts/mujoco_viewer.py
```

A separate process that mirrors `/joint_states` + `/ground_truth/state` into
its own MuJoCo render window. It's read-only — closing it doesn't affect the
physics running inside `LeggedHWMujoco`.

### Recovering after a fall

1. In the gait terminal, switch to `stance` first (a controller restart does
   not clear "which gait is active" — if it fell during `trot`/`pace` it will
   immediately retry the same gait and likely fall again).
2. Reset physics state to the model's home keyframe:
   ```bash
   rosservice call /mujoco_hw/reset
   ```
3. Restart the controller (also resets the MPC's internal solver state, see
   below):
   ```bash
   rosservice call /controller_manager/switch_controller \
     "{start_controllers: ['controllers/legged_cheater_controller'], stop_controllers: [], strictness: 2, start_asap: false, timeout: 0.0}"
   ```
4. Publish a zero `/cmd_vel` again (see above) so it stands to full height.

If step 3 leaves the controller `stopped` instead of `running`, it fell again
immediately — repeat from step 1.

## Bugs found and fixed in `LeggedHWMujoco`

These were all real bugs in this package (not upstream `legged_control`, this
package doesn't exist upstream), fixed in place:

1. **Missing floor**: the model path pointed at bare `a1.xml` instead of
   `scene.xml` (which includes the ground plane + skybox).
2. **IMU quaternion order**: MuJoCo stores `(w, x, y, z)`; the code was
   feeding it into a field read as `(x, y, z, w)`.
3. **IMU angular velocity / linear acceleration in the wrong frame**: read
   straight from MuJoCo's world-frame `cvel`/`cacc` instead of rotating into
   the body frame an IMU actually reports in.
4. **`data_->cacc` silently stays zero** unless the model has an explicit
   `<sensor>` that needs it. Fixed by finite-differencing `cvel` instead.
5. **Site/body id confusion in `setupImu()`**: when IMU frame lookup fell back
   from site to body, the body id was stored in a field later used to index
   `site_bodyid[]` — an out-of-bounds read.
6. **Actuator/joint order mismatch** (the big one): the MJCF declares
   actuators in `FR, FL, RR, RL` order; the ROS `joints` param is in
   `LF, RF, LH, RH` order. `readJoints`/`writeJoints` indexed
   `data_->ctrl[]`/`actuator_force[]` by loop index directly, so each leg's
   torque landed on the wrong leg's actuator — the dominant cause of visible
   jitter/instability. Fixed by looking up the actuator id per joint id
   instead of assuming matching order. (Also had to change the MJCF actuators
   from `<position>` to `<motor>` — the C++ side computes torque itself and
   writes directly to `ctrl[]`, which only makes sense for direct-torque
   actuators.)
7. **`generate_urdf.sh`** wrote the URDF via a plain `>` redirect, which
   `mujoco_sim.launch`'s controller loader could read mid-write. Fixed with a
   temp-file + atomic `mv`.
8. **`legged_mujoco/CMakeLists.txt`** needs C++17 (MuJoCo 3.2's `mjspec.h`
   uses `std::byte`) — already set via `CMAKE_CXX_STANDARD` in that file.
9. **`mujoco_viewer.py` joint name mismatch**: it built its joint map from
   MuJoCo's own joint names (e.g. `FL_calf_joint`) but `/joint_states` uses
   ROS naming (e.g. `LF_KFE`) — the lookup always missed, so the viewer never
   actually mirrored anything (it just happened to sit at a plausible-looking
   default pose). Fixed with the same ROS↔MuJoCo name table used in the C++
   side.
10. **`mujoco_viewer.py` never mirrored the floating base**, only the 12 leg
    joints — its own copy of the model stayed frozen at the default spawn
    height. Fixed by also subscribing to `/ground_truth/state` and writing the
    base pose (`qpos[0:7]`).
11. **MPC warm-start not reset across controller restarts**:
    `LeggedController::starting()` only called `setTargetTrajectories()`, never
    `mpcMrtInterface_->resetMpcNode()`. The underlying SQP solver's warm-start
    trajectory survived across a ros_control stop/start cycle, so restarting
    the controller right after a fall fed the solver a corrupted starting
    point and it would immediately trip safety checks again — even from a
    clean physics reset. Fixed by calling
    `mpcMrtInterface_->resetMpcNode(target_trajectories)` instead, which
    internally resets the MPC before setting the new target.
12. **`/mujoco_hw/reset` service** (added, doesn't exist upstream): resets
    MuJoCo's `data_` to the model's home keyframe without restarting
    `roslaunch`. Two things were required for this to be safe: the actual
    `mj_resetDataKeyframe` call must run on the main control-loop thread
    (gated by an atomic flag set from the service callback, which otherwise
    runs on a separate AsyncSpinner thread and races with `mj_step()`); and it
    must zero both the joint effort commands *and* every hybrid joint's
    `(posDes_, velDes_, kp_, kd_, ff_)` — those survive a stopped controller
    and would otherwise immediately re-disturb the freshly reset pose.
13. **Joint-hold fallback in `writeJoints()`**: whenever a joint's hybrid
    command is all-zero (e.g. right after init or after `/mujoco_hw/reset`
    with no controller running yet), it now PD-holds toward the home keyframe
    angle instead of applying zero torque — otherwise the model collapses
    under gravity within ~0.5–2 s (it has joint damping but isn't passively
    stable).

## GPU rendering

If the viewer is chewing 300%+ CPU and looks slow, it's falling back to Mesa
software rendering. See [`docker/README.md`](../docker/README.md) for the
`docker run` flags that fix this.
