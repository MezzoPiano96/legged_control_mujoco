# legged_control (MuJoCo fork)

A fork of [qiayuanliao/legged_control](https://github.com/qiayuanliao/legged_control)
(NMPC + WBC control stack for legged robots, built on
[OCS2](https://github.com/leggedrobotics/ocs2) and ros-control). All credit for
the control framework itself — NMPC, WBC, state estimation — goes to the
original authors; see their repo for the full writeup, math, and citations if
you use this academically. This fork only adds one thing: **a MuJoCo backend
for the Unitree A1**, alongside the original Gazebo one, in `legged_mujoco/`
(a new package, not part of upstream).

## Quick start (MuJoCo)

### 1. Clone this repo and its OCS2 dependencies as siblings

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone git@github.com:MezzoPiano96/legged_control_mujoco.git legged_control
git clone git@github.com:leggedrobotics/ocs2.git
git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
```

### 2. Build the Docker image (ROS Noetic + MuJoCo SDK)

```bash
cd legged_control/docker
docker build -t legged_control:noetic .
```

### 3. Run the container

```bash
xhost +local:root   # redo this every login, does not persist

docker run -it -d \
  --name legged_ws \
  --network host \
  --ipc host \
  --gpus all \
  --device /dev/dri \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/catkin_ws/src:/home/catkin_ws/src \
  --restart unless-stopped \
  legged_control:noetic
```

(Why each flag exists, and what breaks without it, is in
[`docker/README.md`](docker/README.md) — skip it unless something's not working.)

### 4. Build the workspace

```bash
docker exec -it legged_ws bash
source /opt/ros/noetic/setup.bash
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
catkin build legged_controllers legged_unitree_description legged_mujoco
source devel/setup.bash
```

### 5. Launch the sim (this terminal stays open — you type gaits into it)

```bash
export ROBOT_TYPE=a1
roslaunch legged_mujoco mujoco_sim.launch
```

### 6. In a second terminal, load and start the controller

```bash
docker exec -it legged_ws bash
source /opt/ros/noetic/setup.bash
rosservice call /controller_manager/switch_controller \
  "{start_controllers: ['controllers/legged_cheater_controller'], stop_controllers: [], strictness: 2, start_asap: false, timeout: 0.0}"
```

### 7. Make it stand up

The robot stays crouched (~0.06 m) until it receives at least one `/cmd_vel` —
this is expected controller behavior, not a bug:

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

It should rise to ~0.30 m within a couple seconds.

### 8. Make it walk

- In the terminal from step 5, type a gait: `stance` or `trot`.
- In a third terminal, drive it with the keyboard:
  ```bash
  docker exec -it legged_ws bash
  source /opt/ros/noetic/setup.bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
  ```
  `i`/`,` forward/back, `j`/`l` turn, `k` stop.

### Optional: visualize

```bash
docker exec -it legged_ws bash
source devel/setup.bash
python3 src/legged_control/legged_mujoco/scripts/mujoco_viewer.py
```

## If something goes wrong / after a fall

Fall-recovery steps, why each `docker run` flag exists, and the list of bugs
fixed in `LeggedHWMujoco` during the migration are in
[`legged_mujoco/TROUBLESHOOTING.md`](legged_mujoco/TROUBLESHOOTING.md) and
[`docker/README.md`](docker/README.md).

## License

BSD-3-Clause, same as upstream — see [`LICENSE`](LICENSE).
