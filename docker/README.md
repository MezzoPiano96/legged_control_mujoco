# Docker environment (Gazebo + MuJoCo)

This is the environment this fork's MuJoCo migration was developed and tested
in. It's optional — if you'd rather install everything natively, follow the
main [README's Installation section](../README.md#installation) instead (it
already covers cloning OCS2/pinocchio/hpp-fcl/ocs2_robotic_assets as sibling
packages); this Dockerfile just wraps the same steps plus the MuJoCo SDK.

## Workspace layout

The container expects `/home/catkin_ws/src` to be bind-mounted to a directory
containing this repo **and its OCS2 siblings side by side**, e.g.:

```
your_ws/
├── legged_control/          (this repo)
├── ocs2/
├── ocs2_robotic_assets/
├── hpp-fcl/
└── pinocchio/
```

Clone those four siblings exactly as described in the main README's
Installation section, into the same parent directory as this repo.

## Build the image

```bash
cd docker
docker build -t legged_control:noetic .
```

## Run the container

```bash
xhost +local:root   # allow the container to open GUI windows on your X server

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
  -v /path/to/your_ws:/home/catkin_ws/src \
  --restart unless-stopped \
  legged_control:noetic
```

Notes on the flags:
- `--gpus all` + `--device /dev/dri` + the three `-e` vars: without these,
  Gazebo/MuJoCo rendering silently falls back to Mesa software rendering
  (`llvmpipe`), which is slow (300%+ CPU) and can segfault under load. These
  force it onto the actual NVIDIA GPU via GLVND.
- `--network host` / `--ipc host`: `host` networking avoids ROS's multi-host
  networking setup entirely (fine for single-machine use); `--ipc host` avoids
  an MIT-SHM crash X11 apps can hit under Docker's default IPC namespace.
- `xhost +local:root` does **not** persist across host reboots/logins — you
  need to re-run it every time before starting the container (the container
  itself, if given `--restart unless-stopped`, does not need to be manually
  restarted).

## Build the workspace

```bash
docker exec -it legged_ws bash
source /opt/ros/noetic/setup.bash
catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization legged_control legged_mujoco
```

See [`legged_mujoco/TROUBLESHOOTING.md`](../legged_mujoco/TROUBLESHOOTING.md)
for how to actually run the MuJoCo sim and known gotchas.
