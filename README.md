# roboplan
Modern and performant motion planning library based on Pinocchio.

> [!WARNING]
> This is an experimental, work-in-progress repository!

## Build instructions (colcon)

First, clone this repo to a valid ROS 2 workspace.

```bash
mkdir -p ~/roboplan_ws/src
cd ~/roboplan_ws/src
git clone https://github.com/sea-bass/roboplan.git
```

Source your favorite ROS distro and compile the package.

```bash
source /opt/ros/rolling/setup.bash
colcon build
```

Now you should be able to run a basic example.

```bash
source install/setup.bash
ros2 run roboplan test_pinocchio
```
