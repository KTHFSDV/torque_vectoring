# torque_vectoring_pkg (ROS2 Jazzy)

Torque vectoring for a four-wheel-drive electric Formula Student car. Each control cycle runs three stages:

1. **Force estimation** (`KinematicForceEstimator`): estimates tire forces from the vehicle state.
2. **Yaw control** (`PIDYawController`): computes a corrective yaw moment from the yaw rate error.
3. **Torque allocation** (`LdltTorqueAllocator`): distributes the total force and yaw moment demand across the four motors.

The stages are wired together in [src/main.cpp](src/main.cpp) and tuned in [include/config.hpp](include/config.hpp). Topics are listed in [docs/ros_messages.md](docs/ros_messages.md), and background on the algorithms is in [docs/tv.md](docs/tv.md) and [docs/tire_forces.md](docs/tire_forces.md).

## Dependencies

- `fs_msgs`, `ros2can_msgs` and `vehicle_params_cpp` must be in the colcon workspace.
- Eigen 3 (`libeigen3-dev`), which the [Dockerfile](Dockerfile) installs.

## Running with Docker

```bash
docker compose up --build
```

The entrypoint runs `colcon build` on the first startup and then launches [launch/launch.py](launch/launch.py). To check that the node is running:

```bash
docker exec -it <container> bash -c "source /ws/install/setup.bash && ros2 node list"
```

`ros2 node list` should show `/torque_vectoring_node`. The compose file also starts a noVNC service at `http://localhost:8080/vnc.html` for GUI tools like `rviz2`.

## Running natively

From the workspace root:

```bash
colcon build --packages-up-to torque_vectoring_pkg
source install/setup.bash
ros2 launch torque_vectoring_pkg launch.py
```

## CI

[docker-ci.yml](.github/workflows/docker-ci.yml) starts the compose stack, waits for `colcon build` to finish, and checks that `torque_vectoring_node` is running.

## Codebase Diagram

![Visualization of the codebase](./docs/codebase-diagram.svg)
