# src/

Every ROS 2 package lives here as its own subfolder. `colcon build` scans this
folder recursively and builds whatever it finds a `package.xml` in.

## Current / planned packages

| Package | Type | Purpose |
|---|---|---|
| `rover_interfaces` | ament_cmake | Shared custom `.msg` / `.srv` / `.action` definitions. Every other package depends on this instead of defining its own messages. |
| `rover_bringup` | ament_python or ament_cmake | Top-level launch files that start the whole rover stack. Contains no node logic of its own. |
| `rover_drive` | ament_python | Drive subsystem. See `rover_drive/README.md` — this is the reference example package. |
| `rover_arm` | — | Arm subsystem. |
| `rover_odom` | — | Odometry subsystem. |
| `rover_perception` | — | Perception subsystem (cameras, sensors, detection). |
| `rover_comms` | — | Rover-side antenna/radio link. |
| `base_station_bringup` | — | Launches base station comms + rosbridge. |
| `base_station_comms` | — | Base-station-side antenna/radio link. |

## Adding a new package

```bash
cd src

# Python package
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs

# C++ package
ros2 pkg create --build-type ament_cmake <package_name> --dependencies rclcpp std_msgs
```

Then use `src/_package_template/` as a reference for which subfolders to add
(`config/`, `launch/`, `urdf/`, `meshes/`) depending on what the package needs.

## Rules of thumb

- One package = one subsystem responsibility. If a package is doing two unrelated
  things, split it.
- Custom messages always go in `rover_interfaces`, never defined inline in a
  functional package.
- Package names: lowercase, underscore-separated, prefixed `rover_` or
  `base_station_`.
