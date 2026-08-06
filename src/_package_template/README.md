# _package_template/

**This is not a real package — it won't build.** It exists purely as a
reference.

## Starting a new package

Run `ros2 pkg create --build-type ament_python <pkg_name>` (or
`ament_cmake` for C++) to scaffold a new package. Then, use the guide below to decide which additional folders (`/config`, `/launch`, `/udf`, etc.)

## What additional folders do I need?

- **Node code itself doesn't get its own folder.** It lives directly in the
  package's own module — `<pkg_name>/<pkg_name>/*.py` for Python or `src/`+`include/` for C++.
- Has tunable parameters? → add `config/`
- Has a way to start it standalone? → add `launch/`
- Defines the robot's physical shape/joints? → add `urdf/` (usually only in
  a `_description` package, not in every subsystem package)
- Has 3D models for visualization/simulation? → add `meshes/` (also usually
  only in `_description`)
- Has one-off utility scripts that aren't ROS nodes — not registered as a
  `console_scripts` entry point, not launched by anyone, just run manually
  by a developer (e.g. a calibration helper, a data conversion tool)? →
  add `scripts/`
- Defines custom messages/services/actions? → don't add `msg/`/`srv/`/`action/`
  here — that belongs in `rover_interfaces`, shared across all packages.
