# _package_template/

**This is not a real package — it won't build.** It exists purely as a
reference. Copy whichever subfolders you actually need into your
new package and delete the rest.

## Which folders does my package need?

- Every package needs: `package.xml`, and either `setup.py`+`setup.cfg`
  (Python) or `CMakeLists.txt` (C++).
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
