# rover_interfaces

Shared custom message/service/action definitions, used by every other package.

Avoids circular dependencies.

## Adding a new message

1. Add a `.msg` (or `.srv` / `.action`) file to `msg/` / `srv/` / `action/`
2. Register it in `CMakeLists.txt` under `rosidl_generate_interfaces(...)`
3. `colcon build --packages-select rover_interfaces`
4. Any package using it needs `rover_interfaces` in its `package.xml`
   (`<depend>rover_interfaces</depend>`) and imports like:
   ```python
   from rover_interfaces.msg import DriveCommand
   ```
