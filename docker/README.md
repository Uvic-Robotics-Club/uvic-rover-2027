# docker/

Container build files. Not ROS packages — colcon ignores this folder.

| File | Purpose |
|---|---|
| `Dockerfile` | Defines the dev image (ROS 2 Jazzy + build tools + non-root user). Multi-stage when we add a CI/CD later. |
| `entrypoint.sh` | Runs once as the container's PID 1. Sources the ROS 2 environment for **non-interactive** processes. |

