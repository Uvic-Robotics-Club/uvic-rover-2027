#!/usr/bin/env python3
"""
terminal_input_node.py - Terminal input source for rover arm control.

Publishes commands to the rover arm control node via ROS2 topics.
This is one of several possible input sources (Xbox controller, network, etc.)

Topics published:
    /arm/cmd_joint          (sensor_msgs/JointState)    - absolute joint targets
    /arm/cmd_joint_relative (sensor_msgs/JointState)    - relative joint deltas
    /arm/cmd_ik_pose        (geometry_msgs/PoseStamped) - cartesian IK target

Modes:
    joint - command joints using move (relative) or moveto (absolute)
    ik    - enter a cartesian x y z target for IK solving
    jog   - continuous cartesian jogging with keyboard held down

Commands:
    mode joint      switch to joint mode
    mode ik         switch to IK mode
    mode jog        switch to jog mode
    home            move to calibrated home pose from config/config.yaml
                    (falls back to all zeros if not calibrated)
    calibrate | c   save current joint angles as the home pose
    status          show current joint positions
    help            print this message
    quit            exit

Joint mode input:
    move   <index> <deg> [<index> <deg> ...]   relative move
    moveto <index> <deg> [<index> <deg> ...]   absolute move

IK mode input:
    <x> <y> <z>    target in metres, rover frame

Jog mode keys (hold to move continuously, ESC to exit):
    W / S        +Z / -Z   (up / down)
    A / D        +Y / -Y   (left / right)
    I / K        +X / -X   (forward / backward)
    Q / E        wrist roll  - / +
    R / F        wrist pitch + / -
    Z / X        hand roll   - / +
    [ / ]        step size decrease / increase
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import math
import sys
import tty
import termios
import select
import time
from pathlib import Path

import yaml


JOINT_NAMES = [
    "turret_joint",       # 0
    "shoulder_joint",     # 1
    "elbow_joint",        # 2
    "wrist_roll_joint",   # 3
    "wrist_pitch_joint",  # 4
    "hand_joint",         # 5
]

NUM_JOINTS = len(JOINT_NAMES)

CONFIG_FILENAME = "config.yaml"


def find_config_path() -> Path:
    """
    Locate config/config.yaml in the rover_arm SOURCE folder.

    This script runs from the install/ copy, and colcon copies (not symlinks)
    config/ into install/ on every build — so writing there would be lost on a
    clean rebuild and never reach git. Instead, walk up from the installed share
    directory to the workspace root and use <ws>/src/rover_arm/config/.
    Falls back to the installed copy if the source tree can't be found
    (e.g. a deployed image without sources). Override with the 'config_path'
    ROS parameter if your layout differs.
    """
    share = Path(get_package_share_directory('rover_arm'))
    for parent in share.parents:
        if parent.name == 'install':
            src_pkg = parent.parent / 'src' / 'rover_arm'
            if (src_pkg / 'package.xml').is_file():
                return src_pkg / 'config' / CONFIG_FILENAME
            break
    return share / 'config' / CONFIG_FILENAME

# Jog parameters
JOG_RATE_HZ      = 10.0          # how often to publish while key is held
JOG_STEP_DEFAULT = 0.002         # metres per publish (5mm)
JOG_STEP_MIN     = 0.001         # 1mm minimum step
JOG_STEP_MAX     = 0.05          # 50mm maximum step
JOG_STEP_INCREMENT = 0.001       # step change per [ or ] press

# How far the jog target may lead the actual tool tip. Keeps the target from
# running away from the arm (e.g. while it lags or sits against a limit), so
# reversing direction responds immediately.
JOG_MAX_LEAD = 0.02              # metres

# Wrist / hand keys in jog mode: relative joint moves per publish
JOG_WRIST_STEP_DEG = 2.0         # degrees per publish (20°/s when held at 10 Hz)
JOG_WRIST_KEYS = {
    'q': ("wrist_roll_joint", -1.0),
    'e': ("wrist_roll_joint", +1.0),
    'r': ("wrist_pitch_joint", +1.0),
    'f': ("wrist_pitch_joint", -1.0),
    'z': ("hand_joint",       -1.0),
    'x': ("hand_joint",       +1.0),
}

# ANSI escape sequences for arrow keys
KEY_UP    = '\x1b[A'
KEY_DOWN  = '\x1b[B'
KEY_RIGHT = '\x1b[C'
KEY_LEFT  = '\x1b[D'
KEY_ESC   = '\x1b'


def print_help(mode: str):
    print(f"\n--- Rover Arm Terminal Input (mode: {mode}) ---")
    print()
    print("Mode switching:")
    print("  mode joint                              switch to joint mode")
    print("  mode ik                                 switch to IK mode")
    print("  mode jog                                switch to jog mode")
    print()
    print("Joint mode:")
    print("  move   <index> <deg> [<index> <deg> …]  relative move")
    print("  moveto <index> <deg> [<index> <deg> …]  absolute move")
    print()
    print("Home (any mode):")
    print("  home                                     move to calibrated home (config.yaml)")
    print("  calibrate | c                            save current joint angles as home")
    print()
    print("IK mode:")
    print("  <x> <y> <z>                              target in metres, rover frame")
    print()
    print("Jog mode (hold key to move continuously, ESC to exit):")
    print("  W / S        +Z / -Z   (up / down)")
    print("  A / D        +Y / -Y   (left / right)")
    print("  I / K        +X / -X   (forward / backward)")
    print("  Q / E        wrist roll  - / +")
    print("  R / F        wrist pitch + / -")
    print("  Z / X        hand roll   - / +")
    print("  [ / ]        decrease / increase step size")
    print()
    print("General:")
    print("  status                                   show current joint positions")
    print("  help                                     show this message")
    print("  quit                                     exit")
    print()
    print("Joint index map:")
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {i} = {name}")
    print()

def read_key_nonblocking(timeout: float = 0.0) -> str:
    """
    Read a keypress without blocking.
    Handles regular characters and ANSI escape sequences (arrow keys).
    More robust escape sequence handling with tighter timing.
    """
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return ''

    ch = sys.stdin.read(1)

    if ch != '\x1b':
        return ch

    # Got ESC — read ahead with a tight timeout to detect escape sequences
    # If nothing follows within 20ms, it's a bare ESC keypress
    ready2, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not ready2:
        return KEY_ESC  # bare ESC

    ch2 = sys.stdin.read(1)
    if ch2 != '[':
        # ESC followed by something unexpected — treat as ESC
        return KEY_ESC

    # Read the final character of the sequence
    ready3, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not ready3:
        return KEY_ESC

    ch3 = sys.stdin.read(1)
    return '\x1b[' + ch3   


class TerminalInputNode(Node):

    def __init__(self):
        super().__init__('terminal_input_node')

        self.mode = "joint"

        # Current cartesian jog target [x, y, z] in metres.
        # Seeded from /arm/end_effector_pose each time jog mode is entered.
        self.jog_target = None
        self.jog_step = JOG_STEP_DEFAULT

        # Where the calibrated home pose is saved/loaded
        self.declare_parameter('config_path', '')
        override = self.get_parameter('config_path').value
        self.config_path = Path(override).expanduser() if override else find_config_path()
        self.get_logger().info(f"Home config file: {self.config_path}")

        self.joint_pub = self.create_publisher(JointState, '/arm/cmd_joint', 10)
        self.joint_relative_pub = self.create_publisher(
            JointState, '/arm/cmd_joint_relative', 10
        )
        self.ik_pub = self.create_publisher(PoseStamped, '/arm/cmd_ik_pose', 10)

        self.last_feedback = None
        self.create_subscription(
            JointState, '/arm/joint_states', self._on_joint_states, 10
        )

        # Subscribe to IK status so jog mode can detect when target is unreachable.
        # ik_failed latches on any failure until jog mode consumes it.
        self.last_ik_status = "OK"
        self.ik_failed = False
        self.create_subscription(String, '/arm/ik_status', self._on_ik_status, 10)

        self.last_ee_pose = None
        self.create_subscription(
            PoseStamped, '/arm/end_effector_pose', self._on_ee_pose, 10
        )

        self.get_logger().info("Terminal input node ready.")

    def _on_joint_states(self, msg: JointState):
        self.last_feedback = msg

    def _on_ik_status(self, msg: String):
        self.last_ik_status = msg.data
        if msg.data.startswith("IK failed"):
            self.ik_failed = True

    def _on_ee_pose(self, msg: PoseStamped):
        self.last_ee_pose = msg 

    def _process_incoming(self, max_callbacks: int = 20):
        """
        Handle messages that are waiting, without blocking.

        rclpy only delivers messages when the node is spun, and spin_once()
        handles ONE callback. Joint states, EE pose and IK status each arrive
        at 20 Hz (~60 msgs/s), so jog mode calls this every loop pass to
        stay current. Single-threaded on purpose — no background spinner.
        """
        for _ in range(max_callbacks):
            rclpy.spin_once(self, timeout_sec=0.0)

    def _current_tip(self):
        """Latest tool tip position [x, y, z], or None."""
        if self.last_ee_pose is None:
            return None
        p = self.last_ee_pose.pose.position
        return [p.x, p.y, p.z]

    def print_status(self):
        """Print current joint positions in degrees."""
        if self._wait_for_fresh('last_feedback', timeout_sec=2.0) is None:
            print("  No joint state feedback received within 2 s.")
            return

        print("\n--- Current Joint Positions ---")
        for name, pos_rad in zip(self.last_feedback.name, self.last_feedback.position):
            print(f"  {name:20s} {math.degrees(pos_rad):7.1f}°  ({pos_rad:.4f} rad)")
        print()

    # ============================
    # Publishers
    # ============================

    def send_move(self, joint_indices: list, degrees: list, log: bool = True):
        """Relative move — one or more joints simultaneously."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [JOINT_NAMES[i] for i in joint_indices]
        msg.position = [math.radians(d) for d in degrees]
        self.joint_relative_pub.publish(msg)
        if log:
            self.get_logger().info(
                f"move: {', '.join(f'{JOINT_NAMES[i]} {d:+.1f}°' for i, d in zip(joint_indices, degrees))}"
            )

    def send_moveto(self, joint_indices: list, degrees: list):
        """Absolute move — one or more joints simultaneously."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [JOINT_NAMES[i] for i in joint_indices]
        msg.position = [math.radians(d) for d in degrees]
        self.joint_pub.publish(msg)
        self.get_logger().info(
            f"moveto: {', '.join(f'{JOINT_NAMES[i]} → {d:.1f}°' for i, d in zip(joint_indices, degrees))}"
        )

    def send_home(self):
        """Move to the calibrated home pose (joint angles) from config.yaml."""
        try:
            positions = self._load_home_joints()
        except ValueError as e:
            # File exists but is broken — don't guess a pose, refuse to move.
            print(f"  Error: {e}\n  Home NOT sent. Fix the file or run 'calibrate'.")
            return

        if positions is None:
            print(f"  No calibrated home found at {self.config_path}.\n"
                  "  Using all zeros (arm fully upright — a singular pose, IK/jog "
                  "will struggle from here). Run 'calibrate' to set a bent home.")
            positions = [0.0] * NUM_JOINTS

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self.joint_pub.publish(msg)
        self.get_logger().info(
            "Home command sent: " +
            ", ".join(f"{n} {math.degrees(p):.1f}°" for n, p in zip(JOINT_NAMES, positions))
        )

    # ============================
    # Home calibration (config.yaml)
    # ============================

    def _read_config(self) -> dict:
        """Read config.yaml. Returns {} if missing. Raises ValueError if unreadable."""
        if not self.config_path.is_file():
            return {}
        try:
            with open(self.config_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as e:
            raise ValueError(f"could not read {self.config_path}: {e}")
        if not isinstance(data, dict):
            raise ValueError(f"{self.config_path} does not contain a YAML mapping")
        return data

    def _load_home_joints(self):
        """
        Return home joint angles (radians) in JOINT_NAMES order, or None if no
        home has been calibrated. Raises ValueError if the file is invalid.
        """
        home = self._read_config().get('home')
        if home is None:
            return None
        joints = home.get('joints') if isinstance(home, dict) else None
        if not isinstance(joints, dict):
            raise ValueError(f"'home.joints' missing or invalid in {self.config_path}")

        missing = [n for n in JOINT_NAMES if n not in joints]
        if missing:
            raise ValueError(f"home pose is missing joints {missing} in {self.config_path}")
        try:
            return [float(joints[n]) for n in JOINT_NAMES]
        except (TypeError, ValueError):
            raise ValueError(f"home pose has a non-numeric joint value in {self.config_path}")

    def calibrate_home(self):
        """Save the arm's CURRENT joint angles as the home pose in config.yaml."""
        js = self._wait_for_fresh('last_feedback', timeout_sec=2.0)
        if js is None:
            print("  Error: no /arm/joint_states received within 2 s. Nothing saved.")
            return

        current = dict(zip(js.name, js.position))
        missing = [n for n in JOINT_NAMES if n not in current]
        if missing:
            print(f"  Error: joint states are missing {missing}. Nothing saved.")
            return

        print("\n--- Current pose ---")
        for n in JOINT_NAMES:
            print(f"  {n:20s} {math.degrees(current[n]):7.1f}°  ({current[n]:.4f} rad)")

        try:
            config = self._read_config()  # keep any other keys in the file
        except ValueError as e:
            print(f"  Warning: {e}\n  It will be replaced.")
            config = {}

        if 'home' in config:
            print(f"\n  This will OVERWRITE the existing home in {self.config_path}")
        answer = input("  Save this as home? [y/N] ").strip().lower()
        if answer not in ('y', 'yes'):
            print("  Calibration cancelled.")
            return

        config['home'] = {
            'joints': {n: round(float(current[n]), 6) for n in JOINT_NAMES},
        }

        header = (
            "# rover_arm configuration\n"
            "# home.joints: home pose in radians, keyed by URDF joint name.\n"
            "#   Written by the 'calibrate' command in terminal_input_node.py.\n"
        )

        try:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            # Write to a temp file then rename, so a crash can't leave a half-written config
            tmp = self.config_path.with_suffix('.yaml.tmp')
            with open(tmp, 'w') as f:
                f.write(header)
                yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)
            tmp.replace(self.config_path)
        except OSError as e:
            print(f"  Error writing {self.config_path}: {e}")
            return

        print(f"  Home saved to {self.config_path}")

    def send_ik_command(self, x: float, y: float, z: float, log: bool = True):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.ik_pub.publish(msg)
        if log:
            self.get_logger().info(f"IK command: x={x:.3f} y={y:.3f} z={z:.3f}")

    # ============================
    # Jog mode
    # ============================

    def run_jog_mode(self):
        """
        Blocking jog loop. Runs until ESC is pressed.
        Uses raw terminal input so keys are read immediately without Enter.
        Restores terminal state on exit.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        # Seed jog target from the CURRENT end effector position.
        # Jogging from anything other than the real pose makes the IK goal
        # point somewhere unrelated to the keys pressed, so refuse instead.
        ee_pose = self._wait_for_fresh('last_ee_pose', timeout_sec=2.0)
        if ee_pose is None:
            print("\n  Error: no /arm/end_effector_pose received within 2 s. "
                  "Is arm_control_node running with a valid IK solver? "
                  "Not entering jog mode.")
            return

        self.jog_target = [
            ee_pose.pose.position.x,
            ee_pose.pose.position.y,
            ee_pose.pose.position.z,
        ]
        print(f"\n  Seeded from current EE position: "
            f"x={self.jog_target[0]:.3f} "
            f"y={self.jog_target[1]:.3f} "
            f"z={self.jog_target[2]:.3f}")

        # Send seed as initial goal
        self.send_ik_command(*self.jog_target)
        self.ik_failed = False
        need_reseed = False  # set when a wrist key moves the tip

        print("\n--- Jog Mode ---")
        print(f"  Starting target: x={self.jog_target[0]:.3f} "
            f"y={self.jog_target[1]:.3f} z={self.jog_target[2]:.3f}")
        print(f"  Step size: {self.jog_step*1000:.1f} mm")
        print("  W/S=Z  A/D=Y  I/K=X  Q/E=wrist roll  R/F=wrist pitch  Z/X=hand roll  "
              "[/]=step  ESC=exit")
        print()

        jog_interval = 1.0 / JOG_RATE_HZ
        last_publish_time = 0.0
        last_key = ''

        try:
            tty.setraw(fd)

            while True:
                key = read_key_nonblocking(timeout=jog_interval)

                # Pick up the latest tip pose and IK status every pass
                self._process_incoming()
                tip = self._current_tip()

                # Exit jog mode
                if key == KEY_ESC:
                    break

                # Step size adjustment — restore cooked mode briefly to print
                if key == '[':
                    self.jog_step = max(JOG_STEP_MIN, self.jog_step - JOG_STEP_INCREMENT)
                    # Print without newline issues in raw mode
                    sys.stdout.write(f"\r  Step: {self.jog_step*1000:.1f} mm    \r")
                    sys.stdout.flush()
                    continue

                if key == ']':
                    self.jog_step = min(JOG_STEP_MAX, self.jog_step + JOG_STEP_INCREMENT)
                    sys.stdout.write(f"\r  Step: {self.jog_step*1000:.1f} mm    \r")
                    sys.stdout.flush()
                    continue

                now = time.monotonic()
                # Publish immediately on first press, then at JOG_RATE_HZ while held
                due = key != last_key or (now - last_publish_time) >= jog_interval

                # Wrist / hand keys — direct relative joint moves, not IK
                if key in JOG_WRIST_KEYS:
                    if due:
                        joint, sign = JOG_WRIST_KEYS[key]
                        self.send_move([JOINT_NAMES.index(joint)],
                                       [sign * JOG_WRIST_STEP_DEG], log=False)
                        # Rotating the wrist moves the tip. Re-seed from the real
                        # tip on the next XYZ key so IK doesn't drag it back.
                        need_reseed = True
                        sys.stdout.write(f"\r  {joint} {sign * JOG_WRIST_STEP_DEG:+.0f}°"
                                         f"                              \r")
                        sys.stdout.flush()
                        last_publish_time = now
                        last_key = key
                    continue

                # Map key to cartesian delta
                delta = self._key_to_delta(key)

                if delta is None:
                    last_key = ''
                    continue

                if due:
                    at_limit = False
                    # Restart the target from where the tip ACTUALLY is when:
                    #  - IK couldn't reach the last target (limit / edge of reach),
                    #    so the target never piles up beyond the limit
                    #  - a different key is pressed, so reversing direction acts on
                    #    the first press instead of first undoing the arm's lag
                    #  - a wrist key moved the tip
                    new_key = key != last_key
                    if tip is not None and (self.ik_failed or need_reseed or new_key):
                        at_limit = self.ik_failed
                        self.jog_target = list(tip)
                        self.ik_failed = False
                        need_reseed = False

                    new_target = [t + d for t, d in zip(self.jog_target, delta)]

                    # Never let the target lead the real tip by more than JOG_MAX_LEAD
                    if tip is not None:
                        lead = [n - t for n, t in zip(new_target, tip)]
                        dist = math.sqrt(sum(v * v for v in lead))
                        if dist > JOG_MAX_LEAD:
                            new_target = [t + v * JOG_MAX_LEAD / dist for t, v in zip(tip, lead)]

                    self.jog_target = new_target
                    self.send_ik_command(*self.jog_target, log=False)

                    sys.stdout.write(
                        f"\r  x={self.jog_target[0]:+.3f} "
                        f"y={self.jog_target[1]:+.3f} "
                        f"z={self.jog_target[2]:+.3f}  "
                        f"step={self.jog_step*1000:.1f}mm"
                        f"{'  [LIMIT]' if at_limit else '         '}    \r"
                    )
                    sys.stdout.flush()
                    last_publish_time = now
                    last_key = key

        finally:
            # Always restore terminal even if something goes wrong
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("\n  Exited jog mode.")

    def _wait_for_fresh(self, attr: str, timeout_sec: float):
        """
        Spin until a NEW message lands in self.<attr>, or timeout. Returns it or None.

        A single spin_once() only services one callback from ANY subscription
        (joint_states, ik_status, ee_pose), and this node otherwise only spins
        between typed commands — so cached messages may be None or stale.
        Clearing the attribute first guarantees the returned message is current.
        """
        setattr(self, attr, None)
        deadline = time.monotonic() + timeout_sec
        while getattr(self, attr) is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return getattr(self, attr)

    def _key_to_delta(self, key: str):
        """Map a keypress to an [dx, dy, dz] cartesian delta. Returns None for unknown keys."""
        s = self.jog_step
        mapping = {
            'w':  [ 0,  0,  s],   # +Z up
            's':  [ 0,  0, -s],   # -Z down
            'a':  [ 0,  s,  0],   # +Y left
            'd':  [ 0, -s,  0],   # -Y right
            'i':  [ s,  0,  0],   # +X forward
            'k':  [-s,  0,  0],   # -X backward
        }
        return mapping.get(key, None)

    # ============================
    # Input parsing
    # ============================

    def handle_input(self, raw: str) -> bool:
        """
        Parse and dispatch one line of terminal input.
        Returns False if the node should exit, True otherwise.
        """
        parts = raw.strip().split()

        if not parts:
            return True

        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            return False

        if cmd in ("help", "h", "?"):
            print_help(self.mode)
            return True

        if cmd == "mode":
            if len(parts) < 2 or parts[1].lower() not in ("joint", "ik", "jog"):
                print("  Usage: mode joint | mode ik | mode jog")
            else:
                self.mode = parts[1].lower()
                print(f"  Switched to {self.mode} mode.")
                if self.mode == "jog":
                    self.run_jog_mode()
                    # After ESC, drop back to joint mode
                    self.mode = "joint"
                    print("  Returned to joint mode.")
            return True

        if cmd == "status":
            self.print_status()
            return True

        if cmd == "home":
            self.send_home()
            return True

        if cmd in ("calibrate", "c"):
            self.calibrate_home()
            return True

        if cmd == "seed":
            if len(parts) != 4:
                print("  Usage: seed <x> <y> <z>")
                return True
            try:
                self.jog_target = [float(parts[1]), float(parts[2]), float(parts[3])]
                print(f"  Jog seed set to x={self.jog_target[0]:.3f} "
                    f"y={self.jog_target[1]:.3f} z={self.jog_target[2]:.3f}")
            except ValueError:
                print("  Error: x, y, z must be numbers.")
            return True

        if self.mode == "joint":
            self._parse_joint(cmd, parts[1:])
        elif self.mode == "ik":
            self._parse_ik(parts)

        return True

    def _parse_joint(self, cmd: str, args: list):
        if cmd not in ("move", "moveto"):
            print(f"  Unknown joint command '{cmd}'. Use move, moveto, home, or calibrate.")
            return

        if len(args) < 2 or len(args) % 2 != 0:
            print(f"  Usage: {cmd} <index> <deg> [<index> <deg> …]")
            return

        joint_indices = []
        degrees = []

        for i in range(0, len(args), 2):
            try:
                joint_index = int(args[i])
                deg = float(args[i + 1])
            except ValueError:
                print(f"  Error: expected integer index and float degrees, "
                      f"got '{args[i]} {args[i+1]}'")
                return

            if joint_index < 0 or joint_index >= NUM_JOINTS:
                print(f"  Error: joint index {joint_index} out of range "
                      f"(0–{NUM_JOINTS - 1})")
                return

            joint_indices.append(joint_index)
            degrees.append(deg)

        if cmd == "move":
            self.send_move(joint_indices, degrees)
        else:
            self.send_moveto(joint_indices, degrees)

    def _parse_ik(self, parts: list):
        if len(parts) != 3:
            print("  IK mode usage: <x> <y> <z>  (metres)")
            return

        try:
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("  Error: x, y, z must be numbers.")
            return

        self.send_ik_command(x, y, z)
        self.get_logger().info(f"IK command: x={x:.3f} y={y:.3f} z={z:.3f}")


def main():
    rclpy.init()
    node = TerminalInputNode()

    print_help(node.mode)

    try:
        while rclpy.ok():
            try:
                raw = input(f"[{node.mode}]> ")
            except EOFError:
                break

            if not node.handle_input(raw):
                break

            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()