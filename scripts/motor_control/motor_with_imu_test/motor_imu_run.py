#!/usr/bin/env python3
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import can
import n100
from robonex_common.can import FeedbackHub, Motor
from robonex_common.joints import ACTUATED_JOINTS
from robonex_common.motors import MOTOR_SPECS
from robonex_common.protocol import DEFAULT_INTERFACE, HOST_ID, clamp

DEG = math.pi / 180.0

MOTIONS = {
    +1: {
        "label": "Motion 1 (forward tilt)",
        "targets": {
            1: -0.1545,
            2: +0.3979,
            3: -0.0318,
            4: -0.5482,
            5: -0.0001,
            6: +0.0000,
            7: -0.0398,
            8: +0.0188,
            9: +0.0629,
            10: +0.0000,
            11: -0.0000,
            12: -0.0000,
        },
    },
    -1: {
        "label": "Motion 2 (backward tilt)",
        "targets": {
            1: +0.0135,
            2: -0.0601,
            3: +0.0105,
            4: +0.0177,
            5: +0.0000,
            6: -0.0000,
            7: +0.0940,
            8: -0.4013,
            9: +0.0706,
            10: +0.5172,
            11: -0.0001,
            12: -0.0000,
        },
    },
}

MOUNT_ROLL_DEG = 180.0
PITCH_THRESHOLD_DEG = 10.0
PITCH_SIGN = 1
ZERO_TIME = 0.5
MOVE_TIME = 1.5
RATE = 100.0
MOVE_SPEED = 0.4
HOLD_KP = 40.0
HOLD_KD = 2.0
OVERSPEED_STOP = 2.0
FEEDBACK_TIMEOUT = 0.3
MOTOR_MODELS = {joint.motor_id: joint.motor_model for joint in ACTUATED_JOINTS}
JOINT_MAP = {joint.motor_id: joint.hardware_name for joint in ACTUATED_JOINTS}
CHANNEL_ID_RANGES = {
    "can0": range(1, 7),
    "can1": range(7, 13),
}
SPECS = MOTOR_SPECS


def channel_for_id(motor_id):
    for channel, id_range in CHANNEL_ID_RANGES.items():
        if motor_id in id_range:
            return channel
    raise ValueError(f"No CAN channel for motor ID {motor_id}")


def pump_and_check(hubs, motors):

    for hub in hubs:
        hub.pump()
    now = time.monotonic()
    for motor_id, motor in motors.items():
        if abs(motor.last_velocity) > OVERSPEED_STOP:
            raise RuntimeError(f"ID {motor_id} overspeed ({motor.last_velocity:+.2f} rad/s)")
        if motor.last_feedback_time <= 0.0:
            raise RuntimeError(f"ID {motor_id} never received feedback")
        age = now - motor.last_feedback_time
        if age > FEEDBACK_TIMEOUT:
            raise RuntimeError(
                f"ID {motor_id} feedback timeout ({age:.2f} s > {FEEDBACK_TIMEOUT} s)")


class Motion:


    def __init__(self, motors, hubs, start, targets, label):
        self.motors = motors
        self.hubs = hubs
        self.label = label
        self.start = dict(start)
        self.current = dict(start)
        self.target = {mid: targets.get(mid, start[mid]) for mid in motors}

        travel = max((abs(self.target[mid] - self.start[mid]) for mid in motors),
                     default=0.0)
        self.move_time = max(MOVE_TIME, travel / MOVE_SPEED)
        self.began = time.monotonic()

    def step(self, now):
        progress = min(1.0, (now - self.began) / self.move_time)
        smooth = progress * progress * (3.0 - 2.0 * progress)
        smooth_vel = 6.0 * progress * (1.0 - progress) / self.move_time

        for motor_id, motor in self.motors.items():
            travel = self.target[motor_id] - self.start[motor_id]
            position = self.start[motor_id] + travel * smooth
            self.current[motor_id] = position
            motor.control(
                pos=position,
                vel=clamp(travel * smooth_vel, -MOVE_SPEED, MOVE_SPEED),
                kp=HOLD_KP, kd=HOLD_KD,
            )
        pump_and_check(self.hubs, self.motors)
        return progress >= 1.0


def pitch_deg(sample, offset):
    return PITCH_SIGN * (sample.euler.pitch / DEG - offset)


def tilt_direction(pitch):
    if pitch > PITCH_THRESHOLD_DEG:
        return +1
    if pitch < -PITCH_THRESHOLD_DEG:
        return -1
    return 0


def print_imu(sample, offset):
    q, e = sample.orientation, sample.euler
    w, wr = sample.angular_velocity, sample.angular_velocity_raw
    a, m, g = sample.linear_acceleration, sample.magnetic_field, sample.projected_gravity
    pitch = pitch_deg(sample, offset)
    label = {0: "neutral", +1: "forward", -1: "backward"}[tilt_direction(pitch)]
    print(f"[{time.strftime('%H:%M:%S')}] seq {sample.seq}")
    print(f"  quat    w {q.w:+8.4f}  x {q.x:+8.4f}  y {q.y:+8.4f}  z {q.z:+8.4f}")
    print(f"  rpy     r {e.roll / DEG:+8.2f}  p {e.pitch / DEG:+8.2f}  "
          f"y {e.yaw / DEG:+8.2f}  [deg, raw]")
    print(f"  gyro    x {w.x:+8.4f}  y {w.y:+8.4f}  z {w.z:+8.4f}  [rad/s]")
    print(f"  gyroR   x {wr.x:+8.4f}  y {wr.y:+8.4f}  z {wr.z:+8.4f}  [rad/s, raw]")
    print(f"  accel   x {a.x:+8.4f}  y {a.y:+8.4f}  z {a.z:+8.4f}  [m/s^2]")
    print(f"  mag     x {m.x:+8.2e}  y {m.y:+8.2e}  z {m.z:+8.2e}  [T]")
    print(f"  gproj   x {g.x:+8.4f}  y {g.y:+8.4f}  z {g.z:+8.4f}")
    print(f"  temp {sample.imu_temperature:5.1f} C   pressure {sample.pressure:9.1f} Pa")
    print(f"  pitch   {pitch:+8.2f} deg (zero offset {offset:+.2f})  "
          f"state {label}  threshold +-{PITCH_THRESHOLD_DEG:.0f}\n")


def measure_pitch_offset(driver):
    total = 0.0
    count = 0
    last_seq = 0
    end = time.monotonic() + ZERO_TIME
    while time.monotonic() < end:
        sample = driver.wait_for_sample(timeout=0.2, last_seq=last_seq)
        if sample is None:
            continue
        last_seq = sample.seq
        total += sample.euler.pitch / DEG
        count += 1
    if count == 0:
        raise RuntimeError("No IMU samples were received for zero calibration.")
    return total / count


def main():
    parser = argparse.ArgumentParser(
        description="Trigger motor motions from IMU pitch.")
    parser.add_argument("--imu-port", default="/dev/ttyUSB0", help="IMU serial port")
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_ID_RANGES),
                        choices=list(CHANNEL_ID_RANGES), help="CAN channels to use")
    parser.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    args = parser.parse_args()

    driver = n100.ImuDriver(n100.DriverConfig(
        port=args.imu_port,
        mount_rotation=n100.Quat.from_axis_angle_x(MOUNT_ROLL_DEG * DEG),
    ))
    try:
        driver.start()
    except RuntimeError as error:
        print(f"IMU start failed: {error}")
        print(f"Check the port: ls /dev/ttyUSB* /dev/ttyACM*  "
              f"(permissions: sudo chmod 666 {args.imu_port})")
        return 1

    buses = {}
    motors = {}
    enabled = False
    try:
        print(f"Waiting for the first IMU sample on {args.imu_port}...")
        if driver.wait_for_sample(timeout=3.0) is None:
            print(f"No IMU data within 3 seconds: {driver.last_error() or 'No response'}")
            return 1
        print(f"IMU stream ready. Applied mount rotation Rx({MOUNT_ROLL_DEG:.0f} deg).\n")

        print(f"Calibrating zero for {ZERO_TIME:.1f} s. Keep the robot upright...")
        offset = measure_pitch_offset(driver)
        print(f"  Pitch zero: {offset:+.2f} deg\n")
        print_imu(driver.latest(), offset)

        g = driver.latest().projected_gravity
        if g.z > 0:
            print(f"Warning: upright projected gravity z is {g.z:+.3f}; expected about -1.")
            print("Check MOUNT_ROLL_DEG.\n")

        motor_ids = [mid for mid in MOTOR_MODELS if channel_for_id(mid) in args.channels]
        for channel in sorted({channel_for_id(mid) for mid in motor_ids}):
            buses[channel] = can.Bus(channel=channel, interface=DEFAULT_INTERFACE)
        for motor_id in motor_ids:
            motors[motor_id] = Motor(buses[channel_for_id(motor_id)], motor_id,
                                     SPECS[MOTOR_MODELS[motor_id]])

        hub_by_channel = {
            channel: FeedbackHub(
                bus,
                [m for mid, m in motors.items() if channel_for_id(mid) == channel],
                HOST_ID,
            )
            for channel, bus in buses.items()
        }
        hubs = list(hub_by_channel.values())

        print("Reading current positions...\n")
        pose = {}
        for motor_id, motor in motors.items():
            current = motor.read_mech_position(timeout=0.3)
            if current is None:
                print(f"Motor ID {motor_id} did not respond.")
                return 1
            pose[motor_id] = current
            print(f"  {motor_id:>3}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                  f"{current:+8.4f} rad ({math.degrees(current):+7.2f} deg)")

        print("\nThe real motors will move.")
        if not args.yes:
            if not sys.stdin.isatty():
                print("Confirmation requires an interactive terminal or --yes.")
                return 1
            try:
                input("Press Enter to start, or Ctrl-C to cancel: ")
            except (KeyboardInterrupt, EOFError):
                print("\nCancelled.")
                return 0

        print("\nEnabling...")
        enabled = True
        for motor_id, motor in motors.items():
            motor.write_run_mode_operation()
            time.sleep(0.005)
            motor.enable()
            current = hub_by_channel[channel_for_id(motor_id)].wait_for(motor_id, timeout=0.3)
            if current is None:
                raise RuntimeError(f"ID {motor_id} has no live feedback")
            pose[motor_id] = current
            motor.control(pos=current, vel=0.0, kp=HOLD_KP, kd=HOLD_KD)
        print(f"\n{RATE:.0f} Hz loop started. Press Ctrl-C to stop.\n")

        period = 1.0 / RATE
        motion = None
        last_fired = 0
        last_print = 0.0
        next_tick = time.monotonic()

        while True:
            next_tick += period
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            now = time.monotonic()

            if motion is not None:
                if motion.step(now):
                    pose = motion.target
                    print(f"    {motion.label} complete\n")
                    motion = None
            else:
                for motor_id, motor in motors.items():
                    motor.control(pos=pose[motor_id], vel=0.0, kp=HOLD_KP, kd=HOLD_KD)
                pump_and_check(hubs, motors)

            sample = driver.latest()
            if sample is None:
                continue
            if not driver.is_running:
                raise RuntimeError(f"IMU reader stopped: {driver.last_error() or 'Unknown error'}")

            if now - last_print >= 1.0:
                last_print = now
                print_imu(sample, offset)

            pitch = pitch_deg(sample, offset)
            direction = tilt_direction(pitch)
            if direction == 0:
                last_fired = 0
            elif direction != last_fired:
                last_fired = direction
                spec = MOTIONS[direction]
                if motion is None:
                    start = pose
                    action = "Triggered"
                else:
                    start = motion.current
                    action = f"Switched during {motion.label}"
                motion = Motion(motors, hubs, start, spec["targets"], spec["label"])
                print(f"[{time.strftime('%H:%M:%S')}] {action}: {spec['label']}  "
                      f"(pitch {pitch:+.2f} deg, move time {motion.move_time:.1f} s)")

    except KeyboardInterrupt:
        print("\nStop requested.")
        return 0
    except RuntimeError as error:
        print(f"\nStopped: {error}")
        return 1
    except (OSError, can.CanError) as error:
        print(f"\nCAN error: {error}")
        print("Check that the interface is up:")
        print("  sudo modprobe gs_usb")
        for channel in args.channels:
            print(f"  sudo ip link set {channel} up type can bitrate 1000000")
        return 1
    finally:
        if enabled:
            print("Stopping motors...")
            for motor in motors.values():
                try:
                    motor.stop()
                except can.CanError:
                    pass
        for bus in buses.values():
            bus.shutdown()
        driver.stop()
        print("Stopped.")


if __name__ == "__main__":
    sys.exit(main())
