# measurements

## Structure

```text
measurements/
├── README.md
├── common.py
├── armature/
│   ├── armature.py
│   └── analyze_armature.py
├── check/
│   └── shutdown.py
├── damping/
│   ├── damping.py
│   └── analyze_damping.py
├── friction/
│   ├── friction.py
│   └── analyze_friction.py
├── torque/
│   └── torque.py
├── joint/
│   ├── read_joint_values.py
│   └── scan_joint_limits.py
└── noise/
    ├── motor/
    │   ├── motor_noise.py
    │   ├── analyze_can_noise.py
    │   └── analyze_can_rate.py
    └── imu/
        ├── CMakeLists.txt
        ├── imu_noise.py
        ├── n100_binding.cpp
        └── analyze_imu_noise.py
```

## armature

Armature is the rotational inertia that resists angular acceleration, measured in kg·m².

### `armature.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | - | Select the motor |
| - | `--model` | - | Select `rs02` or `rs03` |
| - | `--torques` | - | Set test torques in N·m |
| - | `--repeats` | `1` | Set repeats per torque |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |

```bash
# Example
python3 scripts/measurements/armature/armature.py --motor-id 11 --model rs02 --torques 0.3 0.5 1.0
```

### `analyze_armature.py`

```bash
# Example
python3 scripts/measurements/armature/analyze_armature.py "scripts/measurements/armature/data/*.csv"
```

## check

### `shutdown.py`

Applies velocity damping, disables the selected motors, and verifies their operating mode before power-off.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--ids` | `1~12` | Select motors to disable |
| - | `--brake-time` | `0.3` | Set braking time in seconds |
| - | `--kd` | `3.0` | Set braking damping gain |

```bash
# Example
python3 scripts/measurements/check/shutdown.py
```

## damping

Damping is the velocity-proportional resisting torque, measured in N·m/(rad/s).

### `damping.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | - | Select the motor |
| - | `--model` | - | Select `rs02` or `rs03` |
| - | `--speeds` | - | Set test speeds in rad/s |
| - | `--repeats` | `1` | Set repeats per speed |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |

```bash
# Example
python3 scripts/measurements/damping/damping.py --motor-id 4 --model rs03 --speeds 0.15 0.20 0.28
```

### `analyze_damping.py`

```bash
# Example
python3 scripts/measurements/damping/analyze_damping.py "scripts/measurements/damping/data/*.csv"
```

## friction

Friction is the breakaway torque required to start a stationary joint moving, measured in N·m.

### `friction.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | - | Select the motor |
| - | `--model` | - | Select `rs02` or `rs03` |
| - | `--signs` | `1 -1` | Select positive (`1`) or negative (`-1`) motor torque directions |
| - | `--repeats` | `1` | Set repeats per direction |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |

```bash
# Example
python3 scripts/measurements/friction/friction.py --motor-id 4 --model rs03
```

### `analyze_friction.py`

```bash
# Example
python3 scripts/measurements/friction/analyze_friction.py "scripts/measurements/friction/data/*.csv"
```

## torque

`torque.py` passively displays type `0x02` torque feedback from all 12 motors. It sends no CAN frames, so a motor controller such as `mujoco_to_real.py` must run separately. The displayed value is the motor controller's internal torque estimate, not an independent load-cell measurement.

### `torque.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--refresh` | `10` | Set the terminal refresh rate in Hz |
| - | `--stale-after` | `0.3` | Set the stale-feedback threshold in seconds |

```bash
# Example
python3 scripts/measurements/torque/torque.py
```

## joint

### `read_joint_values.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--watch` | Off | Continuously refresh joint values |

```bash
# Example
python3 scripts/measurements/joint/read_joint_values.py --watch
```

### `scan_joint_limits.py`

Tracks the minimum and maximum mechanical positions while each joint is moved by hand, then saves the results as CSV.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id`, `--motor-ids` | `1~12` | Select motors to scan |

```bash
# Example
python3 scripts/measurements/joint/scan_joint_limits.py --motor-id 5 6
```

## noise/motor

### `motor_noise.py`

Enables stationary motors with velocity damping and records type `0x02` position, velocity, torque, temperature, and timestamps to CSV.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `1~12` | Select one or more motors to capture |
| - | `--duration` | `60` | Set simultaneous capture time per active CAN channel |

```bash
# Example
python3 scripts/measurements/noise/motor/motor_noise.py --motor-id 1 2 3 --duration 60
```

### `analyze_can_noise.py`

Calculates per-motor position mean, position noise, peak-to-peak variation, and velocity noise from motor noise files.

```bash
# Example
python3 scripts/measurements/noise/motor/analyze_can_noise.py "scripts/measurements/noise/motor/data/*.csv"
```

### `analyze_can_rate.py`

Calculates successful response rate, missed replies, update frequency, and timing jitter for each motor and CAN channel.

```bash
# Example
python3 scripts/measurements/noise/motor/analyze_can_rate.py "scripts/measurements/noise/motor/data/*.csv"
```

## noise/imu

### Python Binding

```bash
# Example
cmake -S scripts/measurements/noise/imu \
  -B scripts/measurements/noise/imu/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(pwd)/.venv/bin/python"
cmake --build scripts/measurements/noise/imu/build -j
```

### `imu_noise.py`

Records stationary N100 raw and fused gyroscope data, acceleration, temperature, and timestamps to CSV.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--port` | `/dev/ttyUSB0` | Select the IMU serial port |
| - | `--duration` | `60` | Set capture time in seconds |

```bash
# Example
python3 scripts/measurements/noise/imu/imu_noise.py --duration 60
```

### `analyze_imu_noise.py`

Combines IMU noise files and calculates per-axis gyroscope bias and noise for raw and fused signals.

```bash
# Example
python3 scripts/measurements/noise/imu/analyze_imu_noise.py "scripts/measurements/noise/imu/data/*.csv"
```
