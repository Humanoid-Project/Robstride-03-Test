# measurements

## Structure

```text
measurements/
├── README.md
├── common.py
├── armature/
│   ├── armature.py
│   ├── analyze_armature.py
│   └── data/
├── check/
│   └── shutdown.py
├── damping/
│   ├── damping.py
│   ├── analyze_damping.py
│   └── data/
├── friction/
│   ├── friction.py
│   ├── analyze_friction.py
│   └── data/
├── joint/
│   ├── read_joint_values.py
│   ├── read_joint_with_imu.py
│   ├── scan_joint_limits.py
│   └── joint_limit/
└── noise/
    ├── imu/
    │   ├── imu_capture.py
    │   ├── analyze_imu_noise.py
    │   └── data/
    └── can/
        ├── can_capture.py
        ├── analyze_can_noise.py
        ├── analyze_can_rate.py
        └── data/
```

<br>

## armature

### `armature.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--motor-id` | Yes | - | Target motor ID |
| `--model` | Yes | - | Motor model (`rs02`, `rs03`) |
| `--channel` | No | Auto from ID | Target CAN channel |
| `--torques` | Yes | - | List of test torques (N·m, signed) |
| `--repeats` | No | `1` | Repeats per torque |

```bash
# Example
python3 scripts/measurements/armature/armature.py \
  --motor-id 11 \
  --model rs02 \
  --channel can1 \
  --torques 0.3 0.5 1.0 1.5 2.0 \
  --repeats 1
```

### `analyze_armature.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `csv_files` | Yes | - | CSV file(s) to analyze, or a glob |
| `--skip-ms` | No | `15` | Startup transient to exclude (ms) |

```bash
# Example
python3 scripts/measurements/armature/analyze_armature.py \
  "scripts/measurements/armature/data/id11_rs02_*.csv"
```

<br>

## check

### `shutdown.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channels` | No | `can0 can1` | CAN channels to use |
| `--ids` | No | `1~12` (from selected channels) | Target motor IDs |
| `--interface` | No | `socketcan` | python-can interface |
| `--host-id` | No | `0xFD` | Host CAN ID |
| `--brake-time` | No | `0.3` | Braking time before disable (s); `0` disables immediately |
| `--kd` | No | `3.0` | Damping gain during the braking phase |

```bash
# Example
python3 scripts/measurements/check/shutdown.py
```

<br>

## damping

### `damping.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--motor-id` | Yes | - | Target motor ID |
| `--model` | Yes | - | Motor model (`rs02`, `rs03`) |
| `--channel` | No | Auto from ID | Target CAN channel |
| `--speeds` | Yes | - | List of target speeds (rad/s, signed) |
| `--repeats` | No | `1` | Repeats per speed |

```bash
# Example
python3 scripts/measurements/damping/damping.py \
  --motor-id 4 \
  --model rs03 \
  --channel can0 \
  --speeds 0.15 0.20 0.28 \
  --repeats 1
```

### `analyze_damping.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `csv_files` | Yes | - | CSV file(s) to analyze, or a glob |
| `--skip-s` | No | `0.1` | Startup settling time to exclude (s) |

```bash
# Example
python3 scripts/measurements/damping/analyze_damping.py \
  "scripts/measurements/damping/data/id4_rs03_*.csv"
```

<br>

## friction

### `friction.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--motor-id` | Yes | - | Target motor ID |
| `--model` | Yes | - | Motor model (`rs02`, `rs03`) |
| `--channel` | No | Auto from ID | Target CAN channel |
| `--signs` | No | `1 -1` | Test direction(s) (`1`, `-1`) |
| `--repeats` | No | `3` | Repeats per direction |

```bash
# Example
python3 scripts/measurements/friction/friction.py \
  --motor-id 4 \
  --model rs03 \
  --channel can0 \
  --signs 1 -1 \
  --repeats 3
```

### `analyze_friction.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `csv_files` | Yes | - | CSV file(s) to analyze, or a glob |

```bash
# Example
python3 scripts/measurements/friction/analyze_friction.py \
  "scripts/measurements/friction/data/id4_rs03_*.csv"
```

<br>

## joint

### `read_joint_values.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channels` | No | `can0 can1` | CAN channels to check |
| `--interface` | No | `socketcan` | python-can interface |
| `--host-id` | No | `0xFD` | Host CAN ID |
| `--timeout` | No | `0.1` | Per-motor response timeout (s); `0.02` with `--watch` |
| `--watch` | No | Off | Continuously print joint values |
| `--interval` | No | `0.1` | Refresh interval for `--watch` (s) |

```bash
# Example
python3 scripts/measurements/joint/read_joint_values.py --watch
```

### `read_joint_with_imu.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--imu-port` | No | `/dev/ttyUSB0` | N100 serial port |
| `--channels` | No | `can0 can1` | CAN channels to check |
| `--n100-dir` | No | Auto-detected | Folder containing `n100*.so` |
| `--no-imu` | No | Off | Print motor values only, without the IMU |
| `--timeout` | No | `0.02` | Per-motor response timeout (s) |

```bash
# Example
python3 scripts/measurements/joint/read_joint_with_imu.py \
  --imu-port /dev/ttyUSB0 \
  --n100-dir scripts/motor_control/motor_with_imu_test
```

### `scan_joint_limits.py`

Tracks live min/max while the joint is moved by hand; press Enter to save.

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--motor-id` | No | `1~12` | Motor IDs to track |
| `--interface` | No | `socketcan` | python-can interface |
| `--host-id` | No | `0xFD` | Host CAN ID |
| `--timeout` | No | `0.1` | Per-motor `mechPos` response wait (s) |
| `--interval` | No | `0.1` | Screen refresh interval (s) |

```bash
# Example
python3 scripts/measurements/joint/scan_joint_limits.py --motor-id 5 6
```

| Output | Description |
| --- | --- |
| `joint/joint_limit/*.csv` | Recorded min/max per motor |

<br>

## noise

### IMU

```bash
python3 scripts/measurements/noise/imu/imu_capture.py \
  --port /dev/ttyUSB0 \
  --duration 60 \
  --tag imu_only_01

python3 scripts/measurements/noise/imu/analyze_imu_noise.py \
  "scripts/measurements/noise/imu/data/imu_capture_imu_only_01_*.csv"
```

### CAN

- `type_0x02`: real-time feedback frame (pos/vel/torque/temp), returned in response to a `control()` command.
- `type_0x11`: parameter-read command, e.g. `mechPos` (`0x7019`) / `mechVel` (`0x701B`).

```bash
python3 scripts/measurements/noise/can/can_capture.py --duration 10

python3 scripts/measurements/noise/can/analyze_can_noise.py \
  "scripts/measurements/noise/can/data/can_capture_*.csv"

python3 scripts/measurements/noise/can/analyze_can_rate.py \
  "scripts/measurements/noise/can/data/can_capture_*.csv"
```
