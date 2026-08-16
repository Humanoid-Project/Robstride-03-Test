# motor_control

## Structure

```text
motor_control/
├── README.md
├── motor_test/
│   ├── motor_run_gui.py
│   └── set_motor_pose.py
├── motor_with_imu_test/
│   ├── CMakeLists.txt
│   ├── motor_imu_run.py
│   ├── n100_binding.cpp
│   └── n100_cpp/
├── mujoco_to_real/
│   └── mujoco_hardware_twin.py
└── policy_test/
    ├── CMakeLists.txt
    ├── print_policy_action.py
    ├── print_policy_values.py
    ├── n100_binding.cpp
    └── n100_cpp/
```

</br>

## motor_test

### `set_motor_pose.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channels` | No | `can0 can1` | CAN channels to use |
| `--interface` | No | `socketcan` | python-can interface |
| `--host-id` | No | `0xFD` | Host CAN ID |
| `--yes` | No | Off | Skip the confirmation prompt |

```bash
# Example
python3 scripts/motor_control/motor_test/set_motor_pose.py --channels can0
```

### `motor_run_gui.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channel` | No | `can0` | Target CAN channel |
| `--interface` | No | `socketcan` | python-can interface |
| `--motor-id` | No | `5` | One or two motor IDs |
| `--model` | No | `rs03` | One shared model, or one model per motor (`rs02`, `rs03`) |
| `--host-id` | No | `0xFD` | Host CAN ID |

```bash
# Example
python3 scripts/motor_control/motor_test/motor_run_gui.py \
  --channel can0 \
  --motor-id 4 \
  --model rs03

python3 scripts/motor_control/motor_test/motor_run_gui.py \
  --channel can0 \
  --motor-id 5 6 \
  --model rs02
```

</br>

## motor_with_imu_test

### IMU SDK

```bash
# Example
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/IMU_N100_Test.git

cmake -S IMU_N100_Test/src/cpp_n100 \
  -B IMU_N100_Test/src/cpp_n100/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build IMU_N100_Test/src/cpp_n100/build -j
```

### Python Binding

```bash
# Example
cd ~/humanoid_project/Robstride-Motor-Test
source .venv/bin/activate

cmake -S scripts/motor_control/motor_with_imu_test \
  -B scripts/motor_control/motor_with_imu_test/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(pwd)/.venv/bin/python"
cmake --build scripts/motor_control/motor_with_imu_test/build -j
```

### `motor_imu_run.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--imu-port` | No | `/dev/ttyUSB0` | N100 serial port |
| `--channels` | No | `can0 can1` | CAN channels to use |
| `--yes` | No | Off | Skip the confirmation prompt |

```bash
# Example
python3 scripts/motor_control/motor_with_imu_test/motor_imu_run.py \
  --imu-port /dev/ttyUSB0 \
  --channels can0 can1
```

</br>

## mujoco_to_real

### `mujoco_hardware_twin.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--hardware` | No | Off | Enable real CAN following |
| `--motor-id` | No | `1~12` | Motor IDs to follow |
| `--model` | No | `scene_fixed.xml` | Path to the fixed-base MJCF |
| `--interface` | No | `socketcan` | python-can interface |
| `--host-id` | No | `0xFD` | Host CAN ID |
| `--rate` | No | `100` | Command rate (Hz) |
| `--max-speed` | No | `0.10` | Max target speed (rad/s) |
| `--max-accel` | No | `0.25` | Max target acceleration (rad/s²) |
| `--kp` | No | `40.0` | Position gain |
| `--kd` | No | `2.0` | Damping gain |
| `--zero-tolerance-deg` | No | `3.0` | Allowed error to declare zero reached (deg) |
| `--limit-margin-deg` | No | `3.0` | Margin inside the joint limit (deg) |
| `--feedback-timeout` | No | `0.30` | Feedback freshness limit (s) |
| `--overspeed` | No | `0.50` | Overspeed stop threshold (rad/s) |
| `--max-error-deg` | No | `10.0` | Tracking-error stop threshold (deg) |
| `--max-temp` | No | `70.0` | Overtemperature stop threshold (°C) |
| `--brake-time` | No | `0.20` | Active braking time before shutdown (s) |
| `--yes` | No | Off | Skip the hardware-start confirmation prompt |
| `--headless` | No | Off | Run without the viewer |
| `--duration` | No | - | Auto-stop after this many seconds |

```bash
# Example
python3 scripts/motor_control/mujoco_to_real/mujoco_hardware_twin.py \
  --headless \
  --duration 5 \
  --motor-id 4

python3 scripts/motor_control/mujoco_to_real/mujoco_hardware_twin.py \
  --hardware \
  --motor-id 4
```

</br>

## policy_test

### Python Binding

```bash
# Example
cd ~/humanoid_project/Robstride-Motor-Test
source .venv/bin/activate

cmake -S scripts/motor_control/policy_test \
  -B scripts/motor_control/policy_test/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(pwd)/.venv/bin/python"
cmake --build scripts/motor_control/policy_test/build -j
```

### `print_policy_values.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--imu-port` | No | `/dev/ttyUSB0` | N100 serial port |
| `--channels` | No | `can0 can1` | CAN channels to check |
| `--interface` | No | `socketcan` | python-can interface |
| `--timeout` | No | `0.02` | Per-motor parameter response timeout (s) |
| `--rate` | No | `10.0` | Print rate (Hz) |

```bash
# Example
python3 scripts/motor_control/policy_test/print_policy_values.py \
  --imu-port /dev/ttyUSB0
```

### `print_policy_action.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--policy` | Yes | - | Path to the closed-loop `policy.onnx` |
| `--imu-port` | No | `/dev/ttyUSB0` | N100 serial port |
| `--channels` | No | `can0 can1` | CAN channels to check |
| `--interface` | No | `socketcan` | python-can interface |
| `--timeout` | No | `0.02` | Per-motor parameter response timeout (s) |
| `--rate` | No | `10.0` | Print rate (Hz) |

```bash
# Example
python3 scripts/motor_control/policy_test/print_policy_action.py \
  --policy /path/to/policy.onnx \
  --imu-port /dev/ttyUSB0
```
