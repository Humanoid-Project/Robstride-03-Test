# motor_control

## Structure

```text
motor_control/
├── README.md
├── motor_test/
│   ├── motor_run_gui.py
│   └── set_motor_pose.py
└── motor_with_imu_test/
    ├── CMakeLists.txt
    ├── motor_imu_run.py
    └── n100_binding.cpp
```

<br>

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

<br>

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

<br>
