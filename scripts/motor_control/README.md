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
| `--channels` | No | `can0 can1` | 사용할 CAN 채널 |
| `--interface` | No | `socketcan` | python-can 인터페이스 |
| `--host-id` | No | `0xFD` | 호스트 CAN ID |
| `--yes` | No | Off | 확인 입력 생략 |

```bash
# Example
python3 scripts/motor_control/motor_test/set_motor_pose.py --channels can0
```

### `motor_run_gui.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channel` | No | `can0` | 대상 CAN 채널 |
| `--interface` | No | `socketcan` | python-can 인터페이스 |
| `--motor-id` | No | `5` | 모터 ID 1개 또는 2개 |
| `--model` | No | `rs03` | 공통 모델 1개 또는 모터별 모델 (`rs02`, `rs03`) |
| `--host-id` | No | `0xFD` | 호스트 CAN ID |

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
| `--imu-port` | No | `/dev/ttyUSB0` | N100 시리얼 포트 |
| `--channels` | No | `can0 can1` | 사용할 CAN 채널 |
| `--yes` | No | Off | 확인 입력 생략 |

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
| `--hardware` | No | Off | 실물 CAN 추종 활성화 |
| `--motor-id` | No | `1~12` | 추종할 모터 ID 목록 |
| `--model` | No | `scene_fixed.xml` | fixed-base MJCF 경로 |
| `--interface` | No | `socketcan` | python-can 인터페이스 |
| `--host-id` | No | `0xFD` | 호스트 CAN ID |
| `--rate` | No | `100` | 명령 주기(Hz) |
| `--max-speed` | No | `0.10` | 최대 목표 속도(rad/s) |
| `--max-accel` | No | `0.25` | 최대 목표 가속도(rad/s²) |
| `--kp` | No | `40.0` | 위치 게인 |
| `--kd` | No | `2.0` | 감쇠 게인 |
| `--zero-tolerance-deg` | No | `3.0` | zero 도달 허용 오차(deg) |
| `--limit-margin-deg` | No | `3.0` | 관절 한계 안쪽 여유(deg) |
| `--feedback-timeout` | No | `0.30` | 피드백 제한 시간(s) |
| `--overspeed` | No | `0.50` | 과속 정지 기준(rad/s) |
| `--max-error-deg` | No | `10.0` | 추종 오차 정지 기준(deg) |
| `--max-temp` | No | `70.0` | 과열 정지 기준(°C) |
| `--brake-time` | No | `0.20` | 종료 전 능동 감쇠 시간(s) |
| `--yes` | No | Off | 실물 시작 확인 입력 생략 |
| `--headless` | No | Off | 뷰어 없이 실행 |
| `--duration` | No | - | 지정 시간 후 종료(s) |

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
| `--imu-port` | No | `/dev/ttyUSB0` | N100 시리얼 포트 |
| `--channels` | No | `can0 can1` | 확인할 CAN 채널 |
| `--interface` | No | `socketcan` | python-can 인터페이스 |
| `--timeout` | No | `0.02` | 모터별 파라미터 응답 대기 시간(s) |
| `--rate` | No | `10.0` | 출력 주기(Hz) |

```bash
# Example
python3 scripts/motor_control/policy_test/print_policy_values.py \
  --imu-port /dev/ttyUSB0
```

### `print_policy_action.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--policy` | Yes | - | closed-loop `policy.onnx` 경로 |
| `--imu-port` | No | `/dev/ttyUSB0` | N100 시리얼 포트 |
| `--channels` | No | `can0 can1` | 확인할 CAN 채널 |
| `--interface` | No | `socketcan` | python-can 인터페이스 |
| `--timeout` | No | `0.02` | 모터별 파라미터 응답 대기 시간(s) |
| `--rate` | No | `10.0` | 출력 주기(Hz) |

```bash
# Example
python3 scripts/motor_control/policy_test/print_policy_action.py \
  --policy /path/to/policy.onnx \
  --imu-port /dev/ttyUSB0
```
