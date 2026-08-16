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
├── damping/
│   ├── damping.py
│   ├── analyze_damping.py
│   └── data/
├── friction/
│   ├── friction.py
│   ├── analyze_friction.py
│   └── data/
└── joint/
    ├── read_joint_values.py
    └── read_joint_with_imu.py
```

</br>

## armature

### `armature.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--motor-id` | Yes | - | 대상 모터 ID |
| `--model` | Yes | - | 모터 모델 (`rs02`, `rs03`) |
| `--channel` | No | ID로 자동 선택 | 대상 CAN 채널 |
| `--torques` | Yes | - | 측정 토크 목록(N·m, 부호 포함) |
| `--repeats` | No | `1` | 토크별 반복 횟수 |

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
| `csv_files` | Yes | - | 분석할 CSV 파일 또는 glob 목록 |
| `--skip-ms` | No | `15` | 시작 과도구간 제외 시간(ms) |

```bash
# Example
python3 scripts/measurements/armature/analyze_armature.py \
  "scripts/measurements/armature/data/id11_rs02_*.csv"
```

</br>

## damping

### `damping.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--motor-id` | Yes | - | 대상 모터 ID |
| `--model` | Yes | - | 모터 모델 (`rs02`, `rs03`) |
| `--channel` | No | ID로 자동 선택 | 대상 CAN 채널 |
| `--speeds` | Yes | - | 목표 속도 목록(rad/s, 부호 포함) |
| `--repeats` | No | `1` | 속도별 반복 횟수 |

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
| `csv_files` | Yes | - | 분석할 CSV 파일 또는 glob 목록 |
| `--skip-s` | No | `0.1` | 시작 정착구간 제외 시간(s) |

```bash
# Example
python3 scripts/measurements/damping/analyze_damping.py \
  "scripts/measurements/damping/data/id4_rs03_*.csv"
```

</br>

## friction

### `friction.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--motor-id` | Yes | - | 대상 모터 ID |
| `--model` | Yes | - | 모터 모델 (`rs02`, `rs03`) |
| `--channel` | No | ID로 자동 선택 | 대상 CAN 채널 |
| `--signs` | No | `1 -1` | 측정 방향 (`1`, `-1`) |
| `--repeats` | No | `3` | 방향별 반복 횟수 |

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
| `csv_files` | Yes | - | 분석할 CSV 파일 또는 glob 목록 |

```bash
# Example
python3 scripts/measurements/friction/analyze_friction.py \
  "scripts/measurements/friction/data/id4_rs03_*.csv"
```

</br>

## joint

### `read_joint_values.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channels` | No | `can0 can1` | 확인할 CAN 채널 |
| `--interface` | No | `socketcan` | python-can 인터페이스 |
| `--host-id` | No | `0xFD` | 호스트 CAN ID |
| `--timeout` | No | `0.1` | 모터별 응답 대기 시간(s); `--watch` 사용 시 `0.02` |
| `--watch` | No | Off | 관절값 연속 출력 |
| `--interval` | No | `0.1` | 연속 출력 갱신 주기(s) |

```bash
# Example
python3 scripts/measurements/joint/read_joint_values.py --watch
```

### `read_joint_with_imu.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--imu-port` | No | `/dev/ttyUSB0` | N100 시리얼 포트 |
| `--channels` | No | `can0 can1` | 확인할 CAN 채널 |
| `--n100-dir` | No | 자동 경로 | `n100*.so`가 있는 폴더 |
| `--no-imu` | No | Off | IMU 없이 모터값만 출력 |
| `--timeout` | No | `0.02` | 모터 응답 대기 시간(s) |

```bash
# Example
python3 scripts/measurements/joint/read_joint_with_imu.py \
  --imu-port /dev/ttyUSB0 \
  --n100-dir scripts/motor_control/motor_with_imu_test
```
