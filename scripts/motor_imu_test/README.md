# motor_imu_test

IMU 가 앞으로 기울면 동작1, 뒤로 기울면 동작2 를 수행한다.

IMU C++ SDK 를 pybind11 로 감싸서 `import n100` 이 되게 했다. 리더 스레드는
C++ 에 남아 GIL 밖에서 돌기 때문에, 파이썬 제어 루프가 시리얼 I/O 때문에
멈추지 않는다.

| 경로 | 설명 |
| --- | --- |
| `n100_cpp/` | [IMU_N100_Test](https://github.com/Humanoid-Project/IMU_N100_Test) `src/cpp_n100` 의 **원본 미러**. 여기서는 수정하지 않는다 |
| `n100_binding.cpp` | pybind11 바인딩 |
| `CMakeLists.txt` | `n100_cpp` 를 붙이고 확장 모듈을 빌드 |
| `motor_imu_run.py` | 본체. CAN 코드까지 이 파일 안에 다 있고 다른 스크립트를 임포트하지 않는다 |

## 빌드

```bash
source .venv/bin/activate
pip install pybind11

cd scripts/motor_imu_test
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

`n100.cpython-310-x86_64-linux-gnu.so` 가 생성된다. 파이썬 버전/아키텍처 전용이라
환경마다 빌드해야 한다.

## 실행

```bash
ls /dev/ttyUSB* /dev/ttyACM*
sudo chmod 666 /dev/ttyUSB0

sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

cd scripts/motor_imu_test
python3 motor_imu_run.py
```

옵션은 `--imu-port`, `--channels`, `--yes` 세 개뿐이다.

**모터가 실제로 구동된다.** 지지대와 비상정지를 확보하고 실행할 것. 시작할 때
현재 자세를 읽어 그 자세를 유지하므로 실행 순간에 로봇이 튀지는 않는다.

## IMU 보정

두 단계가 들어간다.

**1. 장착 보정 (`MOUNT_ROLL_DEG = 180.0`)**

IMU 가 x축 기준 180도 뒤집혀 달려 있다. 보정 전후 비교:

| | roll | pitch | gproj |
| --- | --- | --- | --- |
| 보정 전 | −177.61 | +5.28 | (+0.09, +0.04, **+0.99**) |
| 보정 후 | **+2.39** | +5.28 | (+0.09, −0.04, **−0.99**) |

`gproj.z` 가 직립 규약인 −1 로 잡히고 roll 도 정상화된다. **pitch 는 이 보정에
영향받지 않는다.**

**2. 영점 보정 (자동)**

장착 보정을 해도 직립 자세의 pitch 가 0 이 아니다(+5.28도). 시작할 때
`ZERO_TIME`(0.5초) 동안 pitch 를 평균내어 그만큼 뺀다. 그래서 **시작 시점에
로봇이 똑바로 서 있어야 한다.** 측정된 영점은 실행 시 출력된다.

## 기울기 판정

영점 보정한 pitch 로 판정한다.

```python
PITCH_THRESHOLD_DEG = 10.0
PITCH_SIGN = +1
```

`pitch > +10도` → 동작1, `pitch < -10도` → 동작2. 중립(`|pitch| < 10도`)으로
돌아오면 다시 발동할 수 있고, 같은 쪽으로 계속 기울어 있어도 반복 발동하지 않는다.

**부호는 실기에서 확인해야 한다.** 정지 데이터만으로는 IMU 의 +x 가 로봇의 앞인지
뒤인지 알 수 없다. 로봇을 앞으로 기울였을 때 출력되는 `pitch` 가 음수로 나오면
`PITCH_SIGN = -1` 로 바꿀 것.

## 동작 정의

`motor_imu_run.py` 상단의 `MOTIONS`. `{모터 ID: 목표각 rad}` 형식이고,
`targets` 에 없는 모터는 출발 자세를 그대로 유지한다.

동작은 smoothstep 보간으로 이동하고 도착하면 그 자세를 유지한다. 이동 시간은
`MOVE_TIME`(1.5초)과 `최대 이동량 / MOVE_SPEED` 중 큰 쪽이다.

**이동은 블로킹하지 않는다.** 매 틱 한 스텝씩 진행하므로 이동 중에도 IMU 를
계속 읽는다. 그래서 **동작1 이동 중에 뒤로 기울어지면 동작을 끄지 않고, 그 순간의
위치에서 그대로 동작2 쪽으로 이어서 이동한다** (로그에 `전환` 으로 찍힘).
전환할 때 남은 이동량으로 시간을 다시 계산하므로 속도는 그대로 유지된다.

## 원본 SDK 재동기화

```bash
rsync -a --delete --exclude=build/ \
  ~/humanoid_project/IMU_N100_Test/src/cpp_n100/ \
  scripts/motor_imu_test/n100_cpp/
rm -f scripts/motor_imu_test/n100_cpp/COLCON_IGNORE

cd scripts/motor_imu_test
cmake --build build -j
./build/n100_cpp/protocol_test      # C++ 파서 검증, 하드웨어 불필요
```

현재 미러 기준점: `IMU_N100_Test` `2e8d9b6` + 2026-08-03 16:44 작업본.
