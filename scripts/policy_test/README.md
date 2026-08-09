# policy_test

IMU + CAN 모터 실측값으로 `robonex_balancing` 균형 정책의 관측/출력을 확인하는
읽기 전용 도구 모음. **모터에 명령을 보내지 않는다** — 값을 출력만 한다.

| 경로 | 설명 |
| --- | --- |
| `n100_cpp/` | [IMU_N100_Test](https://github.com/Humanoid-Project/IMU_N100_Test) `src/cpp_n100` 의 **원본 미러**. 여기서는 수정하지 않는다 |
| `n100_binding.cpp` | pybind11 바인딩 |
| `CMakeLists.txt` | `n100_cpp` 를 붙이고 확장 모듈을 빌드 |
| `print_policy_values.py` | 관절 위치/속도, IMU 각속도/중력벡터, (placeholder) 이전 action 을 출력 |
| `print_policy_action.py` | 위 값으로 관측 벡터를 만들어 학습된 정책(ONNX)에 넣고 추정 action 을 출력 |

(원래 `scripts/read_values/`에서 시작했으나, IMU SDK가 붙으면서 성격이 달라져
이 폴더로 옮겼다. `read_values/`에는 순수 CAN 전용 스크립트만 남았다.)

## 빌드

```bash
source .venv/bin/activate
pip install pybind11 numpy onnxruntime

cd scripts/policy_test
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

`n100.cpython-310-x86_64-linux-gnu.so` 가 생성된다. 파이썬 버전/아키텍처
전용이라 환경마다 빌드해야 한다.

## print_policy_values.py

관절 위치/속도(12+12), IMU 각속도/중력벡터(3+3), 이전 action(12, 항상 0
placeholder — 이 스크립트는 정책을 실행하지 않는다)를 출력한다.

```bash
cd scripts/policy_test
python3 print_policy_values.py
```

## print_policy_action.py

`robonex_balancing/logs/rsl_rl/robonex_balancing/*/exported/policy.onnx`
(기본값: 가장 최근 런)를 로드해서, 실측 센서로 만든 관측값을 넣고 정책이
추정하는 action 을 출력한다.

```bash
cd scripts/policy_test
python3 print_policy_action.py
python3 print_policy_action.py --policy /path/to/other/policy.onnx
```

### 관측 벡터 구성

학습 로그의 `params/env.yaml` (`observations.policy`)을 그대로 따른다:

```
joint_pos_rel(12) + joint_vel_rel(12) + imu_ang_vel(3) + projected_gravity(3)
+ last_action(12)  =  42
```

관절 순서(`asset_cfg.joint_names`): `l_hip_yaw, l_hip_pitch, l_hip_roll, l_knee,
l_ankle_roll, l_ankle_pitch, r_hip_yaw, r_hip_pitch, r_hip_roll, r_knee,
r_ankle_roll, r_ankle_pitch`. `default_joint_pos/vel` 이 전부 0 이라 `*_rel` 은
그냥 현재 값과 같다.

`last_action` 은 실제로 갱신된다 — 매 틱 정책이 낸 raw action 을 다음 틱
관측의 `last_action` 으로 그대로 먹인다 (실제 배포와 같은 피드백).

### ⚠ 발목 4개는 근사치다

`l/r_ankle_roll_joint`, `l/r_ankle_pitch_joint` 는 시뮬레이터에서는 출력축
값이지만, 실물은 그 두 값을 차동(differential)으로 만들어내는 모터 2개
(upper/lower, CAN ID 5·6·11·12)뿐이고 둘 다 순수한 roll도 pitch도 아니다.
정확한 변환에는 크랭크 기구학이 필요한데, **이 변환은 아직 유도되지 않았다**
(`project-unmeasured-params` 메모: 발목 크랭크 각도범위조차 아직 URDF
플레이스홀더). `robonex_description/scripts/robonex_serial.py` 에도 이
변환 코드는 없다.

그래서 이 스크립트는:
- 관측의 발목 roll/pitch 슬롯 4개를 실제 모터값으로 채우지 않고 **0.0** 으로 둔다.
- 발목 모터(upper/lower) 원시값은 참고용으로 따로 표시하되 관측에 넣지 않는다.
- 정책이 내놓는 발목 action 은 "가상 출력축 목표각"으로만 표시한다. **그대로
  CAN 프레임으로 만들어 모터에 보내면 안 된다** — 변환 방법이 없다.

이 4개를 제외한 8개 관절(hip_yaw/pitch/roll ×2, knee ×2)은 CAN 모터 하나가
그 관절 그대로라 (`hw-canbus.md` 매핑표) 실제 값을 그대로 쓴다.

### 각속도: raw 를 쓴다

시뮬레이터의 `imu_ang_vel` 은 물리엔진이 계산한 실제 각속도 + 가우시안 잡음이라,
AHRS 융합값(내부 필터의 위상 지연 있음)보다 `angular_velocity_raw`(IMU 프레임
원시 자이로)가 더 가깝다. `print_policy_values.py` 는 반대로 fused 를 기본으로
쓰는데, 그건 정책 없이 사람이 보기 좋은 값을 보여주는 용도라 이유가 다르다.

### 검증

실기(N100 IMU + CAN 모터 12개)로 확인: IMU raw 각속도/중력벡터 정상 수신,
CAN 두 채널 모두 130~147 Hz로 안정적으로 읽힘, ONNX 추론 0.2~0.5ms, 여러 틱에
걸쳐 값이 튀지 않고 수렴. `right_ankle_lower`(모터 12) 가 +5.6 rad 근처의
비정상 값을 계속 리턴하는 것도 확인됐는데, 어차피 관측에는 안 쓰이지만 모터
자체가 영점이 안 맞았거나 응답이 이상할 수 있으니 `set_zero_position.py` 로
확인해볼 것.

## 원본 SDK 재동기화

```bash
rsync -a --delete --exclude=build/ \
  ~/humanoid_project/IMU_N100_Test/src/cpp_n100/ \
  scripts/policy_test/n100_cpp/
rm -f scripts/policy_test/n100_cpp/COLCON_IGNORE

cd scripts/policy_test
cmake --build build -j
./build/n100_cpp/protocol_test      # C++ 파서 검증, 하드웨어 불필요
```

현재 미러 기준점: `IMU_N100_Test` `2e8d9b6` + 2026-08-03 16:44 작업본.

## 정책 재동기화

`robonex_balancing` 에서 새로 학습하면 `--policy` 로 새 경로를 지정하거나,
`logs/rsl_rl/robonex_balancing/` 아래 가장 최근 런이 자동으로 잡힌다. 관절
순서/스케일/기본자세가 학습 설정에서 바뀌면 이 스크립트 상단의 `JOINT_ORDER`,
`ACTION_SCALE`, `DIRECT_JOINT_TO_MOTOR` 도 그 `env.yaml` 을 다시 확인해서
맞춰야 한다 — 자동으로 따라가지 않는다.
