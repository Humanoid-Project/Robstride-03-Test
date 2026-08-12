# cal_values

RS02/RS03 내부 물리량(armature, damping, 정지마찰) 실측 도구.
`memory/project-unmeasured-params.md` 항목 2·3·4에 대응. 관절한계(항목 1)는 발
조립 이후 `scripts/calibration/motor_calibration.py`로 별도 측정한다.

공통 전제: **모터 출력축에 부하(크랭크·링크 등)가 전혀 없는 단독 상태**에서 측정한다.
CAN 위치/속도 피드백은 매뉴얼상 "load-end"(출력축, 감속 후) 기준이라
(`docs/RS03_RSP1000_reference.md`의 `mechPos`/`mechVel` 설명 참고, RS02도 프로토콜 공통)
여기서 나온 값은 감속비 보정 없이 그대로 URDF `armature`/`damping`/`friction`에 대응한다.

`common.py`가 CAN 통신 공용 헬퍼(항목별 폴더 전부가 공유). 각 항목은 폴더로 분리:

```
cal_values/
  common.py
  run_all.py  # armature+damping+friction 전체를 한 모터에 대해 순서대로 자동 실행
  armature/   (항목 2)
    capture_torque_step.py   # 토크 1개 캡처
    analyze_armature.py      # CSV들 묶어서 J 회귀
    run_sweep.py             # capture를 토크 세트 x N회 자동 반복 + 끝나면 analyze
    data/*.csv
  damping/    (항목 3)
    capture_velocity_hold.py
    analyze_damping.py
    run_sweep.py             # capture를 속도 세트 x N회 자동 반복 + 끝나면 analyze
    data/*.csv
  friction/   (항목 4)
    capture_breakaway.py
    analyze_breakaway.py
    run_repeats.py           # capture를 방향별 N회 자동 반복 + 끝나면 analyze
    data/*.csv
```

## 전체 자동 실행 (armature + damping + friction)

```bash
cd scripts/cal_values
python3 run_all.py --motor-id 11 --model rs02              # 세 항목 전부, 항목당 5회(기본)
python3 run_all.py --motor-id 12 --model rs02 --repeats 3
python3 run_all.py --motor-id 11 --model rs02 --items friction damping   # 일부만
```
`armature/run_sweep.py` → `damping/run_sweep.py` → `friction/run_repeats.py` 순서로 그대로
서브프로세스 호출한다(로직 중복 없음). 확인은 맨 처음 한 번만, 각 항목이 끝날 때마다 그
항목의 `analyze_*.py`가 자동 실행되고, 결과는 각자 `<항목>/data/`에 저장된다. 전체 실행에
수 분 이상 걸릴 수 있다(특히 friction은 항목당 최대 15초/회).

세 폴더 다 같은 패턴: `capture_*.py` 1회 실행 = CSV 1개. `run_sweep.py`/`run_repeats.py`는
그 capture 스크립트를 **그대로 서브프로세스로 반복 호출**할 뿐이라(로직 중복 없음) 각
capture의 안전장치가 매 실행마다 원본 그대로 적용된다. 확인 프롬프트는 맨 처음 한 번만
뜨고(`--yes`로 생략 가능) 이후는 자동 진행, 끝나면 해당 `analyze_*.py`도 자동 실행된다.

## 2. Armature (회전자+감속기 관성) — ✅ RS02 측정 완료 (2026-08-09)

정지 상태에서 서로 다른 토크를 스텝으로 인가해 초반 가속도를 비교하는 방식.
마찰이 두 시행에서 거의 같다고 보고 뺄셈으로 상쇄시켜 J를 뽑는다.

```bash
cd scripts/cal_values/armature
python3 run_sweep.py --motor-id 11 --model rs02              # 기본 토크세트(0.3/0.5/1.0/1.5/2.0) 1회씩
python3 run_sweep.py --motor-id 12 --model rs02 --repeats 2  # 재현성 확인하려면 세트당 반복
```
(수동으로 하나씩 하려면 `capture_torque_step.py --motor-id .. --torque ..`을 반복 후 `analyze_armature.py data/id11_rs02_*.csv`)

- 결과: RS02 armature ≈ 0.0036~0.0038 kg·m² (ID11/ID12 교차검증, 개체차 무시 가능 확인됨) → [[hw-rs02]] 참고.
- RS03은 아직 미측정 — 같은 방법으로 진행하면 됨(`--model rs03`, `run_sweep.py`의 RS03 기본
  토크세트는 실측 전 추정값이라 필요하면 `--torques`로 직접 지정).

## 3. Damping (점성 마찰) — ✅ RS02 측정 완료 (2026-08-09)

운영제어(armature와 동일한 type 0x01 메커니즘)로 `kp=0, kd>0, v_set=목표속도`를 계속
보내 "속도만 추종하는 P제어"를 만들고, 정상상태에서 실측 속도·토크 쌍을 기록한다.
정상상태(가속도=0)에서는 `τ_실측 = b*ω_실측 + c*sign(ω_실측)`가 성립하므로, 여러 속도
(부호 섞어도 됨)로 모은 뒤 회귀하면 b(damping)를 구할 수 있다.
(처음엔 run_mode=2 속도모드 + 능동보고(type 0x18)로 시도했으나 능동보고 페이로드 형식이
매뉴얼에 없어 실패 — armature와 같은 검증된 방식으로 우회함, 2026-08-09.)

```bash
cd scripts/cal_values/damping
python3 run_sweep.py --motor-id 11 --model rs02              # 기본 속도세트(±1/2/3 rad/s) 1회씩
python3 run_sweep.py --motor-id 12 --model rs02 --repeats 2
```
(수동으로 하려면 `capture_velocity_hold.py --motor-id .. --speed ..` 반복 후 `analyze_damping.py data/id11_rs02_*.csv`)

- 결과: RS02 damping b ≈ 0(정격 대비 무시 가능, 기존 추정값 0.2의 1/150 이하), 운동마찰
  c ≈ 0.135~0.140 N·m (ID11/ID12 교차검증, 개체차 무시 가능 확인됨) → [[hw-rs02]] 참고.
- RS03·고속구간(무부하 최고속도 근처)은 실익 낮다고 판단해 보류 중.

## 4. 정지마찰 (Coulomb/static friction, breakaway) — RS02 ID11/ID12 진행 중

아주 낮은 토크(기본 0.02N·m)부터 시작해 단계마다(기본 0.3s) 토크를 조금씩(기본 0.02N·m)
올리는 계단식 탐색. 위치가 노이즈보다 훨씬 큰 폭(기본 0.03rad)만큼 벗어나는 순간을
"움직이기 시작함"으로 보고 멈춘다 — 그 직전/그 단계 토크가 breakaway의 브래킷이다.
armature/damping과 달리 **결과가 CSV 1개당 값 하나**(회귀 아님)라 `analyze_breakaway.py`는
여러 실행을 모아 정리·비교만 한다.

```bash
cd scripts/cal_values/friction
python3 run_repeats.py --motor-id 11 --model rs02 --repeats 3   # +1/-1 각각 3회, 총 6회
python3 run_repeats.py --motor-id 12 --model rs02 --repeats 3
```
(수동으로 하려면 `capture_breakaway.py --motor-id .. --sign 1|-1` 반복 후
`analyze_breakaway.py data/id11_rs02_*.csv data/id12_rs02_*.csv`)

- **개체차가 armature/damping과 달리 실제로 큼(최대 90%)** — armature 측정 중 우연히 관찰한
  정황(ID11은 0.3N·m에서 움직였는데 ID12는 안 움직임)과 교차검증됨. "1개 대표 측정 일괄
  적용" 불가, 나머지 4개체(ID1,5,6,7)도 여유 있으면 측정 권장.
- **방향 비대칭도 있음** — 두 모터 다 +방향이 −방향보다 큼(기구적 원인 추정, `analyze_breakaway.py`가 자동 비교).
- **반복측정이 특히 중요** — 2026-08-09 ID12 `-1` 방향에서 3회 중 1회가 동떨어진 값
  (-0.39 vs -0.53/-0.55)이 나온 적이 있어, 조건당 여러 번(3회 이상)이 안전하다.
- 상세 결과 → [[hw-rs02]].
