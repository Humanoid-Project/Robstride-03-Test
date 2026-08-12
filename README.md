# Robstride-Motor-Test

## Setup

```bash
git clone https://github.com/Humanoid-Project/Robstride-Motor-Test.git
cd Robstride-Motor-Test
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`tkinter`는 Python GUI의 시스템 패키지다. Ubuntu에서 GUI 실행 시 모듈이
없다면 `sudo apt install python3-tk`로 설치한다.

## CAN Interface

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000

sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000
```

## Motor Test

### 1. motor_run

```bash
cd scripts/motor_run/

python3 one_motor_run_gui.py
python3 two_motor_run_gui.py
```

`motor_pose_run.py`와 IMU 연동 구동 스크립트는 위치 제어를 시작할 때 반드시
type `0x02` 실시간 피드백을 기준으로 삼는다. `mechPos(0x7019)`는 멀티턴 표시
용도이며 `kp>0` 위치 목표로 사용하지 않는다.

### 2. cal_values

RS02/RS03의 armature, damping, friction 측정 도구다. ID를 생략 없이 실제 로봇
매핑에 따라 ID 1~6은 `can0`, ID 7~12는 `can1`으로 자동 선택한다. 벤치 배선이
다르면 반드시 `--channel`로 명시한다.

```bash
cd scripts/cal_values
python3 run_all.py --motor-id 2 --model rs03
```

모터 출력축에 링크나 크랭크가 없는 단독 무부하 상태에서만 실행한다.

### 3. motor_id

```bash
cd scripts/motor_id/

python3 find_motor_id.py --scan
python3 find_motor_id.py --check-id {ID}
python3 set_motor_id.py --current-id {ID} --new-id {NEW_ID}
python3 set_motor_id_gui.py
```

### 4. calibration

```bash
cd scripts/calibration/

python3 motor_calibration.py --motor-id {ID}
```

### 5. zero_position

```bash
cd scripts/zero_position/

python3 set_zero_position.py
python3 set_zero_position.py --channels can0
python3 set_zero_position.py --save
python3 set_zero_position.py --zero-sta 1
```
