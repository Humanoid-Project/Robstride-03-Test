# Robstride-Motor-Test

## Setup
```bash
git clone https://github.com/Humanoid-Project/Robstride-Motor-Test.git
cd Robstride-Motor-Test
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## CAN Interface
```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000

sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000
```

## Motor Test
| Folder | Description | README |
| --- | --- | :---: |
| `calibration` | 모터의 기계적 zero와 CAN ID 설정 | [📖](scripts/calibration/) |
| `measurements` | 모터 Joint값 확인 및 RS02/RS03 물리량 측정 | [📖](scripts/measurements/) |
| `motor_control` | 모터 구동 및 실물 연동 | [📖](scripts/motor_control/) |