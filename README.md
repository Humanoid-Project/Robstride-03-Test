# Robstride-Motor-Test

## Setup
```bash
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robonex-common.git
git clone https://github.com/Humanoid-Project/imu-n100-test.git IMU_N100_Test
git clone https://github.com/Humanoid-Project/Robstride-Motor-Test.git
cd Robstride-Motor-Test
python3 -m venv .venv
source .venv/bin/activate
pip install -e ../robonex-common
pip install -r requirements.txt
```

<br>

## CAN Interface
```bash
# Example
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000

sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000
```

<br>

## Scripts
| Folder | Description | README |
| --- | --- | :---: |
| `calibration` | Motor mechanical zero and CAN ID setup | [📖](scripts/calibration/) |
| `measurements` | Read joint values and measure RS02/RS03 physical parameters | [📖](scripts/measurements/) |
| `motor_control` | Motor drive and hardware integration | [📖](scripts/motor_control/) |
