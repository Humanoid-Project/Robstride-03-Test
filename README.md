# robstride-motor-test

## Setup
```bash
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robonex-common.git
git clone https://github.com/Humanoid-Project/robstride-motor-test.git
cd robstride-motor-test
source ../robonex-common/setup/setup.sh IMU_N100_Test
```

Shared across repos — see [`robonex-common/setup/SETUP.md`](https://github.com/Humanoid-Project/robonex-common/blob/main/setup/SETUP.md).

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
