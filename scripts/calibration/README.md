# calibration

## Structure

```text
calibration/
├── README.md
├── motor_id/
│   ├── motor_id.py
│   └── set_motor_id_gui.py
└── zero_position/
    └── set_zero_position.py
```

</br>

## motor_id

### `motor_id.py`

| Command | Option | Required | Default | Description |
| --- | --- | :---: | --- | --- |
| `check` | `--channels` | No | `can0 can1` | 확인할 CAN 채널 |
| `find` | `--channel` | Yes | - | 검색할 채널 (`can0`, `can1`) |
| `find` | `--motor-id` | No | - | 특정 ID만 확인 (`0~127`) |
| `find` | `--scan-max` | No | `127` | 검색할 최대 ID (`0~127`) |
| `set` | `--channel` | Yes | - | 대상 채널 (`can0`, `can1`) |
| `set` | `--current-id` | Yes | - | 현재 ID (`0~127`) |
| `set` | `--new-id` | Yes | - | 새 ID (`1~127`) |

```bash
# Example
python3 scripts/calibration/motor_id/motor_id.py check
python3 scripts/calibration/motor_id/motor_id.py find --channel can0
python3 scripts/calibration/motor_id/motor_id.py find --channel can0 --motor-id 4
python3 scripts/calibration/motor_id/motor_id.py set --channel can0 --current-id 1 --new-id 4
```

</br>

### `set_motor_id_gui.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channel` | No | `can0` | 대상 CAN 채널 |
| `--interface` | No | `socketcan` | python-can 인터페이스 |
| `--host-id` | No | `0xFD` | 호스트 CAN ID |
| `--scan-max` | No | `127` | 검색할 최대 ID (`0~127`) |

```bash
# Example
python3 scripts/calibration/motor_id/set_motor_id_gui.py --channel can0
```

</br>

## zero_position

### `set_zero_position.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channels` | No | `can0 can1` | 설정할 CAN 채널 |
| `--interface` | No | `socketcan` | python-can 인터페이스 |
| `--host-id` | No | `0xFD` | 호스트 CAN ID |
| `--save` | No | Off | 영점 설정 후 저장 프레임(`0x16`) 전송 |
| `--zero-sta` | No | - | 전원 인가 시 범위: `0=0..2π`, `1=-π..π`; 지정 시 자동 저장 |
| `--tolerance` | No | `0.05` | 영점 확인 허용 오차(rad) |
| `--yes` | No | Off | 확인 입력 생략 |

```bash
# Example
python3 scripts/calibration/zero_position/set_zero_position.py
python3 scripts/calibration/zero_position/set_zero_position.py --channels can0 --zero-sta 1
```
