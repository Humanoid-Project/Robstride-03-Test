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

<br>

## motor_id

### `motor_id.py`

| Command | Option | Required | Default | Description |
| --- | --- | :---: | --- | --- |
| `check` | `--channels` | No | `can0 can1` | CAN channels to check |
| `find` | `--channel` | Yes | - | Channel to scan (`can0`, `can1`) |
| `find` | `--motor-id` | No | - | Check only this ID (`0~127`) |
| `find` | `--scan-max` | No | `127` | Highest ID to scan (`0~127`) |
| `set` | `--channel` | Yes | - | Target channel (`can0`, `can1`) |
| `set` | `--current-id` | Yes | - | Current ID (`0~127`) |
| `set` | `--new-id` | Yes | - | New ID (`1~127`) |

```bash
# Example
python3 scripts/calibration/motor_id/motor_id.py check
python3 scripts/calibration/motor_id/motor_id.py find --channel can0
python3 scripts/calibration/motor_id/motor_id.py find --channel can0 --motor-id 4
python3 scripts/calibration/motor_id/motor_id.py set --channel can0 --current-id 1 --new-id 4
```

<br>

### `set_motor_id_gui.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channel` | No | `can0` | Target CAN channel |
| `--interface` | No | `socketcan` | python-can interface |
| `--host-id` | No | `0xFD` | Host CAN ID |
| `--scan-max` | No | `127` | Highest ID to scan (`0~127`) |

```bash
# Example
python3 scripts/calibration/motor_id/set_motor_id_gui.py --channel can0
```

<br>

## zero_position

### `set_zero_position.py`

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channels` | No | `can0 can1` | CAN channels to set |
| `--interface` | No | `socketcan` | python-can interface |
| `--host-id` | No | `0xFD` | Host CAN ID |
| `--save` | No | Off | Send save frame (`0x16`) after setting zero |
| `--zero-sta` | No | - | Power-on range: `0=0..2π`, `1=-π..π`; auto-saves when set |
| `--tolerance` | No | `0.05` | Allowed error for zero verification (rad) |
| `--yes` | No | Off | Skip the confirmation prompt |

```bash
# Example
python3 scripts/calibration/zero_position/set_zero_position.py
python3 scripts/calibration/zero_position/set_zero_position.py --channels can0 --zero-sta 1
```
