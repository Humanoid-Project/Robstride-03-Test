# motor_control

## Structure

```text
motor_control/
├── README.md
├── motor_run_gui.py
└── set_motor_pose.py
```

## motor_control

### `set_motor_pose.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | - | - | Move all registered motors to the configured pose |

```bash
# Example
python3 scripts/motor_control/set_motor_pose.py
```

### `motor_run_gui.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `5` | Select one motor or two motors on the same CAN channel |

```bash
# Example
python3 scripts/motor_control/motor_run_gui.py --motor-id 4
python3 scripts/motor_control/motor_run_gui.py --motor-id 5 6
```
