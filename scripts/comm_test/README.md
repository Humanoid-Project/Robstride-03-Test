# RoboNex Communication Test

## Structure

```text
comm_test/
├── core/                  # Reusable communication components
│   ├── __init__.py
│   ├── constants.py       # Hardware mapping and protocol constants
│   ├── can_bus.py         # Parallel, read-only CAN acquisition
│   └── imu.py             # Non-blocking N100 wrapper
├── tests/
│   ├── test_core.py       # Hardware-free stage 1 tests
│   └── test_read.py       # Motor and IMU integration test
├── n100_cpp/              # N100 C++ SDK (single vendored copy)
├── policies/              # Local ONNX policies
├── n100_binding.cpp       # pybind11 bindings
├── CMakeLists.txt
├── can_up.sh
├── requirements.txt
└── README.md
```

This directory is self-contained and does not import modules from elsewhere in
the repository. Components and their direct-execution tests are added one stage
at a time.

<br>

## Setup

```bash
source .venv/bin/activate
pip install -r scripts/comm_test/requirements.txt

cmake -S scripts/comm_test -B .venv/build/comm_test \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build .venv/build/comm_test -j
ctest --test-dir .venv/build/comm_test --output-on-failure
```

Verify the binding without opening the IMU serial port:

```bash
cd scripts/comm_test
../../.venv/bin/python -c "import n100; print(n100.__doc__)"
```

<br>

## `can_up.sh`

Configures the specified SocketCAN interfaces for Robstride motors. With no
arguments, both `can0` and `can1` are configured.

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `CHANNEL...` | No | `can0 can1` | CAN interface names to configure |

```bash
# Example
./scripts/comm_test/can_up.sh
./scripts/comm_test/can_up.sh can0
```

The script requires `sudo`. Do not disconnect motor power while a motor is
enabled; run the shutdown procedure first.

<br>

## `tests/test_core.py`

Runs hardware-free tests for CAN frame construction/parsing, parallel channel
acquisition, missing-response handling, constants, and the N100 wrapper.

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `-v` | No | Off | Print each unit-test result |

```bash
# Example
.venv/bin/python scripts/comm_test/tests/test_core.py -v
```

<br>

## `tests/test_read.py`

Sends parameter-read requests only. It never enables a motor or sends a motor
control command. Position and velocity requests are issued as one batch per
channel, while `can0` and `can1` are acquired in parallel.

| Option | Required | Default | Description |
| --- | :---: | --- | --- |
| `--channels` | No | `can0 can1` | CAN channels to read |
| `--interface` | No | `socketcan` | python-can interface |
| `--host-id` | No | `0xFD` | Host CAN ID |
| `--timeout` | No | `0.02` | Whole-channel batch response timeout in seconds |
| `--imu-port` | No | `/dev/ttyUSB0` | N100 serial port |
| `--duration` | No | `5.0` | Measurement duration in seconds |
| `--print-hz` | No | `2.0` | Intermediate print frequency |
| `--no-can` | No | Off | Test only the IMU |
| `--no-imu` | No | Off | Test only the CAN motors |

```bash
# Example
.venv/bin/python scripts/comm_test/tests/test_read.py --duration 10

# Isolate each sensor path when diagnosing a failure
.venv/bin/python scripts/comm_test/tests/test_read.py --no-imu
.venv/bin/python scripts/comm_test/tests/test_read.py --no-can
```

The summary reports two different rates. `scan/s` means complete sweeps of all
six motors on one channel and must reach at least 60 Hz for the policy loop.
`parameter responses/s` counts individual position or velocity responses and
is also compared with the provisional 200 responses/s target.

No missing response is replaced with zero or a previous value. An incomplete
cycle is reported as incomplete and causes the test to fail.

<br>

## Unconfirmed policy constants

`POLICY_JOINT_ORDER`, `DEFAULT_JOINT_POSITIONS_RAD`, and `ACTION_SCALE_RAD` in
`core/constants.py` intentionally remain empty. They must be populated from the
exact RoboNex training configuration before observation assembly or motor
control is implemented.
