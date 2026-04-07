# ARM Terminal - Quick Start Guide

## 1. Prerequisites

- `slam.py` running and accessible
- TLS certificates copied to `project_v0/slam/v0/certs/`
- Python 3.7+
- Network connectivity between devices

## 2. Setup (One Time)

### Copy TLS Certificates

```bash
cd project_v0/slam/v0
mkdir -p certs
cp path/to/server.crt certs/
cp path/to/server.key certs/
```

### Verify Structure

```
project_v0/slam/v0/
├── arm_terminal/
│   ├── __init__.py
│   ├── arm_terminal.py
│   ├── relay.py
│   ├── net_utils.py
│   └── README.md
├── certs/
│   ├── server.crt
│   └── server.key
├── slam.py
├── arm_relay_manager.py  (NEW)
├── rp_lidar_api.py
└── ... (other slam files)
```

## 3. Start SLAM with Relay

**Terminal 1 (Robot/Pi):**

```bash
cd project_v0/slam/v0
source ../env/bin/activate
python3 slam.py
```

Watch for relay startup messages:
```
[relay_manager] Starting ARM relay...
[arm_relay] Starting relay server on port 65433...
[arm_relay] Relay server listening on port 65433.
[arm_relay] Waiting for arm_terminal.py to connect...
```

## 4. Connect Arm Terminal

**Terminal 2 (Local or Remote):**

### Local (Same Machine)
```bash
cd project_v0/slam/v0
python3 arm_terminal/arm_terminal.py
```

### Remote (Different Machine)
```bash
export ARM_RELAY_HOST=<pi-ip-or-hostname>
cd project_v0/slam/v0
python3 arm_terminal/arm_terminal.py
```

Example with IP:
```bash
export ARM_RELAY_HOST=192.168.1.100
cd project_v0/slam/v0
python3 arm_terminal/arm_terminal.py
```

You should see:
```
[arm_terminal] Connecting to relay at 192.168.1.100:65433...
[arm_terminal] TLS: enabled
[arm_terminal] Connected to relay!
```

And in slam.py terminal:
```
[arm_relay] Arm terminal connected!
```

## 5. Send Arm Commands

In the arm_terminal, type commands:

### Move Base
```
[arm_terminal] > b 90
[arm_terminal] Sent BASE=90
```

### Move Shoulder
```
[arm_terminal] > s 85
[arm_terminal] Sent SHOULDER=85
```

### Move Elbow
```
[arm_terminal] > e 90
[arm_terminal] Sent ELBOW=90
```

### Move Gripper
```
[arm_terminal] > g 80
[arm_terminal] Sent GRIPPER=80
```

### Set Speed
```
[arm_terminal] > v 25
[arm_terminal] Sent SPEED=25
```

### Home Position
```
[arm_terminal] > h
[arm_terminal] Sent ARM_HOME
```

### Stop Motors
```
[arm_terminal] > x
[arm_terminal] Sent STOP
```

### Emergency Stop
```
[arm_terminal] > e
[arm_terminal] Sent E-STOP
```

### Quit
```
[arm_terminal] > q
[arm_terminal] Quitting.
[arm_terminal] Disconnected.
```

## 6. Example Session

```bash
# Terminal 1: Start SLAM
$ cd project_v0/slam/v0
$ source ../env/bin/activate
$ python3 slam.py
...
[arm_relay] Arm terminal connected!

# Terminal 2: In another terminal/machine
$ export ARM_RELAY_HOST=192.168.1.100
$ cd project_v0/slam/v0
$ python3 arm_terminal/arm_terminal.py
[arm_terminal] Connected to relay!
[arm_terminal] > h
[arm_terminal] Sent ARM_HOME
[robot] OK
[arm_terminal] > v 20
[arm_terminal] Sent SPEED=20
[robot] OK
[arm_terminal] > b 45
[arm_terminal] Sent BASE=45
[robot] OK
[arm_terminal] > q
[arm_terminal] Quitting.
[arm_terminal] Disconnected.
```

## 7. Troubleshooting

### Connection Refused
```
[arm_terminal] Failed to connect to relay
```
**Solution**: Ensure slam.py is running and displays relay startup messages.

### TLS Certificate Error
```
[arm_terminal] TLS certificate not found at '.../server.crt'
```
**Solution**: Copy certificates to `project_v0/slam/v0/certs/`:
```bash
mkdir -p project_v0/slam/v0/certs
cp server.crt project_v0/slam/v0/certs/
cp server.key project_v0/slam/v0/certs/
```

### Arm Terminal Timeout
```
[arm_relay] No arm terminal connected within 30s
```
**Solution**: Start arm_terminal within 30 seconds of slam.py startup. Increase timeout with:
```bash
export ARM_TERM_TIMEOUT=60
cd project_v0/slam/v0
python3 slam.py
```

### Commands Not Reaching Robot
- Verify arm_terminal shows `[arm_terminal] Sent ...`
- Check slam.py shows relay connected
- Ensure API server (`rp_lidar_api.py`) is running
- Check SLAM_LIDAR_PORT is correct (default 9999)

## 8. Advanced Configuration

### Change Relay Port
```bash
export ARM_RELAY_PORT=8888
python3 slam.py
```

Then connect to that port:
```bash
export ARM_RELAY_PORT=8888
python3 arm_terminal/arm_terminal.py
```

### Disable TLS (Local Testing Only)
```bash
export TLS_ENABLED=False
python3 arm_terminal/arm_terminal.py
```

### Connect to Remote API Server
```bash
export SLAM_SERVER_IP=192.168.1.50
export SLAM_LIDAR_PORT=9999
python3 slam.py
```

## Command Reference

| Key | Purpose | Range |
|-----|---------|-------|
| `b <angle>` | Base | 0-180 |
| `s <angle>` | Shoulder | 70-120 |
| `e <angle>` | Elbow | 60-120 |
| `g <angle>` | Gripper | 70-100 |
| `v <speed>` | Speed | 1-50 |
| `h` | Home | — |
| `x` | Stop | — |
| `e` | E-Stop | — |
| `q` | Quit | — |

## File Locations

```
project_v0/slam/v0/
├── arm_terminal/                      ← ARM terminal client code
│   ├── arm_terminal.py               ← User runs this
│   ├── relay.py                      ← Relay server (runs in slam.py)
│   └── net_utils.py                  ← Network utilities
├── certs/
│   ├── server.crt                    ← Copy from main project
│   └── server.key                    ← Copy from main project
├── slam.py                           ← Entry point
├── arm_relay_manager.py              ← NEW: manages relay lifecycle
└── rp_lidar_api.py                   ← API server
```

## Next Steps

- Read full documentation: [ARM Terminal README](./arm_terminal/README.md)
- Check implementation details: [ARM Terminal Implementation](./ARM_TERMINAL_IMPLEMENTATION.md)
- See related topics: [Second Terminal](../../../SensorArray/second_terminal/README.md)
