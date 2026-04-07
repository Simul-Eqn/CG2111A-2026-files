# ARM Terminal - Robot Arm Control

The ARM terminal allows remote control of the robot arm over TCP/TLS via a relay server running in `slam.py`.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│ arm_terminal.py                                                 │
│ (Interactive arm control client)                                │
└────────────┬────────────────────────────────────────────────────┘
             │ TCP/TLS (port 65433)
             │
┌────────────▼────────────────────────────────────────────────────┐
│ slam.py                                                         │
│ ┌──────────────────────────────────────────────────────────┐   │
│ │ ARM Relay Server (relay.py, arm_relay_manager.py)        │   │
│ └──────────────────┬───────────────────────────────────────┘   │
└────────────────────┼──────────────────────────────────────────┘
                     │ TCP/Binary (port 9999 - SLAM_LIDAR_PORT)
                     │
            ┌────────▼────────────┐
            │ rp_lidar_api.py    │
            │ (API Server)       │
            └────────┬────────────┘
                     │ Serial
                     │
            ┌────────▼────────────┐
            │ Arduino/Robot      │
            │ (Arm Controller)   │
            └────────────────────┘
```

## Files

### New in `project_v0/slam/v0/arm_terminal/`

- **`arm_terminal.py`** - Interactive terminal client for arm control
- **`relay.py`** - Relay server module that listens for connections
- **`net_utils.py`** - TCP/TLS network utilities

### Modified in `project_v0/slam/v0/`

- **`arm_relay_manager.py`** (new) - Manages relay lifecycle and API server connection
- **`ui_custom.py`** or main SLAM entry point (to be integrated)

## Setup

### 1. Copy TLS Certificates

The relay uses TLS for secure connections. Copy the certificate and key to the certs directory:

```bash
# From the project_v0/slam/v0 directory
mkdir -p certs
cp path/to/server.crt certs/
cp path/to/server.key certs/
```

### 2. Environment Variables

Optional environment variables to customize behavior:

```bash
# Arm relay configuration
export ARM_RELAY_PORT=65433          # Port arm_terminal connects to (default: 65433)
export ARM_TERM_TIMEOUT=30           # Seconds to wait for arm terminal (default: 30)
export TLS_ENABLED=True              # Enable TLS (default: True)

# API server configuration (from SLAM perspective)
export SLAM_SERVER_IP=localhost      # Robot API server IP (default: localhost)
export SLAM_LIDAR_PORT=9999          # Robot API server port (default: 9999)

# Arm terminal client configuration
export ARM_RELAY_HOST=localhost      # Where to find relay (default: localhost)
export ARM_RELAY_PORT=65433          # Relay port (default: 65433)
```

## Usage

### Master Terminal (PI or Robot)

Run the SLAM system with relay enabled:

```bash
cd project_v0/slam/v0
source ../env/bin/activate
python3 slam.py
```

The relay starts automatically on port 65433.

### Arm Terminal (Remote or Local)

Open a separate terminal and run:

```bash
cd project_v0/slam/v0
python3 arm_terminal/arm_terminal.py
```

Or from a remote machine:

```bash
export ARM_RELAY_HOST=<robot-ip>
cd project_v0/slam/v0
python3 arm_terminal/arm_terminal.py
```

### Commands

Once connected, use these commands to control the arm:

| Command | Description | Range |
|---------|-------------|-------|
| `b <angle>` | Base rotation | 0 - 180° |
| `s <angle>` | Shoulder | 70 - 120° |
| `e <angle>` | Elbow | 60 - 120° |
| `g <angle>` | Gripper | 70 - 100° |
| `v <speed>` | Arm speed | 1 - 50 |
| `h` | Home position | - |
| `x` | Stop motors | - |
| `e` | Emergency stop | - |
| `q` | Quit | - |

### Example Session

```
[arm_terminal] Connecting to relay at localhost:65433...
[arm_terminal] TLS: enabled
[arm_terminal] Connected to relay!
[arm_terminal] Commands: b/s/e/g <angle>, h, v <speed>, x, e, q
[arm_terminal] Type 'q' to quit, or press Ctrl+C

[arm_terminal] > h
[arm_terminal] Sent ARM_HOME

[arm_terminal] > v 30
[arm_terminal] Sent SPEED=30

[arm_terminal] > b 90
[arm_terminal] Sent BASE=90

[robot] OK

[arm_terminal] > q
[arm_terminal] Quitting.
[arm_terminal] Disconnected.
```

## Network Configuration

### Local Network (Raspberry Pi + Laptop)

```bash
# On Raspberry Pi (running slam.py)
SLAM_SERVER_IP=localhost SLAM_LIDAR_PORT=9999 python3 slam.py

# On Laptop (running arm_terminal.py)
ARM_RELAY_HOST=<pi-ip> ARM_RELAY_PORT=65433 python3 arm_terminal/arm_terminal.py
```

### Remote Network (Tailscale VPN)

```bash
# Get your Tailscale IP
tailscale ip -4

# On Pi - use Tailscale IP for API server (if running on same Pi)
# Actually, use localhost for API (it's on same machine)
python3 slam.py

# On Remote - connect to Pi's Tailscale IP
ARM_RELAY_HOST=<tailscale-pi-ip> python3 arm_terminal/arm_terminal.py
```

## Troubleshooting

### Connection Refused
- **Problem**:"Failed to connect to relay"
- **Solution**: Ensure `slam.py` is running and the relay started successfully
- **Check**: Look for `[arm_relay] Relay started.` in slam.py output

### TLS Certificate Not Found
- **Problem**: "TLS certificate not found"
- **Solution**: Copy server.crt and server.key to the `certs/` directory
- **Path**: `project_v0/slam/v0/certs/server.crt` and `project_v0/slam/v0/certs/server.key`

### API Server Connection Failed
- **Problem**: Commands are sent but don't reach the robot
- **Solution**: Ensure the API server (rp_lidar_api.py) is running
- **Check**: Verify `SLAM_SERVER_IP` and `SLAM_LIDAR_PORT` are correct

### Arm Terminal Timeout
- **Problem**: "No arm terminal connected within 30s"
- **Solution**: Start the arm terminal within 30 seconds of starting slam.py
- **Adjust**: Set `ARM_TERM_TIMEOUT` environment variable to increase timeout

## Implementation Details

### How Relay Works

1. **Startup**: `slam.py` initializes and calls `arm_relay_manager.start_relay()`
2. **Server Listening**: ARM relay listens on port 65433 for connections
3. **Client Connect**: `arm_terminal.py` connects to the relay
4. **Command Flow**:
   - User types command in arm_terminal
   - Command packed into TPacket frame
   - Frame sent to relay over TCP/TLS
   - Relay forwards to API server (rp_lidar_api.py)
   - API server sends to Arduino over serial
   - Arduino executes arm command
   - Response comes back through API → relay → arm_terminal
5. **Display**: Arm terminal displays responses and status

### Thread Safety

- All API server communication is protected by `_api_lock`
- Relay listens in a separate daemon thread
- Main UI continues to run independently

### Support for Multiple Terminals

Currently, only one arm terminal can be connected at a time. The relay will:
- Accept the first connection
- Queue commands from that connection
- Drop subsequent connection attempts
- Reconnect if the current terminal disconnects

Future enhancement could support multiple simultaneous connections.

## Related Files

- `SensorArray/second_terminal/` - Similar relay pattern for sensor commands
- `rp_lidar_api.py` - API server that receives commands
- `slam.py` - Main entry point
- `ui_custom.py` - Matplotlib SLAM UI (where relay is integrated)

## See Also

- [SLAM Documentation](README.md)
- [API Server Documentation](../rp_lidar_api.py)
- [TLS Documentation](../../../RobotIntegration/docs/tls-setup.md)
