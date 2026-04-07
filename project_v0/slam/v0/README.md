## To change map size 
- Increase MAP_SIZE_METERS for more real-world area 
- Increase MAP_SIZE_PIXELS for more resolution 

## Configuration

All default values are defined in `settings.py`:
Edit defaults directly in `project_v0/slam/v0/settings.py`.
- `MAP_SIZE_PIXELS`, `MAP_SIZE_METERS` - Map configuration
- `LIDAR_*` - LIDAR hardware settings
- `MPSV0_SERVER_IP` / `MPSV0_SERVER_PORT` - Default API server target for `lidar.py`
- `SECOND_TERM_PORT` - Port for second terminal relay server (default: `65432`)
- `SECOND_TERM_TIMEOUT` - Initial wait timeout for second terminal (default: `30` seconds)

Override defaults using environment variables:
```bash
export MPSV0_SERVER_IP=192.168.1.100       # LIDAR client target server IP
export MPSV0_SERVER_PORT=12345             # LIDAR client target server port
export SLAM_SERVER_IP=192.168.1.100        # Alias for MPSV0_SERVER_IP
export SLAM_LIDAR_PORT=12345               # Alias for MPSV0_SERVER_PORT
export SECOND_TERM_PORT=65432              # second_terminal relay port
export SECOND_TERM_HOST=192.168.1.100      # Where second_terminal connects
```

## Debugging 
- flip LIDAR_ANGLE_SIGN or change LIDAR_OFFSET_DEG in settings.py 
- flip x/y axis of map view 

## If using simulator: 

### Environment Simulator 

```python reference_rp_lidar_api_sim.py``` 

Or with custom IP:
```bash
set SLAM_SERVER_IP=127.0.0.1
python reference_rp_lidar_api_sim.py
```

### Main terminal setup 

```python slam.py``` 

Or with custom API server:
```bash
set MPSV0_SERVER_IP=127.0.0.1
python slam.py
```

Note: start `rp_lidar_api.py` (or the simulator) before `slam.py`; otherwise
`slam.py` will show connection timeout errors when trying to reach the API server.

### Second terminal setup 

```python second_terminal/second_terminal.py``` 

Or with custom relay host:
```bash
set SECOND_TERM_HOST=127.0.0.1
python second_terminal/second_terminal.py
```

If second terminal disconnects, you can reconnect by rerunning the command above.

### Camera receive terminal setup 

```python camera_receiver_server.py``` 

