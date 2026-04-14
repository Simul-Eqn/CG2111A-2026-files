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

Or with custom IP: (this is the IP address of the camera receiver)
```bash
set SLAM_SERVER_IP=127.0.0.1
python reference_rp_lidar_api_sim.py
```

settings.py change LIDAR_OFFSET_DEG = 0


## ROBOT SETUP 

### Real Robot 

```bash
SET SLAM_SERVER_IP=[camera receiver ip address]
python rp_lidar_api.py --debug 
```

settings.py change LIDAR_OFFSET_DEG = 180


### Main terminal setup 

```bash
set MPSV0_SERVER_IP=[raspberry pi's ip address]
python slam.py
```

Note: start `rp_lidar_api.py` (or the simulator) before `slam.py`; otherwise
`slam.py` will show connection timeout errors when trying to reach the API server.

### Second terminal setup 

```bash
set SECOND_TERM_HOST=[main terminal ip address]
python second_terminal/second_terminal.py
```

If second terminal disconnects, you can reconnect by rerunning the command above.

### Camera receiver terminal setup 

```python camera_receiver_server.py``` 




## Notes 
MOVEMENT MOTOR SPEED: 130 is good 

If jumping aound: either increase MIN_VALID or decrease MAX_TRANSLATION or something 

export RP_LIDAR_SKIP_TINY_ROUNDS=0 

