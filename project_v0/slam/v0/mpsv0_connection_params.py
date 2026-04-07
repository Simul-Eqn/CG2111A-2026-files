# this time, client is your computer 
#client_IP = '100.71.68.106'
#client_lidar_port = 12345

# server is the raspberry pi 
import os

# Backward-compatible env var handling:
# prefer MPSV0_* for this project, but accept SLAM_* aliases too.
server_IP = os.getenv('MPSV0_SERVER_IP', os.getenv('SLAM_SERVER_IP', '100.71.68.106'))
server_lidar_port = int(os.getenv('MPSV0_SERVER_PORT', os.getenv('SLAM_LIDAR_PORT', '12345')))


# command types 
CMD_CONNECT = 0
CMD_GET_SCAN_MODE = 1
CMD_START_SCAN_ROUNDS = 2
CMD_GET_NEXT_SCAN_ROUND = 3
CMD_DISCONNECT = 4

# Non-LIDAR sensor control API commands
CMD_SENSOR_SERIAL_CONNECT = 5
CMD_SENSOR_SERIAL_DISCONNECT = 6
CMD_SENSOR_ESTOP = 7
CMD_SENSOR_GET_STATUS = 8
CMD_SENSOR_GET_COLOR = 9
CMD_SENSOR_DRIVE = 10
CMD_SENSOR_SET_SPEED = 11
CMD_SENSOR_CAMERA_CAPTURE = 12
CMD_SENSOR_ARM_TEXT = 13

CMD_QUIT = -1


