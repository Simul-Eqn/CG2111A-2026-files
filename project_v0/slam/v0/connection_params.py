import os

server_IP = os.getenv('SLAM_SERVER_IP', '100.76.104.32')
lidar_port = int(os.getenv('SLAM_LIDAR_PORT', '12345'))
camera_port = int(os.getenv('SLAM_CAMERA_PORT', '12346'))


LIDAR_CONNECTION_PARAMS = (server_IP, lidar_port)
CAMERA_CONNECTION_PARAMS = (server_IP, camera_port) 



