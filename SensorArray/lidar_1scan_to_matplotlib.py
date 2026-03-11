import shutil
import sys
import numpy as np
import time
from lidar.alex_lidar import lidarConnect, lidarDisconnect, lidarStatus, performSingleScan
import struct
import socket 

# ==============================================================================
# GLOBAL CONFIGURATION
# ==============================================================================
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
CONNECTION_PARAMS = ('100.71.68.106', 12345) 

'''
GRID_WIDTH = 100   
GRID_HEIGHT = 60   
MAX_RANGE_MM = 2500 
CLI_ASPECT_RATIO = 2.2

DENSITY_CHARS = " ░▒▓█" 
AXIS_CHAR_H = "─"
AXIS_CHAR_V = "│"
AXIS_CHAR_CENTER = "┼"
'''




if __name__ == "__main__":

    print("====== LiDAR Live Plot ======")
    lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    status = lidarStatus(lidar)
    mode = status['typical_scan_mode']
    print(f"Connected. Mode: {mode}\n")

    print("====== Scanning ======")
    # Reserve space for the CLI plot and hide the cursor.
    #move_up_amount = ui_prepare_frame(GRID_HEIGHT)

    lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    s = socket.socket()
    s.connect(CONNECTION_PARAMS) 

    try:
        
        if True:
            # Retrieve and print the LiDAR device information
            status = lidarStatus(lidar)
            
            #print("====== Scanning ======")
            # Get a single scan using the library. This returns the scan data.
            scan_data = performSingleScan(lidar, status['typical_scan_mode'])

            packed_data = struct.pack('i', len(scan_data[0]))
            packed_data += struct.pack(f'{len(scan_data[0])}d', *scan_data[0])
            packed_data += struct.pack(f'{len(scan_data[1])}d', *scan_data[1])

            s.sendall(packed_data)


    except KeyboardInterrupt:
        # Move to bottom of the scan area for clean exit
        #sys.stdout.write("\n" * 2)
        print("Scan stopped by user.")
    finally:
        lidarDisconnect(lidar)
        s.sendall(struct.pack('i' -1)) # -1 is exit 
        s.close() 
        #ui_show_cursor()
        sys.stdout.flush()
