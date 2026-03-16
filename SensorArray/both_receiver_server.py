LIDAR_PORT = 12345 
CAMERA_PORT = 12346


import matplotlib.pyplot as plt
import cv2
import numpy as np
import math
import time
import struct
plt.ion()

# structure:
# i of length
# list of doubles: angles 
# list of doubles: distances 


import socket 
import asyncio

async def handle_camera_client(reader, writer):
    addr = writer.get_extra_info('peername')
    print(f"Connected by {addr}")


    try:

        while True:
            size_data = await reader.readexactly(8) # header 
            if size_data == b'\xff\xff\xff\xff\xff\xff\xff\xff': # -1, -1
                print("CLIENT TERMINATED CONNCECTION; close window with any key") 
                break # client quit 
            img_height, img_width = struct.unpack('!II', size_data)

            img_data = size_data.readexactly(8*img_height*img_width ) 

            img = np.frombuffer(img_data, dtype=np.uint8).reshape((img_height, img_width))

            cv2.imshow(addr, img)
            cv2.waitKey(1)

            
    except asyncio.CancelledError:
        print(f"Client handler task cancelled for {addr}")
    except Exception as e:
        print(f"Error with client {addr}: {e}")
    finally:
        print(f"Closing connection for {addr}")
        writer.close()
        await writer.wait_closed()
        cv2.waitKey(0)
        cv2.destroyWindow(addr) 


async def handle_lidar_client(reader, writer):
    addr = writer.get_extra_info('peername')
    print(f"Connected by {addr}")

    fig, ax = plt.subplots()


    try:

        while True:
            length_data = await reader.readexactly(4)
            list_length = struct.unpack('i', length_data)[0]
            if list_length == -1: # quit
                print("CLIENT TERMINATED CONNECTION") 
                break 

            angles_data = await reader.readexactly(list_length * 8)
            angles = np.array(struct.unpack(f'{list_length}d', angles_data))

            
            distances_data = await reader.readexactly(list_length * 8)
            distances = np.array(struct.unpack(f'{list_length}d', distances_data))

                
            # make sure it's numpy arrays
            xs = distances * np.cos(angles*np.pi/180)
            ys = distances * np.sin(angles*np.pi/180) 
            
            #line.set_xdata(xs)
            #line.set_ydata(ys)
            #ax.relim(); 
            xys = np.array([xs,ys]).swapaxes(0,1)
            #print(xys.dtype)
            mask = ((xys==[0.0,0.0]) | (xys<-1500) | (xys>1500)).all(axis=1)
            xys = xys[~mask]
            ax.clear()
            ax.hexbin(xys[:,1], xys[:,0], gridsize=100)
            ax.set_xlim((-1500,1500))
            ax.set_ylim((-1500,1500))
            ax.autoscale_view(); plt.draw(); plt.pause(0.001)

            
            # Sleep briefly to allow for a smoother display and to prevent overwhelming the terminal with updates. Adjust as needed for performance.
            time.sleep(0.01)

            

    except asyncio.CancelledError:
        print(f"Client handler task cancelled for {addr}")
    except Exception as e:
        print(f"Error with client {addr}: {e}")
    finally:
        print(f"Closing connection for {addr}")
        plt.close()
        writer.close()
        await writer.wait_closed()



async def main():
    camera_server = await asyncio.start_server(
        handle_camera_client, '0.0.0.0', CAMERA_PORT)

    addrs = ', '.join(str(sock.getsockname()) for sock in camera_server.sockets)
    print(f'Serving camera server on  on {addrs}')


    lidar_server = await asyncio.start_server(
        handle_lidar_client, '0.0.0.0', LIDAR_PORT)

    addrs = ', '.join(str(sock.getsockname()) for sock in lidar_server.sockets)
    print(f'Serving lidar server on  on {addrs}')





    # Run the server indefinitely
    async with camera_server, lidar_server:
        while True: 
            await asyncio.sleep(3600) 

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped manually.")




'''
import socket             

s = socket.socket()


s.bind(('', PORT))
s.listen(5)
print("Listening...")

c, addr = s.accept()
print ('Got connection from', addr)

c.send('Thank you for connecting'.encode()) 



c.close()
'''
