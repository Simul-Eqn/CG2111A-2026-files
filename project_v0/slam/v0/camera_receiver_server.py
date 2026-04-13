PORT = 12346

DISPLAY_SIZE = (80*5, 44*5) 

import cv2
import numpy as np
import math
import time
import struct

# structure:
# i of length
# list of doubles: angles 
# list of doubles: distances 


import socket 
import asyncio

async def handle_client(reader, writer):
    addr = writer.get_extra_info('peername')
    window_name = str(addr)
    print(f"Connected by {addr}")


    try:

        while True:
            size_data = await reader.readexactly(8) # header 
            if size_data == b'\xff\xff\xff\xff\xff\xff\xff\xff': # -1, -1
                print("CLIENT TERMINATED CONNCECTION; close window with any key") 
                break # client quit 
            img_height, img_width = struct.unpack('!II', size_data)

            img_data = await reader.readexactly(img_height * img_width)

            img = np.frombuffer(img_data, dtype=np.uint8).reshape((img_height, img_width))

            display_img = cv2.resize(img, DISPLAY_SIZE, interpolation=cv2.INTER_LINEAR)
            #display_img = cv2.rotate(display_img, cv2.ROTATE_180)

            cv2.imshow(window_name, display_img)
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
        cv2.destroyWindow(window_name) 

async def main():
    server = await asyncio.start_server(
        handle_client, '0.0.0.0', PORT)

    addrs = ', '.join(str(sock.getsockname()) for sock in server.sockets)
    print(f'Serving on {addrs}')

    # Run the server indefinitely
    async with server:
        await server.serve_forever()

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
