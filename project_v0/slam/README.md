# HOW TO SET UP 

## In raspberrypi: 

Be sure to first set connection_params.py at /project_v0/slam/v0 to be your computer's ip address 

Go to Documents/CG2111A_github/SensorArray, and 

```source env/bin/activate``` 

Then ```cd ../project_v0/slam/v0``` to go to this folder 

```python rp_lidar_api.py``` 



## On computer receiving camera: 
(currently this is same computer as your computer for other stuff)
On one terminal (on the computer receiving camera stuff)
Go to project_v0/slam/v0, and run 

```python camera_receiver_server.py``` 



## On your computer: 

use make_venv.txt's instructions to make a virtual environment and enter that virtual environment 

Go to project_v0/slam/v0, and run 

``` python slam.py``` 

This should be it. 


# Debugging note 

This one's idea is moving every computation from the raspberrypi to our computer 

So, while their sample code as slam run on raspberrypi and call lidar functions, 

I instead made rp_lidar_api.py as the raspberrypi having an API to call the functions; and then all the slam part is run on your computer. 

Meanwhile, lidar.py is the on-your-computer-side handler to call the rp_lidar_api.py APi functions 

And then after that i didn't bother renaming and added the other API functions like camera and stuff to the same files 

All the handling of button presses in UI are in ```ui.py``` 

