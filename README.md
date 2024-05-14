# AI- GPS 
this project come to gave solution to the GPS with AI 



at this project i use pi 4B with ubuntu flash - 22.04 -> for ros iron use with dorne 
at the drone i use ardupilot - for the telemetry esc 
--------------------------------------------------------------------------------
need to add here the artical that i read
















missions that not do - big
3) learn pytorch - control it 
6) make code that shave all the data to csv file 
7) make the nn




#need to fill this line here 
data need to take from the drone:
voltage of the drone 
compass heading 
heading from the GPS - GLOBAL_POSITION_INT (pymavlink)
imu - SCALED_IMU //RAW_IMU
ATTITUDE // ALTITUDE ( #141 )





-----------------------------------------------------------------------------
mission for the close time 
1) change from scv to somthing that can andel more data
2) problem with the time
3) run time - ask if i can get the actual time



------------------------------------------------------------------------------

download python 3.9.12 -- or else 

# Update package lists
sudo apt update

# Install dependencies
sudo apt install -y build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev wget

# Download Python 3.9.12 source code
wget https://www.python.org/ftp/python/3.9.12/Python-3.9.12.tgz

# Extract the downloaded archive
tar -xf Python-3.9.12.tgz

# Navigate to the extracted directory
cd Python-3.9.12

# Configure the installation
./configure --enable-optimizations

# Compile and install Python
make #this proccese take some time - 

sudo make altinstall

# Update alternatives to set Python 3.9 as default
sudo update-alternatives --install /usr/bin/python3 python3 /usr/local/bin/python3.9 12

--------------------------------------------------------------------------------
download mavlink 
# need to download and check if the tinker board 2s get pip - if not, do this setup

1) wget https://bootstrap.pypa.io/get-pip.py
2) sudo python3 get-pip.py
3) pip3 --version --- check the version and if the pip install
if it work - do install with the virtual enveirment 
4) python3 -m venv myenv
5) source myenv/bin/activate
6) pip install --upgrade pip
7) deactivate


connect to the tinker board to the MavLink:
dont tink that i need to move do "chmod"
sudo mavproxy.py --master=/dev/ttyS0

--------------------------------------------------------------------------------
# type of model i need to test 
1) model_1 - when the camera gave only x,y data - and the motor do the ai 
2) model_2 - when the camera do a - ai atself - model 3
3) model_3 - two layer of ai o ne for the camera and one for the motor 
---------------------------------------------------------------------------------

# run time -
need all the sensor gave the data at the smae time - so if the time not work together need to do some interpulation 
  
1) GPS - all dim ---
   for holybro micro m10:
Up to 25 Hz (single GNSS),
Up to 10 Hz (4 concurrent GNSS)
  for gep-m10:
Up to 10 Hz
* the ardupilot set to publish this data at 50 hz -> need to check it 

3) imu - euiler andle 
4) compass
  -IST8310 Compass - 200Hz - i ask only for 10hz....

   
    
6) camera fps
   need to check -> make some node maby ??
   
9) baro altitude
10) motor telem
    the ardupilot pucligh it to 10 mhz -> need to check the actual speed 




/// some thinking - maby make skript for every instrument to check the run time of him - and take the main 
------------------------------------------------------------------------------
camera code 
at the first step lets do openc cv simple code with tracing algoritem 

https://www.youtube.com/watch?app=desktop&v=GgGro5IV-cs  - explanation about the code i need 

i use         # Enhancing the Lucas-Kanade parameters
need to check it and determen if had onother optical flow camera for ros2
----------------------------------------------------------------------------------------------
main mavros node 

ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyAMA0 -p gcs_url:=udp://@127.0.0.1

i can call to the ros2 topic list and take the topic of the esc - maby its batter....


important ros2 command:

ssh -X drone@   -- publish the rqt to the main computer
ros2 launch drone_project drone_launch.py
ros2 launch drone_project optical_flow_processor.launch.py 
ros2 run v4l2_camera v4l2_camera_node --- run v4l node for the camera 
ros2 run rqt_image_view rqt_image_view --- important command to see the vedio from the camera 


14.5 - this is the build for right now - witout the camera 


![rosgraph](https://github.com/naorwaiss/Ai_gps_ros2/assets/122612935/5e39db3d-c7cd-40ed-8e74-d49728d9db71)




problem:
1) the hdop is super high 
2) run process on boot - need the help of someone

3) at the dron part -
    make the leg more stronger
     add the fpv cam
     move to elers
     change to little camera and not to the night one - or mabt the night one is more impresive??? -> need to check it 
     

