# AI- GPS 
this project come to gave solution to the GPS with AI 



at this project i use asus tinker board with dorne 
at the drone i use ardupilot - for the telemetry esc 
--------------------------------------------------------------------------------
missions that not do - big
2)connect compass 
3) learn pytorch - control it 
4) start make the image code with the euiler or quarterion angel 
5) learn how to get the ESC data to pymavlink or mavsdk at real time
6) make code that shave all the data to csv file 
7) make the nn






the data i need to take from the drone - need to order it to with gps and not with gps - for the learn



#need to fill this line here 
data need to take from the drone:
telem from the engine - voltage/rpm/temp
voltage of the drone 
compass heading 
heading from the GPS - GLOBAL_POSITION_INT (pymavlink)
Gps - location GLOBAL_POSITION_INT /GPS_GLOBAL_ORIGIN /GLOBAL_POSITION_INT_COV-after some fileter(mabey batter)//LOCAL_POSITION_NED_COV(have here v and x at local ned)
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

3) imu - euiler andle 
4) compass
  -IST8310 Compass - 200Hz - i ask only for 10hz....
    
6) camera fps 
7) camera output (for model 1)
8) baro altitude
9) motor telem



/// some thinking - maby make skript for every instrument to check the run time of him - and take the main 
------------------------------------------------------------------------------
camera code 
at the first step lets do openc cv simple code with tracing algoritem 

https://www.youtube.com/watch?app=desktop&v=GgGro5IV-cs  - explanation about the code i need 

i need to make the run time of the camera little slower becuse the tinker board is little lazy


----------------------------------------------------------------------------------------------
main mavros node 

ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyAMA0 -p gcs_url:=udp://@127.0.0.1

i can call to the ros2 topic list and take the topic of the esc - maby its batter....



i do little change here to use ros
1) to lunch the optical flow - use this command:  ros2 launch camera_flow_pkg drone_Ai_launch.py

2) need now to make again the code for the esc telemtry - need to make somthing for myself ----- the other data i can get from the ros






