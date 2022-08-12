# Distributed_Slam
**Goal**: Perform distributed slam using ROS, Gazebo and Matlab.

## Environment
This work focuses in tacckling the issue of performing distributed slamming in 
an outdoor environment  
where there may be trees, holes and other agricultural objects.  
It is supposed to have at disposal a finite number of agents which exploit 
lidar sensors to acquire
data from the environment.   
The GPS location is supposed to be unkown.  
Each agent has a communication range in which it can trasnfer data.

## System Settings
For simulating the environment the [Gazebo 11](https://classic.gazebosim.org/) 
simulator has been used
and integrated with the 
[ROS 1 package](https://classic.gazebosim.org/tutorials?tut=ros_installing).  
The ROS version employed is ROS 1 
[noetic](http://wiki.ros.org/noetic#Installation).  

Matlab version is 2021b.  
The OS in use is Ubuntu 20.04.

## System Setup
### Package Installation
First of all install Gazebo11, ROS and gazebo_ros_pkg following the links in 
the upper section.  
### Bash file changes
Add to your ```.bashrc``` file the following lines:  
``` 
export ROS_IP="<ros_ip>"  
export ROS_MASTER_URI="http://<ros_master_ip>:<ros_master_port>"
source <path_to_noetic_setup_file>/setup.sh
source <path_to_Gazebo_setup_file>/setup.sh
```  
Where:  
```ros_ip``` it is the IP address of the machine running ROS, by default it 
is ```localhost```,  
```ros_master_ip``` it is the IP address of the ROS master node, usually it is 
equal to ```ROS_IP```,  
```ros_master_port``` it is the port which the master node uses, default is 
11311.

An example of these lines is:
```  
export ROS_IP="192.168.1.97"  
export ROS_MASTER_URI="http://192.168.1.97:11311"  
source /opt/ros/noetic/setup.sh  
source /usr/share/gazebo/setup.sh  
```

After that source the new settings: ```source ~/.bashrc```

### Import custom model
Copy the ```MyLidar``` and ```MyAgent``` folder in gazebo models path:  
```cp -r MyLidar ~/.gazebo/models/ && cp -r MyAgent ~/.gazebo/models/```

## Important notice
Check that inside the path addressed by ```$GAZEBO_PLUGIN_PATH``` it exist the 
plugin ```libgazebo_ros_laser.so```  
which is the plugin used to collect the data from the Lidar sensor.

## Running the simulation
N.B.: For visualization purposes it is suggested to use a terminal emulator 
which support pane splitting 
(like [terminator](https://gnome-terminator.org/) or 
[tmux](https://github.com/tmux/tmux/wiki)).  

Having at your disposal 2 terminals:  
In the first run ```roscore``` which will initiate the ROS Master.  
In the second run ```rosrun gazebo_ros gazebo --verbose [<path_to_world.sdf>]```  
The last optional parameter tells gazebo where to find the world to load. 
In our case it will be ```$(pwd)/test_world.sdf```.

Then, the last step is tu run the matlab code provided which will automatically 
collect data from gazebo.  
To allow it to do so, it is necessary to set the environment variable at the 
start of the file, as done for  
the ```.bashrc``` file.
