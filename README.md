# Running Commands (13 Aug)
Notes for KUKA project

# 1. Open Manipulator + YOLO

- First, show the system architecture diagram and explain the project.

- To run open manipulator simulation with rviz point cloud, please run 
```
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```
in new terminal.

- If you want to detect objects (currently supports coca cola cans), while open manipulator is running, run this in another terminal

```
rosservice call /yolodetect
```

# 2. Grasp Estimation

- To demonstrate grip estimation around point cloud, use the following - 
```
cd /home/silvey/catkin_ws/gpd/build
./detect_grasps ../cfg/eigen_params.cfg ../tutorials/krylon.pcd
```
- Press Q to go through the process

# 3. Show KUKA IIWA not working

- The problem with KUKA iiwa is that we can see the motion planning in rviz, but it is not simulated in Gazebo

- Try to run and motion plan with the following command
```
roslaunch iiwa_moveit moveit_planning_execution.launch sim:=true     
```


# To clarify with Kinesis Lab

- Can we have a 2 finger gripper? If so, what are the ros-packages? What topic and message should I publish to control it?
- Can we have 2 Microsoft Kinect (or any other depth camera which can give point cloud for grasp estimation). 
- For the Camera , I am planning to use an Intel D455i camera on the KUKA arm, as recommended by Prof. Tzes. May I know if the lab has it? and if it is, is there a ROS package that the lab is using?

- What is the best (or usual way) to segment and clean up the point clouds? Any suggestions?
- How to connect to KUKA iiwa? What topics and messages should I publish to control it? Does it support Inverse Kinematics?


---


# General Setup Notes

## 1. install open_manipulator and make

catkin_ws/

clone open_manipulator from github

if

```
Add the installation prefix of "open_manipulator_msgs" to CMAKE_PREFIX_PATH or set "open_manipulator_msgs_DIR" to a directory containing one of the above files.  If "open_manipulator_msgs" provides a separate development package or SDK, be sure it has been installed.
```
appears, git clone `open_manipulator_msgs`.

** Do not take literally the errors **

After `catkin_make` in catkin_ws directory, ALWAYS `source ./devel/setup.zsh`



## 2. launch, moveit, teleop

before running launch, install open_manipulator_gazebo package, from here https://github.com/ROBOTIS-GIT/open_manipulator_simulations

Then 

```
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```

in the gazebo simulation, click run

Check packages

Run moveit

```
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false use_moveit:=true
```

Teleop

```
roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
```
 Then control with keyboard
 

## 3. Implement the camera

ROS depth camera tutorial https://classic.gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros

#### !! launch rviz and save the configuration to a file. !!

Add camera to launch file

Find the launch file

```
find ./ -name "*_gazebo"     
```

go into launch/ directory, open `open_manipulator_gazebo.launch`, spawn a camera. 

to spawn a model, follow http://wiki.ros.org/simulator_gazebo/Tutorials/SpawningObjectInSimulation

spawning model using .urdf, .sdf, and .xacro use different params for spawning. 

Try with command first:
```
rosrun gazebo_ros spawn_model -model Kinect -sdf -file model.sdf  
```
Turn this command into some code for service

There is already   <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
     args="-urdf -model open_manipulator -x 0.5 -z 0.0 -J gripper_sub 0.005 -param robot_description"/>

So modify it and paste the second half of the command into "args" (provide absolute path!!!)

```
  <!-- silv edits -->
  <node name="kinect_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
     args="-model Kinect -sdf -file /home/silvey/.gazebo/models/kinect/model.sdf"/>
```

to turn a model, add -Y -P -R (for example) `-x 0.5 -z 0.0 -R -1.57 -J`

## 4. Save file from the camera and delete. Write a service

modify   `<arg name="paused" default="false"/> `   to false, so no need to click play every time

try saving the input image

```
rosrun image_view image_saver image:=/camera/color/image_raw "_filename_format:=/home/silvey/catkin_ws/src/yolo_package/src/runs/detect/image_%04d.%s"
```

then write it as service in the launch file (remember to remove "" and _ before filename_format), split into parameters

```
	<node name="image_saver" pkg="image_view" type="image_saver">
		<remap from="image" to="/camera/color/image_raw"/>
		<param name="save_all_image" value="false"/>
		<param name="filename_format" value="/home/silvey/catkin_ws/src/yolo_package/src/runs/detect/image_%04d.%s"/>
	</node>
```

To lauch our configured rvis as service, use the saved configuration file from he last step (absolute path!)

```
  <node type="rviz" name="rviz" pkg="rviz" args="-d /home/silvey/Documents/ros_notes/whatever.rviz" />
```

To save image once, call the image saver service

```
rosservice call /image_saver/save   
```
 
To list all rosservice and all functions inside, run

```
rosservice list  
```

## 5. setup yolo

use venv

try first with detect.py

```
python3 detect.py --source people.jpg --save-txt --nosave --exist-ok
```

create a server.py file for the yolo part. inside there should be commands for activating the environment, etc.


cp /home/silvey/catkin_ws_old/src/open_manipulator_simulations/open_manipulator_gazebo/rviz/depth_camera.rviz /home/silvey/catkin_ws/src/open_manipulator_simulations/open_manipulator_gazebo/rviz/






## Setting up Grip Pose Library

Install Point Cloud Library (https://pointclouds.org/downloads/#linux)

Install Eigen (https://tutorialforlinux.com/2020/09/25/step-by-step-eigen-c-ubuntu-20-04-installation-guide/)

Set up eigen path in standard library
(https://stackoverflow.com/questions/23284473/fatal-error-eigen-dense-no-such-file-or-directory)


While installing OpenCV Dependencies, you will need to resolve libjasper manually using
```
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main" 
sudo apt update 
sudo apt install libjasper1 libjasper-dev
```

Install OpenCV 3.4.0 (https://web.archive.org/web/20180212100129/https://www.python36.com/how-to-install-opencv340-on-ubuntu1604/)


Then install GPD C++ library https://github.com/atenpas/gpg


Use
```
make install
```
to install as a library 


Then install the ros wrapper from https://github.com/atenpas/gpd_ros


Edit the weights file in /home/silvey/catkin_ws/gpd/cfg/ros_eigen_params.cfg to 
```
weights_file = /home/silvey/catkin_ws/gpd/models/lenet/15channels/params/
```


More info at : https://www2.ccs.neu.edu/research/helpinghands/code/gpd/

-----
How to segment the table or plane from point cloud
(https://towardsdatascience.com/how-to-automate-3d-point-cloud-segmentation-and-clustering-with-python-343c9039e4f5)
