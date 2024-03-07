—-----------------------------------------------------------------------------------------------------------------------
README - [Packages - carone, python_scripts]
—----------------------------------------------------------------------------------------------------------------------
The packages, carone, python_scripts are ros2 galactic packages which contains a 4 wheeled Vehicle designed in SolidWorks and exported as URDF FIle.

URDF file has the all the joint and link configurations, Velocity and position controller joint configurations, LIDAR and IMU configurations

carone - The package consists of robot URDF, controllers, launch and worlds
python_scripts - The packages consits of python files to control the robot

Available launch files
1 gazebo.launch.py - Spawns the robot in gazebo
2 display.launch.py - Spawns the robot in RVIZ
3 debug.launch.py  - Spawns the robot in both gazebo and RVIZ
4 competition.launch.py - Spawns a competition setup and the robot

Controlling - The robot can be controlled by
1 Publishing commands
2 teleop script
3 Publisher script

Create a workspace (car_ws) and a /src folder and copy the packages in the src folder
In car_ws folder run the following

Terminal 1
$ colcon build
$ source install/setup.bash
$ ros2 launch carone gazebo.launch.py

To move the robot with a velocity

Terminal 2 [Control] 
$ source install/setup.bash
$ ros2 topic pub /joint_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [-5.0, 5.0],layout: {dim:[], data_offset: 1"}}

----OR-----

To control and navigate the vehicle in the plane using W A S D keys

$ source install/setup.bash
$ ros2 run python_scripts teleop

----OR-----

Publisher Script to move to location (10,10)
$ source install/setup.bash
$ ros2 run python_scripts publisher

If publisher throws an error make sure you have this library installed
$ sudo apt install ros-galactic-tf-transformations
$ sudo pip3 install transforms3d

For subscriber node, run
$ source install/setup.bash
$ ros2 run python_scripts subscriber


-> uncomment spawning coordinates in spawn_robot_ros2.launch.py file,  if you are launching competition file to make it spawn on track, or else it just spawns at origin.

For launching competition setup 
$ colcon build
$ source install/setup.bash
$ ros2 launch carone competition.launch.py

In other terminal use teleop command to control the robot using teleop keyboard

