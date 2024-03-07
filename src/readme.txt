___________________________________________________________________________________________________________________________

README - RUTHWIK DASYAM _ ROBOT LEARNING HOMEWORK 3
___________________________________________________________________________________________________________________________
Contents: 

URDF file has the all the joint and link configurations, Velocity and position controller joint configurations, LIDAR configurations
carone - The package consists of robot URDF, controllers, launch and worlds
python_scripts - The packages consits of python files to control the robot
Available launch file ---->  1 gazebo.launch.py - Spawns the robot in gazebo in its environment
Controlling - The robot can be controlled by ---> teleop.py = Teleoperation , autodrive.py = Autonomous driving
____________________________________________________________________________________________________________________________
Behavior or robot -> 

1. A Behavior to show that it avoids obstacles and keeps moving forward
2. A 4 wheeled vehicle is designed in SolidWorks and simulated in Gazebo using ROS. This vehicle uses LIDAR to detect obstacles
    and a control method to keep it going based on the error from sensor output
3. simulation environment - ROS, Gazebo
4. Proportional control is used to calculate the turning angle based on error detected from sensors, This control method avoids going in other directions
    and always keeps the vechicle alligened to the forward path
4. One example could be Rat behavior, when it is left in a loop, it juts finds the way to move forward and keeps moving in loops, it never identifies the
    path out of it. Keeps moving forward untill it is taken out of the loop.
____________________________________________________________________________________________________________________________

        [_________Docker___________]
        <_><_><_><_><_><_><_><_><_><

** Docker - osrf/ros:galactic image **

1) [^-^] Terminal 1 <> Build and run the container
---------------------------------------------------------
$ docker build --no-cache -t myrobot -f myrobot .
$ docker run --rm --privileged --net=host -it -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all --runtime=nvidia --env="DISPLAY" --device=/dev/video0:/dev/video0 --name infiniterobot_container infiniterobot---------------------------------------------------------------------------------------
---------------------------------------------------------
Launch the Simulation in this terminal with commands given below
-------------------------------------
$ source install/setup.bash
$ ros2 launch carone gazebo.launch.py
-------------------------------------


2) [^-^] Terminal 2 <> Start new Terminal to run python scripts
----------------------------------------------
$ docker exec -it myrobot_container /bin/bash
----------------------------------------------

Run teleop or autonomous in this terminal - Commands given below
------------------------------------
$ source install/setup.bash
    <> For teleoperation - control with W S A D keys
$ ros2 run python_scripts teleop
    <> For complete autodriving in the loop 
$ ros2 run python_scripts autodrive
------------------------------------
__________________________________________________________________________________________________________________________________

 [__________ Simlation without Docker ____________]
          <_><_><_><_><_><_><_><_><_><


1) [^-^] Terminal 1 <> Launch package

Download homework3_ws
Navigate into the folder from your terminal
---------------------------------------------------------
$ source install/setup.bash
$ colon build 
$ ros2 launch carone gazebo.launch.py
-------------------------------------


2) [^-^] Terminal 2 <> Start new Terminal to run python scripts

Run teleop or autonomous in this terminal - Commands given below
------------------------------------
$ source install/setup.bash
    <> For teleoperation - control with W S A D keys
$ ros2 run python_scripts teleop
    <> For complete autodriving in the loop 
$ ros2 run python_scripts autodrive
------------------------------------
__________________________________________________________________________________________________________________________________