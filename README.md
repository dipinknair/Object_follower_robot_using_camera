# Mobile robot object follower using camera and lidar
A mobile robot modelled in Gazebo environment. Object detection using camera and lidar which is visualized using Rviz. A demo of this project is shown below.

<img src="images/run.gif" alt="demo" width="500" height="280"/></a>
### world and models

Solarized dark             |  Solarized Ocean          | Solarized dark             
:-------------------------:|:-------------------------:| :-------------------------:
![](https://images/robot.png)  |  ![](https://...Ocean.png) | ![](https://...Ocean.png)

## Overview  
Created two ROS packages inside your `catkin_ws/src`: the `drive_bot` and the `ball_chaser`. Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:  
1. `drive_bot`:  
* Create a `my_robot` ROS package to hold your robot, the white ball, and the world.
* Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot’s differential drive, lidar, and camera. The robot you design should be significantly different from the one presented in the project lesson. Implement significant changes such as adjusting the color, wheel radius, and chassis dimensions. Or completely redesign the robot model! After all you want to impress your future employers :-D
* House your robot inside the world you built in the **Build My World** project.
* Add a white-colored ball to your Gazebo world and save a new copy of this world.
* The `world.launch` file should launch your world with the white-colored ball and your robot.
2. `ball_chaser`:
* Create a `ball_chaser` ROS package to hold your C++ nodes.
* Write a `drive_bot` C++ node that will provide a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
* Write a `process_image` C++ node that reads your robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, your node should request a service via a client to drive the robot towards it.
* The `ball_chaser.launch` should run both the `drive_bot` and the `process_image` nodes.  
## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 
## Setup Instructionfor prerequisite  
 
1. Install Gazebo and [ROS](http://wiki.ros.org/ROS/Installation) on Linux.
2. Install `sudo apt-get install ros-${ROS_DISTRO}-gazebo-ros-pkgs`
(If you are using gazebo11 it should be `sudo apt-get install ros-${ROS_DISTRO}-gazebo11-ros-pkgs`)
3. Cmake and gcc/g++

4. Build and run your code.  
## Project Description  
Directory Structure  
```
.Project                # Go Chase It Project
├── my_robot                       # my_robot package
│   ├── launch                     # launch folder for launch files
│   │   ├── robot_description.launch
│   │   ├── world.launch
│   ├── meshes                     # meshes folder for sensors
│   │   ├── hokuyo.dae
│   ├── urdf                       # urdf folder for xarco files
│   │   ├── my_robot.gazebo
│   │   ├── my_robot.xacro
│   ├── world                      # world folder for world files
│   │   ├── UdacityOffice.world
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
├── ball_chaser                    # ball_chaser package
│   ├── launch                     # launch folder for launch files
│   │   ├── ball_chaser.launch
│   ├── src                        # source folder for C++ scripts
│   │   ├── drive_bot.cpp
│   │   ├── process_images.cpp
│   ├── srv                        # service folder for ROS services
│   │   ├── DriveToTarget.srv
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
```
## Build and Launch

1. Clone and initialize project with a catkin workspace
```console
$ mkdir catkin_ws && cd catkin_ws
$ git clone https://github.com/dipinknair/Object_follower_robot_using_camera.git
$ mv Object_follower_robot_using_camera src
$ cd src && catkin_init_workspace
```

2. Move back to `catkin_ws\` and build
```
$ cd ..
$ catkin_make
```

3. Launch the world
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

4. Open another terminal (Ctrl+Shift+T), and launch the `ball_chaser` package
```
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```

5. Play around! Pick up the white ball and place in front of the mobile robot. The robot will follow the ball.
Footer

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Alert!!!

Here comes the most stupid thing:

After you follow the whole instruction on Udacity, you'll find out that your project won't be passed because of "Spawnmodel Failure: Model Name Already exists". Even though the whole project works with this little error, those reviewer still won't let you pass.

All you have to do is delete everything about ```"<model name='my_robot'>"``` in "office.world" file.

There is nothing mentioned in their instrcutions. Hope this'll help anyone who is in need.
