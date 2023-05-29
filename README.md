# simulation-to-gridworld-ros2

**Contributors:** Matt Taylor, Ojas Upalekar, and Evan Yu

Capture the flag environment using ROS2 and Gazebo

Running on Ubuntu 20.04, ROS2 Foxy

> **_NOTE:_**  Copied From [Tran-Research-Group](https://github.com/Tran-Research-Group)

## Instructions

For one time installation and environment setup instructions, see `setup.md`.

## Activate Environment

Each time a terminal is opened, run this command to activate the custom simulation environment. Make sure you are in the base folde, i.e. `simulation-to-gridworld-ros2/`. 

```
source install/setup.bash
```

Note: If you are using a shell other than bash, use that shell extension instead. E.g. `setup.zsh`

## Development 

If a file is changed in the ctf_package library, make sure to build and source.

```
colcon build
source install/setup.bash
```

## Launch Gazebo with ROS2

### Custom CTF World

This launches 2 turtlebots in a 10x10 m capture the flag environment. Each agent has separate velocity, imu, and differential drive nodes that can be accessed. The world itself is called `easy_ctf.world` and is shown below. 

![image](https://user-images.githubusercontent.com/60635839/234325191-275a4aab-5fe7-4ea7-b240-bdff2d713bf7.png)


Using the 1v1 launch file, we can spawn 2 Turtlebots in this world file!

```
ros2 launch ctf_package 1v1_ctf.launch.py
```

Once this command is run, Gazebo should open looking like the image below.

![image](https://user-images.githubusercontent.com/60635839/234327772-f381fcde-ae14-416e-be42-1e2b1df1cabe.png)


### Manual Control of turtlebot

Create a teleoperation node. This will allow a user to manually control a turtlebot using the WAXD keys. Note that they key S makes the turtlebot stop.

The turtlebot corresponding to the red team is `turtlebot_0` while the blue team is `turtlebot_1`.

```
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap cmd_vel:=turtlebot_0/cmd_vel
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap cmd_vel:=turtlebot_1/cmd_vel
```
Now you should be able to control the turtlebot with w,a,s,d, and x keys!


## Waypoint Navigation Demo

Currently, this repository supports automated python waypoint navigation for a **single** robot. Multigrid can also be used on top of the Gazebo and ROS2 environment. Multigrid creates a simpler environment to work with for RL applications.


The following navigation demo given an example with a single turtlebot spawned in the same `easy_ctf` custom world file as before. To launch the world and spawn the Turtlebot, run the following in a terminal:


`ros2 launch ctf_package single_robot_ctf.launch.py`

Currently, this launches two windows: Gazebo and RViz as shown in the screenshot below. It's important to note that the RViz window is not needed for navigation purposes. It does, however, provide a nice 2D visualization of the turtlebot joints and path planning capabilities. Initially, the agent starts in position (0,-2) relative to Gazebo's coordinate system which has a center of (0, 0). In the screenshot, the X-axis points up and the Y-axis points left.

![image](https://github.com/Tran-Research-Group/simulation-to-gridworld-ros2/assets/60635839/1636b86a-ae55-4242-bf64-5375c700fab6)

In a separate terminal, run the multigrid navigation code. This file provides a hard-coded policy to the agent. Three waypoints are given for demonstration purposes: (0, 2), (-1, 2), and (0, -2). As you can see, the agent moves to the three positions in Gazebo/RViz and in the multigrid viewer! 

`python3 src/control/ctf_multigrid_navigation.py`

![image](https://github.com/Tran-Research-Group/simulation-to-gridworld-ros2/assets/60635839/2b426104-2069-4e73-a0f2-74e854de088e)

### Video

https://github.com/mht3/simulation-to-gridworld-ros2/assets/60635839/3ba8e306-2a89-48b4-81a5-b291363cbc02
