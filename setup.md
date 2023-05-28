# Installation Instructions

**Contributors:** Matt Taylor, Ojas Upalekar, and Evan Yu

## Getting Started

To clone the repository onto your local system, run the following command.

```
git clone https://github.com/Tran-Research-Group/simulation-to-gridworld-ros2.git
```

GitHub recently switched to token-based authentication for all accounts. Use [this link](https://docs.github.com/en/github/authenticating-to-github/keeping-your-account-and-data-secure/creating-a-personal-access-token) to create a personal access token. Make sure to allow permissions for everything and to set an expiration date long enough for your use of the repository.

## Pushing to the Repository

Always make sure to pull from the repository first.

```
git pull origin main
```


Once you make your changes, push them to the repository.

```
git add <files>
git commit -m <commit message>
git push origin main
```

## Python Setup

To setup the environment, run the following commands.

```
conda create --name sim2grid python=3.9
conda activate sim2grid
pip install -r requirements.txt
```
When running the code, make sure to start up the environment every time.

```
conda activate sim2grid
```

When running any code, make sure to run it from the root of the repository directory. This is done so that all users know how they should name their files.

```
python /path/to/simulation-to-gridworld-ros2/file.py
```

Note that if pip3 is used, the `python3` command must be used instead of `python`.

## ROS 2 Foxy Installation Instructions.

Note that if you're using the G15 laboratory laptop, this step should already be done.

Ros2 Foxy Installation Guide: [Here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

Each time a terminal is opened, run this command to activate ROS2 Foxy. This should automatically be put in the `.bashrc` file after proper setup.

```
source /opt/ros/foxy/setup.bash
```

Check ROS2 Foxy is the correct distrubution. This should print `foxy`. If Not, revisit the Foxy installation guide and make sure no steps are missed.

```
printenv ROS_DISTRO
```
## Gazebo 

Installation instructions for Gazebo for Ubuntu can be found [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu). Again, if you are using the G15 laptop, all of this setup should already be done.

## ROS2 Gazebo Packages

Download necessary ros2-gazebo packages

```
sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup

```

## Turtlebot3 Packages

Download necessary turtlebot3 packages: See full guide [Here](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html). All important installation instructions are listed below.

```
sudo apt install ros-foxy-turtlebot3*
```

Setup the gazebo model path
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
prefix turtlebot3_gazebo \
`/share/turtlebot3_gazebo/models/
```

Export the burger model as the befault model. After this is done, close and open the terminal. 

```
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
```

## Custom World Creation

Creating a custom world is tedious and is currently not automated. It takes several steps to successfully build a custom world in gazebo, because additional top-down maps must be created for multigrid and for NAV2 nodes (navigation).

Please see `custom_world_instructions.md` for further details.
