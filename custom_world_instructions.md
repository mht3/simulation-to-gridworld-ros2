# Custom World Creation

Creating a custom world is tedious and is currently not automated. Three steps must be taken in order to successfully build and deploy a custom world in `ctf_package`. 

## Gazebo Model/Building Editor

The first step (and most straightforward) is to create your physical model in Gazebo with the Gazebo Model Editor. In this editor, worlds and even robots can be created!

To learn more about how to create your own robot, please see [this](http://classic.gazebosim.org/tutorials?tut=model_editor) tutorial.

The focus of this instruction file will be on creating a world, NOT a robot, as we already use the Robotis TurtleBot3 Burger.

In a terminal of your choosing, launch the gazebo editor:

```
gazebo
```

Once gazebo opens, click on `edit`->`Building Editor` as shown in the image below.

![image](https://github.com/Tran-Research-Group/simulation-to-gridworld-ros2/assets/60635839/cb13e8c4-da1b-4566-a741-fe152901d221)

You can now add walls with different textures! Feel free to mess around with the building editor and see how complicated of worlds you can create.

For a more in-depth tutorial on how to create worlds with the building editor, see [this](https://www.youtube.com/watch?v=gSurY5XlsIs) youtube video!

Once you're happy with the world you've created, save the file as `filename.world` in the `src/ctf_package/worlds` folder.

Here's an example of the world we created for capture the flag called `easy_ctf.world`.

![image](https://github.com/Tran-Research-Group/simulation-to-gridworld-ros2/assets/60635839/3eca4c9e-76ba-4990-bf32-61fb4406b394)


## Navigation Map Creation

NAV 2 requires a 2D map for navigation purposes. This can be visualized with `rviz`, an optional window to view a robot's joints, position, orientation, and navigation path. 

An example of a map used for our custom ctf world can be found in the `src/ctf_package/map` folder. There are two required files that must be present.

- `map.pgm`: The image of the map
- `map.yaml`: A file describing the path of the map along with additional parameters for scale and orientation.

An image of the map used in the custom `easy_ctf` world is shown below. 

![image](https://github.com/Tran-Research-Group/simulation-to-gridworld-ros2/assets/60635839/a53fa9e3-23c8-49fd-a0fc-2227cc49f3ce)

One way of creating the map files required for `nav2` is to use `SLAM` capabilities within `turtlebot`. In the launch file, set the `SLAM` argument to `true` and drive the robot around the map. You will see that obstacles and walls will become defined in the RVIZ map. Once you finish use the following command:

```
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: <file_of_map>, image_format: pgm}"
```

Finally, use the `.pgm` file to create a `.yaml` file to use for NAV 2.

## Multigrid Map

The last step is to create a multigrid map that mirrors a top-down view of the `.world` file you created. Again, this is a manual process and takes time to get a map that is pleasing to use. 

The multigrid map Ojas, Matt, and Evan made can be found in `src/control/multigrid_envs/ctf_env.py` in the `_gen_grid()` method.

![image](https://github.com/Tran-Research-Group/simulation-to-gridworld-ros2/assets/60635839/e24b4058-bca0-419c-bd2a-bc12c820fbea)

The original world created for `easy_ctf` is 10 m x 10 m, however, for multigrid, each grid space was chosen to be roughly 1/4 m to smoothly show transitions between grid spaces. This choice was completely arbitrary and can be changed in the `__init__` method at the bottom of `ctf_env.py`.
