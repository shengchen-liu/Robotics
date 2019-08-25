# Arm Mover: Build, Launch and Interact

## Modifying CMakeLists.txt

Before compiling the `arm_mover.cpp` code, you have to include instructions for the compiler. To do so, open the `simple_arm` package `CMakeLists.txt` file located in `/home/workspace/catkin_ws/src/simple_arm/`, and add the following instructions at the bottom of the file:

```html
add_executable(arm_mover src/arm_mover.cpp)
target_link_libraries(arm_mover ${catkin_LIBRARIES})
add_dependencies(arm_mover simple_arm_generate_messages_cpp)
```

## Building the package

Now that you’ve written the `arm_mover` C++ script, and included specific instructions for your compiler, let’s build the package:

```sh
$ cd /home/workspace/catkin_ws/
$ catkin_make
```

## Launching the project with the new service

To get the `arm_mover` node, and accompanying `safe_move` service, to launch along with all of the other nodes, modify `robot_spawn.launch`.

Launch files, when they exist, are located within the `launch` directory in the root of a catkin package. Inside a launch file, you can instruct ROS Master which nodes to run. Also you can specify certain parameters and arguments for each of your nodes. Thus, a launch file is necessary inside a ROS package containing more than one node or a node with multiple parameters and arguments. This launch file can run all the nodes within a single command: `roslaunch package_name launch_file.launch`. `simple_arm`’s launch file is located in `/home/workspace/catkin_ws/src/simple_arm/launch`

To get the `arm_mover` node to launch, add the following:

```xml
  <!-- The arm mover node -->
  <node name="arm_mover" type="arm_mover" pkg="simple_arm" output="screen">
    <rosparam>
      min_joint_1_angle: 0
      max_joint_1_angle: 1.57
      min_joint_2_angle: 0
      max_joint_2_angle: 1.0
    </rosparam>
  </node>
```

Inside the launch file, the node tag specifies the name, type, package name and output channel. The ROS parameters specify the min and max joint angles. More information on the format of the launch file can be found on the [XML page of the ROS wiki](http://wiki.ros.org/roslaunch/XML).

## Testing the new service

Now that you've built your code and modified the launch file, you are ready to test it all out.

Launch the `simple_arm`, verify that the `arm_mover` node is running and that the `safe_move` service is listed:

**Note:** You will need to make sure that you've exited your previous `roslaunch` session before re-launching.

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch simple_arm robot_spawn.launch
```

Then, in a new terminal, verify that the node and service have indeed launched.

```text
$ rosnode list
$ rosservice list
```

Check that both the service (`/arm_mover/safe_move`) and the node (`/arm_mover`) show up as expected. If they do not appear, check the logs in the `roscore` console. You can now interact with the service using `rosservice`.

To view the camera image stream, you can use the command `rqt_image_view` (you can learn more about rqt and the associated tools on the [RQT page of the ROS wiki](http://wiki.ros.org/rqt)):

```sh
$ rqt_image_view /rgb_camera/image_raw
```



[![img](https://video.udacity-data.com/topher/2018/November/5bdb8627_rqtimage/rqtimage.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/1f349ee0-9c40-4964-a6a8-4e0818a15fde/modules/d0fbb2f2-55d1-4217-8116-a52ac989c07f/lessons/e007bba9-c618-4709-9afd-2dc976d401cb/concepts/22dd55da-b30a-4938-8d7d-b91c44d957c4#)



## Adjusting the view

The camera is displaying a gray image. This is to be expected, given that it is pointing straight up, towards the gray sky of our Gazebo world.

To point the camera towards the numbered blocks on the countertop, we need to rotate both joint 1 and joint 2 by approximately pi/2 radians. Let’s give that a try:

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosservice call /arm_mover/safe_move "joint_1: 1.57
joint_2: 1.57"
```

Note: `rosservice call` can tab-complete the request message, so that you don’t have to worry about writing it out by hand. Also, be sure to include a line break between the two joint parameters.

Upon entering the command, you should see the arm move and eventually stop, reporting the new joint clamped angles to the console. This is as expected.

What was not expected was the resulting position of the arm. Looking at the `roscore` console, we can see the problem. The requested angle for joint 2 was out of the safe bounds, so it was clamped. We requested 1.57 radians, but the maximum joint angle was set to 1.0 radians.

By setting the `max_joint_2_angle` on the parameter server, we should be able to increase joint 2’s maximum angle and bring the blocks into view the next time we request a service. To update that parameter, use the command `rosparam`

```sh
$ rosparam set /arm_mover/max_joint_2_angle 1.57
```

Now we should be able to move the arm such that all of the blocks are within the field of view of the camera:

```sh
$ rosservice call /arm_mover/safe_move "joint_1: 1.57
joint_2: 1.57"
```



[![img](https://video.udacity-data.com/topher/2018/November/5bdb8639_camerablocks/camerablocks.png)](https://classroom.udacity.com/nanodegrees/nd209/parts/1f349ee0-9c40-4964-a6a8-4e0818a15fde/modules/d0fbb2f2-55d1-4217-8116-a52ac989c07f/lessons/e007bba9-c618-4709-9afd-2dc976d401cb/concepts/22dd55da-b30a-4938-8d7d-b91c44d957c4#)



And there you have it. All of the blocks are within the field of view!

## `arm_mover` GitHub branch

You can always download a copy of this branch [from the GitHub repo](https://github.com/udacity/RoboND-simple_arm/tree/arm_mover).