# Look Away: Build, Launch and Interact

## Modifying CMakeLists.txt

Before compiling the `look_away.cpp` code, you have to include instructions for the compiler. As a reminder, for every C++ ROS node you write, you have to add its dependencies in `CMakeLists.txt file`. Open the `simple_arm` package’s `CMakeLists.txt` file, located in `/home/workspace/catkin_ws/src/simple_arm/`, and add the following instructions at the bottom of the file:

```html
add_executable(look_away src/look_away.cpp)
target_link_libraries(look_away ${catkin_LIBRARIES})
add_dependencies(look_away simple_arm_generate_messages_cpp)
```

## Building the package

Now that you’ve written the `look_away` C++ script, and included specific instructions for your compiler, let’s build the package:

```sh
$ cd /home/workspace/catkin_ws/
$ catkin_make
```

## Launching the nodes

You can now launch and interact with `simple_arm` just as before:

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch simple_arm robot_spawn.launch
```

## Interacting with the arm

After launching, the arm should move away from the grey sky and look towards the blocks. To view the camera image stream, you can use the same command as before:

```sh
$ rqt_image_view /rgb_camera/image_raw
```

To check that everything is working as expected, open a new terminal and send a service call to point the arm directly up towards the sky (note that the line break in the message is necessary):

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosservice call /arm_mover/safe_move "joint_1: 0
joint_2: 0"
```