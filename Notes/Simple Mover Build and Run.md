# Simple Mover: Build and Run

Before you can run the `simple_mover` node, you have to compile the C++ script.

## Modifying CMakeLists.txt

In order for catkin to generate the C++ libraries, you must first modify `simple_arm`’s `CMakeLists.txt`.

CMake is the build tool underlying catkin, and `CMakeLists.txt` is a CMake script used by catkin. If you’re familiar with the concept of makefiles, this is similar.

Navigate to the package `CMakeLists.txt` file and open it:

```sh
$ cd /home/workspace/catkin_ws/src/simple_arm/
$ gedit CMakeLists.txt 
```

First, ensure that the `find_package()` macro lists `std_msgs`, `message_generation`, and`controller_manager` as required packages. The `find_package()` macro should look as follows:

```html
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation
        controller_manager
)
```

As the names might imply, the `std_msgs` package contains all of the basic message types, and `message_generation` is required to generate message libraries for all the supported languages (cpp, lisp, python, javascript). The `contoller_manager` is another package responsible for controlling the arm.

Now, add the following block of code at the bottom of the file:

```html
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simple_mover src/simple_mover.cpp)
target_link_libraries(simple_mover ${catkin_LIBRARIES})
add_dependencies(simple_mover simple_arm_generate_messages_cpp)
```

These instructions ask the compiler to include the directories, executable file, link libraries, and dependencies for your C++ code:

```html
add_executable(node_name sourcecode_directory)
```

Creates the executable `simple_mover` file.

```html
target_link_libraries(node_name ${catkin_LIBRARIES})
```

This will add all the linked libraries to the compiler.

```html
add_dependencies(node_name package_name_generate_messages_cpp)
```

Generates message headers for this package before you can use them.

Keep in mind that you should always include these instructions whenever you want to write a C++ ROS node. For more information about `CMakeLists.txt` check out [the CMakeLists.txt page](http://wiki.ros.org/catkin/CMakeLists.txt) on the ROS wiki.

## Building the Package

Now that you have included specific instructions for your compiler, let’s build the package:

```sh
$ cd /home/workspace/catkin_ws/
$ catkin_make
```

## Running simple_mover

Assuming that your workspace has recently been built, you can launch `simple_arm` as follows:

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch simple_arm robot_spawn.launch
```

Once the ROS Master, Gazebo, and all of our relevant nodes are up and running, we can finally launch `simple_mover`. To do so, open a new terminal and type the following commands:

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosrun simple_arm simple_mover
```



## `simple_mover` GitHub branch

You can always download a copy of this branch [here](https://github.com/udacity/RoboND-simple_arm/tree/simple_mover).

Congratulations! You’ve now written your first ROS node in C++!