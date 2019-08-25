# Arm Mover

You’ve written your first ROS C++ node! This was no trivial task. You’ve had to learn quite a few things to get to this point.

But before we rush off, we have more ground to cover:

- Custom message generation
- Services
- Parameters
- Launch Files

In order to gain an understanding of the above, you will write another node called `arm_mover`.

## Description of Arm Mover

In many respects, `arm_mover` is quite similar to `simple_mover`. Like `simple_mover`, it is responsible for commanding the arm to move. However, instead of simply commanding the arm to follow a predetermined trajectory, the `arm_mover` node provides the service `safe_move`, which allows other nodes in the system to send `movement_commands`.

In addition to allowing movements via a service interface, `arm_mover` also allows for configurable minimum and maximum joint angles, by using parameters.

## Creating a new service definition

An interaction with a service consists of two messages. A node passes a request message to the service, and the service returns a response message to the node. The definitions of the request and response message types are contained within .srv files living in the `srv` directory under the package’s root.

Let’s define a new service for `simple_arm`. We shall call it `GoToPosition`.

```sh
$ cd /home/workspace/catkin_ws/src/simple_arm/
$ mkdir srv
$ cd srv
$ gedit GoToPosition.srv
```

You should now edit `GoToPosition.srv` with gedit, so it contains the following:

```text
float64 joint_1
float64 joint_2
---
string msg_feedback
```

Service definitions always contain two sections, separated by a ‘---’ line. The first section is the definition of the request message. Here, a request consists of two float64 fields, one for each of `simple_arm`’s joints. The second section contains the service response. The response contains only a single field, msg_feedback. The `msg_feedback` field is of type string, and is responsible for indicating that the arm has moved to a new position.

Note: Defining a custom message type is very similar. The only differences is that message definitions live within the `msg` directory of the package root, have a `.msg` extension, and do not contain the `---`section divider. You can find more detailed information on creating [messages](http://wiki.ros.org/msg) and [services](http://wiki.ros.org/srv) on the ROS wiki.

## Modifying CMakeLists.txt

As a reminder, in order for catkin to generate the C++ libraries which allow you to utilize messages in your code you must modify `simple_arm`’s `CMakeLists.txt` file. You can find this file in`/home/workspace/catkin_ws/src/simple_arm/`.

First, uncomment the `add_service_files()` macro so it looks like this:

```text
add_service_files(
   FILES
   GoToPosition.srv
)
```

This tells catkin to add the newly created service file.

Then, make sure that the `generate_messages()` macro is uncommented:

```text
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)
```

This macro is actually responsible for generating the code.

To force ROS to compile your C++ code with C++ 11 include this line of code:

```text
add_compile_options(-std=c++11)
```

## Modifying package.xml

Now that you have updated the `CMakeLists.txt` file, there’s one more file which needs to be modified: `package.xml`.

`package.xml` is responsible for defining many of the package’s properties, such as the name of the package, version numbers, authors, maintainers, and dependencies.

Right now, we’ll focus on the dependencies. You already learned about build-time dependencies and run-time package dependencies. When `rosdep` is searching for these dependencies, it’s the `package.xml` file that is being parsed. So make sure that the `message_generation` build dependency and the `message_runtime` run dependency exist in `package.xml`.

```html
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>

  <run_depend>controller_manager</run_depend>
  <run_depend>effort_controllers</run_depend>
  <run_depend>gazebo_plugins</run_depend>
  <run_depend>gazebo_ros</run_depend>
  <run_depend>gazebo_ros_control</run_depend>
  <run_depend>joint_state_controller</run_depend>
  <run_depend>joint_state_publisher</run_depend>
  <run_depend>robot_state_publisher</run_depend>
  <run_depend>message_runtime</run_depend>
  <run_depend>xacro</run_depend>
```

For more information about `package.xml`, check out the [ROS Wiki](http://wiki.ros.org/catkin/package.xml).

## Checking Service with ROS

Now that you’ve created your `GoToPosition` service file, let's make sure that ROS can see it using the `rossrv show` command:

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rossrv show GoToPosition
```

You will see:

```text
[simple_arm/GoToPosition]:
float64 joint_1
float64 joint_2
---
string msg_feedback
```

This indicates that ROS can see your service.

Great job, you accomplished so much in this lesson! First you created the `GoToPosition.srv` file. Then, you’ve added its dependencies in `CMakeLists.txt`. In addition, you checked for the build and run dependencies in `package.xml`. Lastly, you checked if ROS can see your service file. Now, let’s move onto the code for `arm_mover`.