# Look Away: The Code

## Creating the empty `look_away` node script

The steps that you should take to create the `look_away` node are exactly the same as the steps you took to create the `simple_mover` and `arm_mover` scripts, but of course change the actual name of the file itself.

Open a new terminal, and type the following:

```sh
$ cd /home/workspace/catkin_ws/src/simple_arm/src/
$ gedit look_away.cpp
```

You have created and opened the C++ `look_away` file with the **gedit** editor. Now copy and paste the code below and save the file.

## look_away.cpp

```C++
#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

// Define global vector of joints last position, moving state of the arm, and the client that can request services
std::vector<double> joints_last_position{ 0, 0 };
bool moving_state = false;
ros::ServiceClient client;

// This function calls the safe_move service to safely move the arm to the center position
void move_arm_center()
{
    ROS_INFO_STREAM("Moving the arm to the center");

    // Request centered joint angles [1.57, 1.57]
    simple_arm::GoToPosition srv;
    srv.request.joint_1 = 1.57;
    srv.request.joint_2 = 1.57;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
}

// This callback function continuously executes and reads the arm joint angles position
void joint_states_callback(const sensor_msgs::JointState js)
{
    // Get joints current position
    std::vector<double> joints_current_position = js.position;

    // Define a tolerance threshold to compare double values
    double tolerance = 0.0005;

    // Check if the arm is moving by comparing its current joints position to its latest
    if (fabs(joints_current_position[0] - joints_last_position[0]) < tolerance && fabs(joints_current_position[1] - joints_last_position[1]) < tolerance)
        moving_state = false;
    else {
        moving_state = true;
        joints_last_position = joints_current_position;
    }
}

// This callback function continuously executes and reads the image data
void look_away_callback(const sensor_msgs::Image img)
{

    bool uniform_image = true;

    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] - img.data[0] != 0) {
            uniform_image = false;
            break;
        }
    }

    // If the image is uniform and the arm is not moving, move the arm to the center
    if (uniform_image == true && moving_state == false)
        move_arm_center();
}

int main(int argc, char** argv)
{
    // Initialize the look_away node and create a handle to it
    ros::init(argc, argv, "look_away");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from safe_move
    client = n.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

    // Subscribe to /simple_arm/joint_states topic to read the arm joints position inside the joint_states_callback function
    ros::Subscriber sub1 = n.subscribe("/simple_arm/joint_states", 10, joint_states_callback);

    // Subscribe to rgb_camera/image_raw topic to read the image data inside the look_away_callback function
    ros::Subscriber sub2 = n.subscribe("rgb_camera/image_raw", 10, look_away_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
```

## The code: Explained

```C++
#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
```

The header files are similar to those in `arm_mover`, except this time we included the `JointState.h`header file so that we can read the arm joints’ positions. We also include the `Image.h` header file so that we can use the camera data.

```C++
ros::init(argc, argv, "look_away");
ros::NodeHandle n;
```

Inside the C++ main function, the `look_away` node is initialized and a ROS NodeHandle object `n` is instantiated to communicate with ROS.

```C++
client = n.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");
```

A `client` object is created here. This object can request `GoToPosition` services from the `/arm_mover/safe_move` service created earlier in the `arm_mover` node. This client object is defined globally in the code, so we can request services within any function. In particular, this happens in the `move_arm_center()` function.

```C++
ros::Subscriber sub1 = n.subscribe("/simple_arm/joint_states", 10, joint_states_callback);
```

The first subscriber object `sub1`, subscribes to the `/simple_arm/joint_states` topic. By subscribing to this topic, we can track the arm position by reading the angle of each joint. The `queue_size` is set to 10, meaning that a maximum of 10 messages can be stored in the queue. The data from each new incoming message is passed to the `joint_states_callback` function.

```C++
ros::Subscriber sub2 = n.subscribe("rgb_camera/image_raw", 10, look_away_callback);
```

The second subscriber object `sub2`, subscribes to the `/rgb_camera/image_raw` topic. The `queue_size` is also set to 10. And the `look_away_callback` function is called each time a new message arrives.

```C++
ros::spin();
```

The `ros::spin()` function simply blocks until a shutdown request is received by the node.

```C++
void joint_states_callback(const sensor_msgs::JointState js)
{
    // Get joints current position
    std::vector<double> joints_current_position = js.position;

    // Define a tolerance threshold to compare double values
    double tolerance = 0.0005;

    // Check if the arm is moving by comparing its current joints position to its latest
    if (fabs(joints_current_position[0] - joints_last_position[0]) < tolerance && fabs(joints_current_position[1] - joints_last_position[1]) < tolerance)
        moving_state = false;
    else {
        moving_state = true;
        joints_last_position = joints_current_position;
    }
}
```

When `sub1` receives a message on the`/simple_arm/joint_states` topic, the message is passed to the `joint_states_callback` in the variable `js`. The `joint_states_callback()` function checks if the current joint states provided in `js` are the same as the previous joint states, which are stored in the global `joints_last_position` variable. If the current and previous joint states are the same (up to the specified error tolerance), then the arm has stopped moving, and the `moving_state` flag is set to `False`. This flag is defined globally so as to be shared with other functions in the code. On the other hand, if the current and previous joint states are different, then the arm is still moving. In this case, the function sets `moving_state` to `true` and updates the `joints_Last_position` variable with current position data stored in `joints_current_position`.

```C++
void look_away_callback(const sensor_msgs::Image img)
{

    bool uniform_image = true;

    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] - img.data[0] != 0) {
            uniform_image = false;
            break;
        }
    }

    // If the image is uniform and the arm is not moving, move the arm to the center
    if (uniform_image == true && moving_state == false)
        move_arm_center();
}
```

The `look_away_callback()` function receives [image data](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) from the `/rgb_camera/image_raw` topic. The callback function first checks if all color values in the image are the same as the color value of the first pixel. Then, if the image is uniform and the arm is not moving, the `move_arm_center()` function is called.

```C++
void move_arm_center()
{
    ROS_INFO_STREAM("Moving the arm to the center");

    // Request centered joint angles [1.57, 1.57]
    simple_arm::GoToPosition srv;
    srv.request.joint_1 = 1.57;
    srv.request.joint_2 = 1.57;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
}
```

Inside the `move_arm_center` function, a `GoToPosition` request message is created and sent using the `arm_mover/safe_move` service, moving both joint angles to `1.57` radians.