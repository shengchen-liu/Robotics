# Arm Mover: The Code

## Creating the empty `arm_mover` node script

The steps that you should take to create the `arm_mover` node are exactly the same as the steps you took to create the `simple_mover` node, except the actual name of the node itself.

Open a new terminal, and type the following:

```sh
$ cd /home/workspace/catkin_ws/src/simple_arm/src/
$ gedit arm_mover.cpp
```

You have created and opened the C++ `arm_mover` source code with the **gedit** editor. Now copy and paste the code below into the source code and save the file.

## arm_mover.cpp

```C++
#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>

// Global joint publisher variables
ros::Publisher joint1_pub, joint2_pub;

// This function checks and clamps the joint angles to a safe zone
std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2)
{
    // Define clamped joint angles and assign them to the requested ones
    float clamped_j1 = requested_j1;
    float clamped_j2 = requested_j2;

    // Get min and max joint parameters, and assigning them to their respective variables
    float min_j1, max_j1, min_j2, max_j2;
    // Assign a new node handle since we have no access to the main one
    ros::NodeHandle n2;
    // Get node name
    std::string node_name = ros::this_node::getName();
    // Get joints min and max parameters
    n2.getParam(node_name + "/min_joint_1_angle", min_j1);
    n2.getParam(node_name + "/max_joint_1_angle", max_j1);
    n2.getParam(node_name + "/min_joint_2_angle", min_j2);
    n2.getParam(node_name + "/max_joint_2_angle", max_j2);

    // Check if joint 1 falls in the safe zone, otherwise clamp it
    if (requested_j1 < min_j1 || requested_j1 > max_j1) {
        clamped_j1 = std::min(std::max(requested_j1, min_j1), max_j1);
        ROS_WARN("j1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
    }
    // Check if joint 2 falls in the safe zone, otherwise clamp it
    if (requested_j2 < min_j2 || requested_j2 > max_j2) {
        clamped_j2 = std::min(std::max(requested_j2, min_j2), max_j2);
        ROS_WARN("j2 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2);
    }

    // Store clamped joint angles in a clamped_data vector
    std::vector<float> clamped_data = { clamped_j1, clamped_j2 };

    return clamped_data;
}

// This callback function executes whenever a safe_move service is requested
bool handle_safe_move_request(simple_arm::GoToPosition::Request& req,
    simple_arm::GoToPosition::Response& res)
{

    ROS_INFO("GoToPositionRequest received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);

    // Check if requested joint angles are in the safe zone, otherwise clamp them
    std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);

    // Publish clamped joint angles to the arm
    std_msgs::Float64 joint1_angle, joint2_angle;

    joint1_angle.data = joints_angles[0];
    joint2_angle.data = joints_angles[1];

    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback = "Joint angles set - j1: " + std::to_string(joints_angles[0]) + " , j2: " + std::to_string(joints_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize the arm_mover node and create a handle to it
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle n;

    // Define two publishers to publish std_msgs::Float64 messages on joints respective topics
    joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Define a safe_move service with a handle_safe_move_request callback function
    ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
    ROS_INFO("Ready to send joint commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
```



<iframe class="embed-responsive-item" frameborder="0" allowfullscreen="1" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" title="YouTube video player" width="640" height="360" src="https://www.youtube.com/embed/TjYL_qmr_kg?showinfo=0&amp;rel=0&amp;autohide=1&amp;vq=hd720&amp;hl=en-us&amp;cc_load_policy=0&amp;enablejsapi=1&amp;origin=https%3A%2F%2Fclassroom.udacity.com&amp;widgetid=17" id="widget18" style="box-sizing: inherit; margin: 0px; padding: 0px; border: 0px; font: inherit; vertical-align: baseline; position: absolute; top: 0px; bottom: 0px; left: 0px; width: 770px; height: 433.125px;"></iframe>



## The code: Explained

```C++
#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>
```

The included modules for `arm_mover` are the same as `simple_arm`, with the exception of one new file. Namely, the `GoToPosition.h` header file, which is the header file generated from the `GoToPosition.srv` file we created earlier.

```C++
ros::init(argc, argv, "arm_mover");
ros::NodeHandle n;
```

Inside the C++ main function, the `arm_mover` node is initialized and a ROS NodeHandle object `n` is instantiated to communicate with ROS.

```C++
joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);
```

As we did earlier in the `simple_arm` node, two publisher objects are created to publish joint angles to the arm. These objects are defined globally so as to be easily accessible from all the other functions.

```C++
ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
```

Next, the `GoToPosition` service is created with the node name followed by `safe_move`. Generally, you want to name your services with the node name first to easily find them in large projects. This service is defined with a `handle_safe_move_request` callback function. The callback function runs when a service request is received.

```C++
ros::spin();
```

The `ros::spin()` function simply blocks until a shutdown request is received by the node.

```C++
bool handle_safe_move_request(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res)
```

When a client sends a `GoToPosition` request to the `safe_move` service, either from the terminal or from a separate node the handle_safe_move_request function is called. The function parameter `req` is of type `GoToPosition::Request`. And the service response parameter `res` is of type `GoToPosition::Response`.

```C++
std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);
```

This function passes the requested angles to the `clamp_at_boundaries()` function.

```C++
std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2)
{
    // Define clamped joint angles and assign them to the requested ones
    float clamped_j1 = requested_j1;
    float clamped_j2 = requested_j2;

    // Get min and max joint parameters, and assign them to their respective variables
    float min_j1, max_j1, min_j2, max_j2;
    // Assign a new node handle since we have no access to the main one
    ros::NodeHandle n2;
    // Get node name
    std::string node_name = ros::this_node::getName();
    // Get joints min and max parameters
    n2.getParam(node_name + "/min_joint_1_angle", min_j1);
    n2.getParam(node_name + "/max_joint_1_angle", max_j1);
    n2.getParam(node_name + "/min_joint_2_angle", min_j2);
    n2.getParam(node_name + "/max_joint_2_angle", max_j2);

    // Check if joint 1 falls in the safe zone, otherwise clamp it
    if (requested_j1 < min_j1 || requested_j1 > max_j1) {
        clamped_j1 = std::min(std::max(requested_j1, min_j1), max_j1);
        ROS_WARN("j1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
    }
    // Check if joint 2 falls in the safe zone, otherwise clamp it
    if (requested_j2 < min_j2 || requested_j2 > max_j2) {
        clamped_j2 = std::min(std::max(requested_j2, min_j2), max_j2);
        ROS_WARN("j2 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2);
    }

    // Store clamped joint angles in a clamped_data vector
    std::vector<float> clamped_data = { clamped_j1, clamped_j2 };

    return clamped_data;
}
```

The `clamp_at_boundaries()` function is responsible for enforcing the minimum and maximum joint angles for each joint. If the joint angles passed in are outside of the operable range, they will be “clamped” to the nearest allowable value. The minimum and maximum joint angles are retrieved from the parameter server each time `clamp_at_boundaries` is called. The rest of this function simply clamps the joint angle if necessary. Warning messages are logged if the requested joint angles are out of bounds.

```C++
std_msgs::Float64 joint1_angle, joint2_angle;

joint1_angle.data = joints_angles[0];
joint2_angle.data = joints_angles[1];

joint1_pub.publish(joint1_angle);
joint2_pub.publish(joint2_angle);
```

Then, the `handle_safe_move_request()` function publishes the clamped joint angles to the arm.

```C++
ros::Duration(3).sleep();
```

The `safe_move` service will be blocked for 3 seconds so the arm has enough time to move to the requested position.

```C++
res.msg_feedback = "Joint angles set - j1: " + std::to_string(joints_angles[0]) + " , j2: " + std::to_string(joints_angles[1]);
ROS_INFO_STREAM(res.msg_feedback);
```

Finally, the `safe_move` service returns back a message indicating that the arm has moved to its new position and displays the clamped joint angles.

## Next steps

Now that you've written the `arm_mover` node, the next step is to **build** it, **launch** it, and **test** it out via the command line!