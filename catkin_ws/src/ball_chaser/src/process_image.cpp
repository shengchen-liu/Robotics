#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving the robot with lin_x: " + std::to_string((double) lin_x) + ", ang_z: " + std::to_string((double) ang_z));
    
    // Requrest service 
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
   
    // Call DriveToTarget service and pass the requested velocities
    if (!client.call(srv)){
        ROS_ERROR("Failed to call service ball_chaser!");
    } 
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

  vector<int> white_pixel{255, 255, 255};

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // Initiate variables
    int height = img.height;
    int step = img.step;

    float x = 0.0;
    float z = 0.0;
    float offset_accumulated = 0;
    int count = 0;
    
// loop through all pixels

    for (int i = 0; i < height; i++){
        for (int j = 0; j < step; j++){
        // count white pixel
            if (img.data[i * step + j] == white_pixel){
            count++;
            offset_accumulated += j - step / 2.0;
            }
        }
    }
    
    if (count == 0){
        x = 0.0;
        z = 0.0;
    }
    else{
        x = 0.1;
        // Calculate the average offset (from -step/2.0 to +step/2.0)
        // Normalize the average offset (from -1.0 to 1.0)
        // Multiply with magic number -4.0 to turn
        z = -4.0 * offset_accumulated / count / (step /2.0);
    }
    
    // send request to service
    drive_robot(x, z);
    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
