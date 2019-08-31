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

    std::vector<int> white_pixel{255, 255, 255};

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // Initiate variables
    int height = img.height;
    int width = img.width;
    int step = img.step;

    float x = 0.0;
    float z = 0.0;
    float offset_accumulated = 0;
    int count = 0;
    
// loop through all pixels

    for (int i = 0; i < height; i++){
        for (int j = 0; j < width; j++){
        // count white pixel
        // in ROS the sensor_msgs/image data is arranged in a 1D vector whereby one pixel is 
        // represented by three consecutive bytes (uint8) comprising the RED, BLUE, and GREEN color information.
            std::vector<int> current_pixel;
            current_pixel.push_back(img.data[i*step + j*3]); // R channel
            current_pixel.push_back(img.data[i*step + j*3 + 1]); // G channel
            current_pixel.push_back(img.data[i*step + j*3 + 2]); // R channel
            // std::cout<<current_pixel[0]<<std::endl;
            // std::cout<<current_pixel[1]<<std::endl;
            // std::cout<<current_pixel[2]<<std::endl;
            if (current_pixel == white_pixel){
            	count++;
            	offset_accumulated += j - width / 2.0;
            }
        }
    }
    
    if (count == 0){
        x = 0.0;
        z = 0.0;
    }
    else{
        x = 0.1;
        // Calculate the average offset (from -width/2.0 to +width/2.0)
        // Normalize the average offset (from -1.0 to 1.0)
        // Multiply with magic number -0.2 to turn
        z = -0.2 * offset_accumulated / count / (width /2.0);
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
