#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Create a service message to send to the server
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the requested velocities
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    // Define where the pixel was found: left, mid, right
    int left_bound = img.width / 3;
    int right_bound = 2 * img.width / 3;

    bool found_white_pixel = false;
    int pixel_position = -1;

    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height * img.width; i++)
    {
        // Each pixel is represented by 3 consecutive values: red, green, and blue channels
        int pixel_index = i * 3;
        if (img.data[pixel_index] == white_pixel && 
            img.data[pixel_index + 1] == white_pixel && 
            img.data[pixel_index + 2] == white_pixel)
        {
            pixel_position = i % img.width;
            found_white_pixel = true;
            break;  // Stop searching once we find the white pixel
        }
    }

    // If a white pixel is found, determine which region it falls into and command the robot
    if (found_white_pixel)
    {
        if (pixel_position < left_bound)
        {
            // White ball is on the left side
            drive_robot(0.5, 1.0);  // Move forward and turn left
        }
        else if (pixel_position > right_bound)
        {
            // White ball is on the right side
            drive_robot(0.5, -1.0);  // Move forward and turn right
        }
        else
        {
            // White ball is in the center
            drive_robot(0.5, 0.0);  // Move straight forward
        }
    }
    else
    {
        // No white ball found, stop the robot
        drive_robot(0.0, 0.0);
    }
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
