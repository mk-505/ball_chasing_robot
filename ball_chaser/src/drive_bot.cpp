#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// include the ball_chaser drivetotarget header file 
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher; 

//function executes when drive_bot service requested, publishes inear x and angular velocity to robot wheel joints and after publishing the req velocities, feedback is returned with the requested wheel velocities 
bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
{
    //create motor command object 
    geometry_msgs::Twist motor_command;
    //set the requested wheel velocities
    motor_command.linear.x = req.linear_x; 
    motor_command.angular.z = req.angular_z; 
    
    // publish requested velocities to robot 
    motor_command_publisher.publish(motor_command);

    //return response message 
    res.msg_feedback = "Wheel velocities set - linear_x: " + std::to_string(req.linear_x) + ", angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);
    
    return true;
}
int main(int argc, char** argv)
{
    // Initialize the drive bot node
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    //inform ROS master we will publish message of type geometry_msgs::Twist on robot actuation topic with publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    //define a drive /ball_chaser/command_robot service with handle_drive_request callback function 
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request); 

    ROS_INFO("Ready to send wheel commands");

    //handle ROS communication events
    ros::spin(); 


    return 0; 
}



























