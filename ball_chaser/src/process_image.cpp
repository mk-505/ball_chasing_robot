#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <torch/script.h>

// Global client to request services
ros::ServiceClient client;

// Load YOLOv5 model
torch::jit::script::Module model;
bool model_loaded = false;

void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service command_robot");
    }
}

void process_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;

    if (!model_loaded)
    {
        ROS_WARN("Model not loaded, skipping frame.");
        return;
    }

    // Resize and convert image to tensor
    cv::Mat img_resized;
    cv::resize(frame, img_resized, cv::Size(640, 640));  // YOLOv5 expects 640x640
    img_resized.convertTo(img_resized, CV_32F, 1.0 / 255);

    torch::Tensor img_tensor = torch::from_blob(img_resized.data, {1, 640, 640, 3}).permute({0, 3, 1, 2}).contiguous();
    img_tensor = img_tensor.to(torch::kCPU);

    // Inference
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(img_tensor);
    at::Tensor output = model.forward(inputs).toTuple()->elements()[0].toTensor();

    // Post-process detections
    bool found_ball = false;
    int frame_center = frame.cols / 2;
    int ball_center_x = -1;

    for (int i = 0; i < output.size(0); ++i)
    {
        auto detection = output[i];
        float confidence = detection[4].item<float>();
        if (confidence > 0.6)
        {
            // Get class label (e.g. class 0 for ball)
            int class_id = detection[5].item<int>();
            if (class_id == 0)  // Assuming ball is class 0
            {
                float x_center = detection[0].item<float>() * frame.cols;
                ball_center_x = static_cast<int>(x_center);
                found_ball = true;
                break;
            }
        }
    }

    if (found_ball)
    {
        if (ball_center_x < frame_center - frame.cols / 6)
            drive_robot(0.5, 1.0); // Left
        else if (ball_center_x > frame_center + frame.cols / 6)
            drive_robot(0.5, -1.0); // Right
        else
            drive_robot(0.5, 0.0); // Center
    }
    else
    {
        drive_robot(0.0, 0.0);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    try
    {
        model = torch::jit::load("/path/to/your/yolov5_model.pt");
        model.eval();
        model_loaded = true;
    }
    catch (const c10::Error& e)
    {
        ROS_ERROR("Error loading the YOLOv5 model: %s", e.what());
        return 1;
    }

    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ros::spin();
    return 0;
}
