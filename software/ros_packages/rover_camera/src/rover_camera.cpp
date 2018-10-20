#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include "rover_camera/CameraControlMessage.h"

// Useful info
// RTSP stream is 704x480
// 3 image channels
// Image type 16

class RoverCamera{
public:
    RoverCamera(int argc, char** argv){
        ros::init(argc, argv, "camera");

        node_handle = new ros::NodeHandle("~");

        node_handle->param("is_rtsp_camera", is_rtsp_camera, false);
        node_handle->param("device_path", capture_device_path, std::string("/dev/video0"));
        node_handle->param("fps", fps, 30);

        node_handle->param("upside_down", upside_down, false);

        node_handle->param("large_image_width", large_image_width, 1280);
        node_handle->param("large_image_height", large_image_height, 720);
        node_handle->param("medium_image_width", medium_image_width, 640);
        node_handle->param("medium_image_height", medium_image_height, 360);
        node_handle->param("small_image_width", small_image_width, 256);
        node_handle->param("small_image_height", small_image_height, 144);

        broadcast_large_image = false;
        broadcast_medium_image = false;
        broadcast_small_image = true;

        if(is_rtsp_camera){
            cap = new cv::VideoCapture(capture_device_path);
            ROS_INFO_STREAM("Connecting to RTSP camera with path: " << capture_device_path);

        }else{
            cap = new cv::VideoCapture(capture_device_path);
            ROS_INFO_STREAM("Connecting to USB camera with path: " << capture_device_path);

            cap->set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
            cap->set(CV_CAP_PROP_FRAME_WIDTH, large_image_width);
            cap->set(CV_CAP_PROP_FRAME_HEIGHT, large_image_height);
            cap->set(CV_CAP_PROP_FPS, fps);
        }

        large_image_node_name = "image_" + std::to_string(large_image_width) + "x" + std::to_string(large_image_height);
        medium_image_node_name = "image_" + std::to_string(medium_image_width) + "x" + std::to_string(medium_image_height);
        small_image_node_name = "image_" + std::to_string(small_image_width) + "x" + std::to_string(small_image_height);

        large_image_transport = new image_transport::ImageTransport(*node_handle);
        medium_image_transport = new image_transport::ImageTransport(*node_handle);
        small_image_transport = new image_transport::ImageTransport(*node_handle);

        large_image_publisher = large_image_transport->advertise(large_image_node_name, 1);
        medium_image_publisher = medium_image_transport->advertise(medium_image_node_name, 1);
        small_image_publisher = small_image_transport->advertise(small_image_node_name, 1);

        control_subscriber = node_handle->subscribe("camera_control", 1, &RoverCamera::control_callback, this);

        if(is_rtsp_camera){
            loop_rate = new ros::Rate(fps + 10);
        }else{
            loop_rate = new ros::Rate(fps + 2);
        }

        image_black = cv::Mat(large_image_height, large_image_width, CV_8UC3, CvScalar(0, 0, 0));
    }

    void run(){
        if(!cap->isOpened()){
            return;
        }

        while (ros::ok()) {

            if(!is_rtsp_camera) {
                cap->read(image_large);
            }else{
                cap->read(image_rtsp_raw);
                image_black.copyTo(image_large);

                float image_scalar = float(image_large.rows) / image_rtsp_raw.rows;

                cv::resize(image_rtsp_raw, image_rtsp_scaled, CvSize(int(image_rtsp_raw.cols * image_scalar), int(image_rtsp_raw.rows * image_scalar)));

                int x = (image_large.cols - image_rtsp_scaled.cols) / 2;

                image_rtsp_scaled.copyTo(image_large(CvRect(x , 0, image_rtsp_scaled.cols, image_rtsp_scaled.rows)));
            }

            if(!image_large.empty()){
                if(upside_down){
                    cv::flip(image_large, image_large, -1);
                }

                if(broadcast_large_image){
                    large_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_large).toImageMsg();
                    large_image_publisher.publish(large_image_message);
                }else if(broadcast_medium_image){
                    cv::resize(image_large, image_medium, cv::Size(medium_image_width, medium_image_height));
                    medium_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_medium).toImageMsg();
                    medium_image_publisher.publish(medium_image_message);
                }else if(broadcast_small_image){
                    cv::resize(image_large, image_small, cv::Size(small_image_width, small_image_height));
                    small_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_small).toImageMsg();
                    small_image_publisher.publish(small_image_message);
                }
            }

            ros::spinOnce();
            loop_rate->sleep();
        }
    }

    void control_callback(const rover_camera::CameraControlMessage::ConstPtr& msg){
        broadcast_small_image = msg->enable_small_broadcast;
        broadcast_medium_image = msg->enable_medium_broadcast;
        broadcast_large_image = msg->enable_large_broadcast;
    }

    ~RoverCamera(){
        if(cap->isOpened()){
            cap->release();
        }
    }

private:
    ros::NodeHandle *node_handle;

    cv::VideoCapture *cap;

    bool is_rtsp_camera;

    std::string capture_device_path;
    int fps;

    ros::Rate *loop_rate;

    bool upside_down;

    int large_image_width;
    int large_image_height;
    int medium_image_width;
    int medium_image_height;
    int small_image_width;
    int small_image_height;

    bool broadcast_large_image;
    bool broadcast_medium_image;
    bool broadcast_small_image;

    std::string large_image_node_name;
    std::string medium_image_node_name;
    std::string small_image_node_name;

    image_transport::ImageTransport *large_image_transport;
    image_transport::ImageTransport *medium_image_transport;
    image_transport::ImageTransport *small_image_transport;

    image_transport::Publisher large_image_publisher;
    image_transport::Publisher medium_image_publisher;
    image_transport::Publisher small_image_publisher;

    ros::Subscriber control_subscriber;

    cv::Mat image_black;

    cv::Mat image_rtsp_raw;
    cv::Mat image_rtsp_scaled;
    cv::Mat image_large;
    cv::Mat image_medium;
    cv::Mat image_small;

    sensor_msgs::ImagePtr large_image_message;
    sensor_msgs::ImagePtr medium_image_message;
    sensor_msgs::ImagePtr small_image_message;

};

int main(int argc, char** argv)
{
    RoverCamera camera(argc, argv);
    camera.run();
}
