#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    std::string capture_device_path;
    int fps;

    int large_image_width;
    int large_image_height;
    int medium_image_width;
    int medium_image_height;
    int small_image_width;
    int small_image_height;

    ros::init(argc, argv, "camera");
    ros::NodeHandle node_handle("~");

    node_handle.param("device_path", capture_device_path, std::string("/dev/video0"));
    node_handle.param("fps", fps, 30);

    node_handle.param("large_image_width", large_image_width, 1280);
    node_handle.param("large_image_height", large_image_height, 720);
    node_handle.param("medium_image_width", medium_image_width, 640);
    node_handle.param("medium_image_height", medium_image_height, 360);
    node_handle.param("small_image_width", small_image_width, 256);
    node_handle.param("small_image_height", small_image_height, 144);

    cv::VideoCapture cap(capture_device_path);

    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, large_image_width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, large_image_height);
    cap.set(CV_CAP_PROP_FPS, fps);

    if(!cap.isOpened()){
        return -1;
    }

    std::string large_image_node_name = "image_" + std::to_string(large_image_width) + "x" + std::to_string(large_image_height);
    std::string medium_image_node_name = "image_" + std::to_string(medium_image_width) + "x" + std::to_string(medium_image_height);
    std::string small_image_node_name = "image_" + std::to_string(small_image_width) + "x" + std::to_string(small_image_height);

    image_transport::ImageTransport large_image_transport(node_handle);
    image_transport::ImageTransport medium_image_transport(node_handle);
    image_transport::ImageTransport small_image_transport(node_handle);

    image_transport::Publisher large_image_publisher = large_image_transport.advertise(large_image_node_name, 1);
    image_transport::Publisher medium_image_publisher = medium_image_transport.advertise(medium_image_node_name, 1);
    image_transport::Publisher small_image_publisher = small_image_transport.advertise(small_image_node_name, 1);

    cv::Mat image_large;
    cv::Mat image_medium;
    cv::Mat image_small;

    ros::Rate loop_rate(fps + 2);

    while (ros::ok()) {

        cap.read(image_large);

        if(!image_large.empty()){
            cv::resize(image_large, image_medium, cv::Size(medium_image_width, medium_image_height));
            cv::resize(image_medium, image_small, cv::Size(small_image_width, small_image_height));

            sensor_msgs::ImagePtr large_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_large).toImageMsg();
            sensor_msgs::ImagePtr medium_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_medium).toImageMsg();
            sensor_msgs::ImagePtr small_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_small).toImageMsg();

            large_image_publisher.publish(large_image_message);
            medium_image_publisher.publish(medium_image_message);
            small_image_publisher.publish(small_image_message);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();
}
