#include <iostream>
// display
#include <opencv2/opencv.hpp>
// zed
#include <sl/Camera.hpp>
// ini
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// image transport
#include "cv_bridge/cv_bridge.h"
// #include "image_transport/image_transport.hpp"

using namespace std;

int main()
{
    // ptree 선언 & read file
    boost::property_tree::ptree pt; 
    boost::property_tree::ini_parser::read_ini("../settings/camera.ini", pt);
    const bool display=pt.get<bool>("debug.display");
 
    // 시리얼 넘버를 사용하여 각 카메라를 초기화
    const int serial_number_front = pt.get<int>("front.serial_number");// first ZED 카메라의 시리얼 넘버
    const int serial_number_rear = pt.get<int>("rear.serial_number");// second ZED 카메라의 시리얼 넘버

    // Initialize ZED cameras
    sl::Camera zed_front, zed_rear;
    sl::InitParameters init_params;
    init_params.sdk_verbose = true; // debug info during camera initialization
    init_params.camera_resolution = sl::RESOLUTION::HD1080;
    init_params.camera_fps = 30;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    init_params.depth_mode = sl::DEPTH_MODE::NONE;

    // Open the first camera
    init_params.input.setFromSerialNumber(serial_number_front);
    sl::ERROR_CODE err_front = zed_front.open(init_params);
    if (err_front != sl::ERROR_CODE::SUCCESS)
    {
        cerr << "Error opening ZED camera 1: " << sl::toString(err_front) << endl;
        return -1;         
    }

    // // Open the second camera
    init_params.input.setFromSerialNumber(serial_number_rear);
    sl::ERROR_CODE err_rear = zed_rear.open(init_params);
    if (err_rear != sl::ERROR_CODE::SUCCESS)
    {
        cerr << "Error opening ZED camera 2: " << sl::toString(err_rear) << endl;
        return -1;
    }

    // Create sl::Mat to hold images
    sl::Mat image_front, image_rear;
   

    // Create OpenCV windows
    cv::namedWindow("ZED Camera Front", cv::WINDOW_NORMAL);
    cv::namedWindow("ZED Camera Rear", cv::WINDOW_NORMAL);

    while (true)
    {
        vector<cv::Mat> images;
        // Grab frames from both cameras
        if (zed_front.grab() == sl::ERROR_CODE::SUCCESS)
        {
            zed_front.retrieveImage(image_front, sl::VIEW::LEFT);
            cv::Mat cv_image_front(image_front.getHeight(), image_front.getWidth(), CV_8UC4, image_front.getPtr<uchar>());
            cv::cvtColor(cv_image_front, cv_image_front, cv::COLOR_BGRA2BGR);
            images.push_back(cv_image_front);
            if(display) cv::imshow("ZED Camera Front", cv_image_front);

        }

        if (zed_rear.grab() == sl::ERROR_CODE::SUCCESS)
        {
            zed_rear.retrieveImage(image_rear, sl::VIEW::RIGHT);
            cv::Mat cv_image_rear(image_rear.getHeight(), image_rear.getWidth(), CV_8UC4, image_rear.getPtr<uchar>());
            cv::cvtColor(cv_image_rear, cv_image_rear, cv::COLOR_BGRA2BGR);
            images.push_back(cv_image_rear);
            if(display) cv::imshow("ZED Camera Rear", cv_image_rear);
        }

        // Exit loop if 'q' is pressed
        if (cv::waitKey(1) == 'q') break;
    }

    // Clean up
    zed_front.close();
    zed_rear.close();
    cv::destroyAllWindows();

    return 0;
}
