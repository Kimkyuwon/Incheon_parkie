#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sl/Camera.hpp> // zed
#include <opencv2/opencv.hpp>
// ini file
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <signal.h> // SIGINT 핸들링

using namespace std;

class ZedCameraPublisher : public rclcpp::Node
{
public:
  ZedCameraPublisher() : Node("zed_camera_publisher")
  {
    // ptree 선언 & read file
    boost::property_tree::ini_parser::read_ini("settings/camera.ini", pt);
    display_ = pt.get<bool>("debug.display");
    serial_number_front_ = pt.get<int>("front.serial_number"); // first ZED 카메라의 시리얼 넘버
    serial_number_rear_ = pt.get<int>("rear.serial_number");   // second ZED 카메라의 시리얼 넘버
    init_params_.camera_resolution = sl::RESOLUTION::HD1080;
    init_params_.camera_fps = 30; // 30Hz
    init_params_.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    init_params_.depth_mode = sl::DEPTH_MODE::NONE;

    // Open the first camera
    init_params_.input.setFromSerialNumber(serial_number_front_);
    sl::ERROR_CODE err_front = zed_front_.open(init_params_);
    if (err_front != sl::ERROR_CODE::SUCCESS)
    {
      cerr << "Error opening ZED camera 1: " << sl::toString(err_front) << endl;
      rclcpp::shutdown();
    }
    // Open the second camera
    init_params_.input.setFromSerialNumber(serial_number_rear_);
    sl::ERROR_CODE err_rear = zed_rear_.open(init_params_);
    if (err_rear != sl::ERROR_CODE::SUCCESS)
    {
      cerr << "Error opening ZED camera 2: " << sl::toString(err_rear) << endl;
      rclcpp::shutdown();
    }

    auto zed_front_info = zed_front_.getCameraInformation();
    std::cout << "front_right 카메라의 Intrinsic Parameters:" << std::endl;
    std::cout << "fx: " << zed_front_info.camera_configuration.calibration_parameters.left_cam.fx << std::endl;
    std::cout << "fy: " << zed_front_info.camera_configuration.calibration_parameters.left_cam.fy << std::endl;
    std::cout << "cx: " << zed_front_info.camera_configuration.calibration_parameters.left_cam.cx << std::endl;
    std::cout << "cy: " << zed_front_info.camera_configuration.calibration_parameters.left_cam.cy << std::endl;
    // intrinsic parameter 출력
    auto zed_rear_info = zed_rear_.getCameraInformation();
    std::cout << "rear_left 카메라의 Intrinsic Parameters:" << std::endl;
    std::cout << "fx: " << zed_rear_info.camera_configuration.calibration_parameters.right_cam.fx << std::endl;
    std::cout << "fy: " << zed_rear_info.camera_configuration.calibration_parameters.right_cam.fy << std::endl;
    std::cout << "cx: " << zed_rear_info.camera_configuration.calibration_parameters.right_cam.cx << std::endl;
    std::cout << "cy: " << zed_rear_info.camera_configuration.calibration_parameters.right_cam.cy << std::endl;

    // 퍼블리셔 생성
    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("zed_imgs_topic", 1);

    // 30Hz 주기로 타이머 생성
    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&ZedCameraPublisher::publish_image, this)); // 1000ms / 30Hz = 33ms

    // 종료 시그널 핸들러 설정
    signal(SIGINT, ZedCameraPublisher::signal_handler);
  }

  ~ZedCameraPublisher()
  {
    zed_front_.close();
    zed_rear_.close();
    cv::destroyAllWindows();
  }

private:
  static void signal_handler(int signum)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT 받음, 모든 창 닫기"); // SIGINT (Ctrl+C) 신호 처리
    rclcpp::shutdown();                                                     // ROS2 노드 종료
  }
  void publish_image()
  {
    // Create sl::Mat to hold images
    sl::Mat image_front, image_rear;
    vector<cv::Mat> image_vector;

    // ZED 이미지 캡처
    if (zed_front_.grab() == sl::ERROR_CODE::SUCCESS)
    {
      // 왼쪽 카메라 이미지 가져오기
      zed_front_.retrieveImage(image_front, sl::VIEW::LEFT);
      // ZED 이미지를 OpenCV의 cv::Mat으로 변환
      cv::Mat cv_image_front(image_front.getHeight(), image_front.getWidth(), CV_8UC4, image_front.getPtr<uchar>());
      cv::cvtColor(cv_image_front, cv_image_front, cv::COLOR_BGRA2BGR);
      image_vector.push_back(cv_image_front);
      if (display_)
      {
        cv::imshow("ZED Camera Front", cv_image_front);
        cv::waitKey(1);
      }
    }
    if (zed_rear_.grab() == sl::ERROR_CODE::SUCCESS)
    {
      zed_rear_.retrieveImage(image_rear, sl::VIEW::RIGHT);
      cv::Mat cv_image_rear(image_rear.getHeight(), image_rear.getWidth(), CV_8UC4, image_rear.getPtr<uchar>());
      cv::cvtColor(cv_image_rear, cv_image_rear, cv::COLOR_BGRA2BGR);
      image_vector.push_back(cv_image_rear);
      if (display_)
      {
        cv::imshow("ZED Camera Rear", cv_image_rear);
        cv::waitKey(1);
      }
    }
    if (image_vector.size() == 2)
    {
      // 여러 개의 이미지를 세로로 결합
      cv::Mat combined_image;
      cv::vconcat(image_vector, combined_image);
      // 결합된 이미지를 압축
      std::vector<uchar> compressed_data;
      cv::imencode(".jpg", combined_image, compressed_data);

      // 압축된 데이터를 CompressedImage 메시지로 변환
      auto compressed_image_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      compressed_image_msg->format = "jpeg";
      compressed_image_msg->data = compressed_data;
      compressed_image_msg->header.stamp = this->now(); // 현재 시간 설정
      // 이미지 퍼블리시
      publisher_->publish(std::move(compressed_image_msg));

      // image 캡처 옵션
      // 키 입력 대기
      char key = cv::waitKey(1); // 1ms 대기
      if (key == 'c')
      {
        // 이미지 캡처
        std::string zed_front = "./captureimg/ZED Camera Front" + std::to_string(captureCount_) + ".png";
        std::string zed_rear = "./captureimg/ZED Camera Rear" + std::to_string(captureCount_++) + ".png";
        if (cv::imwrite(zed_front, image_vector[0]) && cv::imwrite(zed_rear, image_vector[1]))
        {
          std::cout << "Image captured: " << zed_front << std::endl;
          std::cout << "Image captured: " << zed_rear << std::endl;
        }
        else
        {
          std::cerr << "Error: Could not save the image." << std::endl;
        }
      }
    }

    else
    {
      RCLCPP_WARN(this->get_logger(), "ZED에서 이미지를 가져오는 데 실패했습니다.");
    }
  }

  boost::property_tree::ptree pt;
  bool display_ = false;
  // 시리얼 넘버를 사용하여 각 카메라를 초기화
  int serial_number_front_, serial_number_rear_;
  // ZED 초기화 파라미터 설정
  sl::Camera zed_front_, zed_rear_;
  sl::InitParameters init_params_;

  // CaptureCount
  int captureCount_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_; // 퍼블리셔
  rclcpp::TimerBase::SharedPtr timer_;                                        // 타이머
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedCameraPublisher>());
  rclcpp::shutdown();
  return 0;
}