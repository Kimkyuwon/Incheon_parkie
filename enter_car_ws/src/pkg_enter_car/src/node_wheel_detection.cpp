#include <iostream>
#include <fstream>
#include <vector>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <NvOnnxParser.h>
#include <NvInferRuntime.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <signal.h> // SIGINT 핸들링

using namespace nvinfer1; // TensorRT 네임스페이스를 사용
using namespace std;      // 표준 라이브러리 네임스페이스를 사용
using namespace cv;

class Logger : public ILogger
{
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
        {
            std::cout << msg << std::endl;
        }
    }
} gLogger;

IRuntime *runtime = createInferRuntime(gLogger);

ICudaEngine *loadEngine(const std::string &enginePath)
{
    std::ifstream file(enginePath, std::ios::binary);
    if (!file)
    {
        std::cerr << "Error opening engine file: " << enginePath << std::endl;
        return nullptr;
    }
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    return runtime->deserializeCudaEngine(buffer.data(), size);
}

// TensorRT 예측을 위한 클래스 정의
class TensorRTPredictor
{
public:
    std::vector<std::tuple<int, float, Rect>> results_;
    TensorRTPredictor(const std::string &enginePath)
    {
        engine = loadEngine(enginePath); // 엔진 파일 로드
        if (engine)
        {
            context = engine->createExecutionContext();             // 실행 컨텍스트 생성
            inputIndex = engine->getBindingIndex("images");         // 입력 바인딩 인덱스 얻기
            outputIndex = engine->getBindingIndex("output0");       // 출력 바인딩 인덱스 얻기
            inputDims = engine->getBindingDimensions(inputIndex);   // 입력 텐서 차원 얻기
            outputDims = engine->getBindingDimensions(outputIndex); // 출력 텐서 차원 얻기
            input_h_ = inputDims.d[2];
            input_w_ = inputDims.d[3];
        }
    }
    ~TensorRTPredictor()
    {
        if (context)
            context->destroy(); // 실행 컨텍스트 파괴
        if (engine)
            engine->destroy(); // 엔진 파괴
        if (runtime)
            runtime->destroy(); // 런타임 파괴
    }

    // 이미지 전처리 함수
    void preprocess(const Mat &img, std::vector<float> &preprocessed)
    {
        Mat resized;                                    // 리사이즈된 이미지 저장을 위한 변수
        resize(img, resized, Size(input_w_, input_h_)); // 이미지를 입력 텐서 크기로 리사이즈
        resized.convertTo(resized, CV_32F, 1.0 / 255);  // 이미지 값을 0-1 범위로 정규화
        Mat channels[3];                                // 채널 분리를 위한 배열
        split(resized, channels);                       // 이미지 채널 분리
        for (int i = 0; i < 3; ++i)
        {
            preprocessed.insert(preprocessed.end(), channels[i].begin<float>(), channels[i].end<float>()); // 분리된 채널을 벡터에 추가
        }
    }

    // 예측 함수 (추론 수행 및 후처리)
    std::vector<std::tuple<int, float, Rect>> predict(vector<cv::Mat> &imgs)
    {
        batch_size_ = imgs.size();
        input_size_ = batch_size_ * 3 * input_w_ * input_h_ * sizeof(float);
        int each_output_size = (outputDims.d[1] * outputDims.d[2]);
        vector<float> output(batch_size_ * each_output_size);

        float *d_input = nullptr;
        float *d_output = nullptr;
        cudaMalloc((void **)&d_input, input_size_);
        cudaMalloc((void **)&d_output, output.size() * sizeof(float));

        vector<float> preprocessed(batch_size_ * 3 * input_h_ * input_w_);
        for (int i = 0; i < batch_size_; ++i)
        {
            vector<float> single_image_preprocessed;
            preprocess(imgs[i], single_image_preprocessed);
            copy(single_image_preprocessed.begin(), single_image_preprocessed.end(),
                 preprocessed.begin() + i * single_image_preprocessed.size());
        }
        cudaMemcpy(d_input, preprocessed.data(), preprocessed.size() * sizeof(float), cudaMemcpyHostToDevice);
        void *bindings[] = {d_input, d_output};
        context->executeV2(bindings);
        cudaMemcpy(output.data(), d_output, output.size() * sizeof(float), cudaMemcpyDeviceToHost);

        for (int i = 0; i < batch_size_; ++i)
        {
            vector<float> sliced_output(output.begin() + i * each_output_size, output.begin() + (i + 1) * each_output_size);
            postprocess(sliced_output, imgs[i]);
            count_++;
        }
        cudaFree(d_input);
        cudaFree(d_output);
        return results_;
    }

private:
    ICudaEngine *engine = nullptr;        // CUDA 엔진을 위한 포인터
    IExecutionContext *context = nullptr; // 실행 컨텍스트를 위한 포인터
    int inputIndex = -1;                  // 입력 바인딩 인덱스
    int outputIndex = -1;                 // 출력 바인딩 인덱스
    Dims inputDims;                       // 입력 텐서 차원
    Dims outputDims;                      // 출력 텐서 차원

    // Model input dimensions
    int input_h_;
    int input_w_;
    int batch_size_;
    int input_size_;
    int count_ = 0;

    // 후처리 함수 (바운딩 박스 및 클래스 ID 추출)
    void postprocess(const std::vector<float> &output, Mat &img)
    {
        // std::cout << "Raw model output shape: (" << outputDims.d[0] << ", " << outputDims.d[1] << ", " << outputDims.d[2] << ")" << std::endl; // 출력 텐서의 모양 출력
        int num_boxes = outputDims.d[2]; // 바운딩 박스의 개수

        std::vector<int> indices;  // 유효한 인덱스를 저장할 벡터
        std::vector<Rect> boxes;   // 바운딩 박스를 저장할 벡터
        std::vector<float> scores; // 신뢰도를 저장할 벡터

        // std::cout << "Filtered model output (confidence > 0.5):" << std::endl; // 신뢰도가 0.7 이상인 결과만 필터링

        int img_h = img.rows; // 이미지의 높이
        int img_w = img.cols; // 이미지의 너비

        int input_h_ = inputDims.d[2]; // 입력 텐서의 높이
        int input_w_ = inputDims.d[3]; // 입력 텐서의 너비

        for (int i = 0; i < num_boxes; ++i)
        {
            float conf = output[4 * num_boxes + i]; // 신뢰도 값
            if (conf > 0.5)
            {
                float x_center = output[i];               // 중심 x 좌표
                float y_center = output[num_boxes + i];   // 중심 y 좌표
                float width = output[2 * num_boxes + i];  // 너비
                float height = output[3 * num_boxes + i]; // 높이

                x_center = x_center * img_w / input_w_; // 이미지 크기에 맞게 변환
                y_center = y_center * img_h / input_h_; // 이미지 크기에 맞게 변환
                width = width * img_w / input_w_;       // 이미지 크기에 맞게 변환
                height = height * img_h / input_h_;     // 이미지 크기에 맞게 변환

                int x1 = static_cast<int>(x_center - width / 2);  // 바운딩 박스의 왼쪽 상단 x 좌표
                int y1 = static_cast<int>(y_center - height / 2); // 바운딩 박스의 왼쪽 상단 y 좌표
                int x2 = static_cast<int>(x_center + width / 2);  // 바운딩 박스의 오른쪽 하단 x 좌표
                int y2 = static_cast<int>(y_center + height / 2); // 바운딩 박스의 오른쪽 하단 y 좌표

                boxes.emplace_back(Rect(Point(x1, y1), Point(x2, y2))); // 바운딩 박스를 벡터에 추가
                scores.push_back(conf);                                 // 신뢰도를 벡터에 추가
            }
        }

        std::vector<int> nmsIndices;                        // 비최대 억제 인덱스를 저장할 벡터
        dnn::NMSBoxes(boxes, scores, 0.0, 0.7, nmsIndices); // 비최대 억제 수행

        for (int idx : nmsIndices)
        {
            rectangle(img, boxes[idx], Scalar(0, 255, 0), 2);                                                                                               // 바운딩 박스를 그린다
            putText(img, "Wheel: " + std::to_string(scores[idx]), Point(boxes[idx].x, boxes[idx].y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2); // 텍스트 추가
            // 추론 결과를 결과 벡터에 추가
            results_.emplace_back(std::make_tuple(0, scores[idx], boxes[idx]));
        }
    }
};

class WheelDetection : public rclcpp::Node
{
public:
    WheelDetection() : Node("wheel_detection")
    {
        string engine_path = "weights/ver8n.engine";                   // 엔진 파일 경로 얻기
        std::cout << "Engine file path: " << engine_path << std::endl; // 경로 출력
        predictor = std::make_shared<TensorRTPredictor>(engine_path);
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "zed_imgs_topic", rclcpp::QoS(1), std::bind(&WheelDetection::camera_callback, this, std::placeholders::_1)); // 이미지 토픽 구독

        // 종료 시그널 핸들러 설정
        signal(SIGINT, WheelDetection::signal_handler);
    }
    ~WheelDetection()
    {
        cv::destroyAllWindows();
    }

private:
    static void signal_handler(int signum)
    {
        // SIGINT (Ctrl+C) 신호 처리
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT 받음, 모든 창 닫기");
        rclcpp::shutdown(); // ROS2 노드 종료
    }
    void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            cv::Mat imgs = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image; //OpenCV 이미지로 변환

            // preprocessing
            // 결합된 이미지를 다시 분리하여 vector<cv::Mat>로 복원
            std::vector<cv::Mat> image_vector;
            const int original_height = imgs.rows / 2;
            const int total_height = imgs.rows; // 결합된 이미지의 전체 높이
            const int width = imgs.cols;
            // 첫 번째, 두 번째 이미지 영역을 슬라이싱 (상단,하단 부분)
            cv::Mat first_image = imgs(cv::Rect(0, 0, width, original_height)).clone();
            cv::Mat second_image = imgs(cv::Rect(0, original_height, width, original_height)).clone();
            image_vector.push_back(first_image);
            image_vector.push_back(second_image);

            // inference
            std::vector<std::tuple<int, float, Rect>> results; // yolo output = {label,conf,bbox vertex}
            results = predictor->predict(image_vector);        // 추론 수행
            // publish_yolo_results(results);

            // display
            cv::Mat display_mat;
            cv::hconcat(image_vector, display_mat);
            cv::resize(display_mat, display_mat, cv::Size(), 0.5, 0.5);
            cv::imshow("ZED Camera Front-Rear", display_mat);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }
    // 추론 결과 퍼블리싱 함수
    void publish_yolo_results(const vector<tuple<int, float, Rect>> &results)
    {

        // //Todo: Mst Type result_msg;  // 결과 메시지 생성
        // for (const auto& result : results) {
        //     result_msg.class_ids.push_back(get<0>(result));  // 클래스 ID 추가
        //     result_msg.confidences.push_back(get<1>(result));  // 신뢰도 추가
        //     result_msg.x_min.push_back(get<2>(result).x);  // 바운딩 박스 x_min 추가
        //     result_msg.y_min.push_back(get<2>(result).y);  // 바운딩 박스 y_min 추가
        //     result_msg.x_max.push_back(get<2>(result).x + get<2>(result).width);  // 바운딩 박스 x_max 추가
        //     result_msg.y_max.push_back(get<2>(result).y + get<2>(result).height);  // 바운딩 박스 y_max 추가
        // }
        // publisher_->publish(result_msg);  // 메시지 발행

        // 터미널에 결과 출력
        cout << "YOLOv8 Inference Results:" << endl;
        for (const auto &result : results)
        {
            cout << "Confidence: " << get<1>(result)
                 << ", BBox: [" << get<2>(result).x << ", " << get<2>(result).y
                 << ", " << get<2>(result).x + get<2>(result).width << ", " << get<2>(result).y + get<2>(result).height << "]" << endl;
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_; // 이미지 구독자
    std::shared_ptr<TensorRTPredictor> predictor;                                     // TensorRT 예측기
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelDetection>());
    rclcpp::shutdown();
    return 0;
}