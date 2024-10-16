#include <iostream>
#include <fstream>
#include <vector>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>
#include <NvOnnxParser.h>
#include <NvInferRuntime.h>
#include <algorithm>
#include <iterator>
#include <cmath> // for sigmoid function

// 이 코드는 단일 이미지에 대해서 tensorRT를 가지고 추론을 하는 코드입니다.
// 맨 아래 main함수에 iamge경로와, engine파일 경로를 적어주고 컴파일하고 돌려줍니다.
// 컴파일 명령어는 맨 아래 주석으로 되어 있습니다.

using namespace nvinfer1;
using namespace std;
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

class TensorRTPredictor
{
public:
    TensorRTPredictor(const std::string &enginePath)
    {
        engine = loadEngine(enginePath);
        if (engine)
        {
            context = engine->createExecutionContext();
            inputIndex = engine->getBindingIndex("images");
            outputIndex = engine->getBindingIndex("output0");
            inputDims = engine->getBindingDimensions(inputIndex);
            outputDims = engine->getBindingDimensions(outputIndex);
            input_h = inputDims.d[2];
            input_w = inputDims.d[3];
        }
    }
    ~TensorRTPredictor()
    {
        if (context)
            context->destroy();
        if (engine)
            engine->destroy();
        if (runtime)
            runtime->destroy();
    }

    void preprocess(const Mat &img, std::vector<float> &preprocessed)
    {
        Mat resized;
        resize(img, resized, Size(input_w, input_h));
        resized.convertTo(resized, CV_32FC3, 1.0 / 255);
        Mat channels[3];
        split(resized, channels);
        for (int i = 0; i < 3; ++i)
        {
            preprocessed.insert(preprocessed.end(), channels[i].begin<float>(), channels[i].end<float>());
        }
    }
    // 이미지 전처리 함수 (H, W, C에서 C, H, W로 변환 후 배치)
    void preprocessImage(const cv::Mat &img, float *gpuInput, int inputWidth, int inputHeight)
    {
        cv::Mat resized, floatImg;
        cv::resize(img, resized, cv::Size(inputWidth, inputHeight)); // 크기 조정
        resized.convertTo(floatImg, CV_32FC3, 1.0 / 255);            // 정규화

        // 이미지를 GPU 입력 버퍼로 복사
        cudaMemcpy(gpuInput, floatImg.ptr<float>(), inputWidth * inputHeight * 3 * sizeof(float), cudaMemcpyHostToDevice);
    }
    // 이 부분 이미지 폴더로 받기
    void predict(const std::string &imgPath)
    {
        Mat img1 = imread("../test.jpg", IMREAD_COLOR);
        Mat img2 = imread("../test640.jpg", IMREAD_COLOR);

        vector<cv::Mat> imgs = {img1, img2};
        batch_size = imgs.size();
        input_size = batch_size * 3 * input_w * input_h * sizeof(float);
        int each_output_size = (outputDims.d[1] * outputDims.d[2]);
        vector<float> output(batch_size * each_output_size);
        
        float * d_input = nullptr;
        float * d_output = nullptr; 
        cudaMalloc((void **)&d_input, input_size);
        cudaMalloc((void **)&d_output, output.size() * sizeof(float));

        vector<float> preprocessed(batch_size * 3 * input_h * input_w);
        for (int i = 0; i < batch_size; ++i)
        {
            vector<float> single_image_preprocessed;
            preprocess(imgs[i], single_image_preprocessed);
            copy(single_image_preprocessed.begin(), single_image_preprocessed.end(), 
                  preprocessed.begin() + i * single_image_preprocessed.size());
        }
        cudaMemcpy(d_input, preprocessed.data(),preprocessed.size() * sizeof(float), cudaMemcpyHostToDevice);
        void *bindings[] = {d_input, d_output};
        context->executeV2(bindings);
        cudaMemcpy(output.data(),d_output, output.size()* sizeof(float), cudaMemcpyDeviceToHost);
        
        for (int i = 0; i < batch_size; ++i)
        {
            vector<float> sliced_output(output.begin() + i * each_output_size, output.begin() + (i + 1) * each_output_size);
            postprocess(sliced_output, imgs[i]);
            count++;
        }
        cudaFree(d_input);
        cudaFree(d_output);
    }

private:
    ICudaEngine *engine = nullptr;
    IExecutionContext *context = nullptr;
    int inputIndex = -1;
    int outputIndex = -1;
    Dims inputDims;
    Dims outputDims;
    // Model input dimensions
    int input_h;
    int input_w;
    int batch_size;
    int input_size;
    int count = 0;

    void postprocess(const std::vector<float> &output, Mat &img)
    {
        cout << "Raw model output shape: (" << outputDims.d[0] << ", " << outputDims.d[1] << ", " << outputDims.d[2] << ")" << endl;
        int num_boxes = outputDims.d[2];
        std::vector<int> indices;
        std::vector<Rect> boxes;
        std::vector<float> scores;

        std::cout << "Filtered model output (confidence > 0.7):" << std::endl;

        int img_h = img.rows;
        int img_w = img.cols;

        for (int i = 0; i < num_boxes; ++i)
        {
            // float conf = 1.0 / (1.0 + exp(-output[4 * num_boxes + i]));  // Apply sigmoid to confidence score
            float conf = output[4 * num_boxes + i];
            if (conf > 0.7)
            {
                float x_center = output[i];
                float y_center = output[num_boxes + i];
                float width = output[2 * num_boxes + i];
                float height = output[3 * num_boxes + i];

                // 이미지 크기에 맞게 좌표 변환
                x_center = x_center * img_w / input_w;
                y_center = y_center * img_h / input_h;
                width = width * img_w / input_w;
                height = height * img_h / input_h;

                // 바운딩 박스를 좌표 형식으로 변환
                int x1 = static_cast<int>(x_center - width / 2);
                int y1 = static_cast<int>(y_center - height / 2);
                int x2 = static_cast<int>(x_center + width / 2);
                int y2 = static_cast<int>(y_center + height / 2);

                std::cout << "Confidence: " << conf << ", x1: " << x1 << ", y1: " << y1
                          << ", x2: " << x2 << ", y2: " << y2 << std::endl;

                boxes.emplace_back(Rect(Point(x1, y1), Point(x2, y2)));
                scores.push_back(conf);
            }
        }

        std::vector<int> nmsIndices;
        dnn::NMSBoxes(boxes, scores, 0.0, 0.5, nmsIndices);

        for (int idx : nmsIndices)
        {
            rectangle(img, boxes[idx], Scalar(0, 255, 0), 2);
            putText(img, "Wheel: " + std::to_string(scores[idx]), Point(boxes[idx].x, boxes[idx].y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
        }

        string name = "../result_img_" + to_string(count) + ".jpg";
        imwrite(name, img);
    }
};

int main()
{
    std::string enginePath = "../../enter_car_ws/weights/ver8n.engine";
    std::string imgPath = "../test.jpg";
    TensorRTPredictor predictor(enginePath);
    predictor.predict(imgPath);
    return 0;
}