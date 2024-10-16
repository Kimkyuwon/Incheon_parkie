#include <fstream>
#include <iostream>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

using namespace nvinfer1;

// Logger 클래스 정의
class Logger : public ILogger
{
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
        {
            std::cout << msg << std::endl;
        }
    }
};

// ONNX 모델을 TensorRT 엔진으로 변환하는 함수
void convertONNXtoTRT(const std::string &onnxModelPath, const std::string &engineFilePath)
{
    Logger logger;

    // TensorRT builder 생성
    IBuilder *builder = createInferBuilder(logger);
    if (!builder)
    {
        std::cerr << "TensorRT builder 생성 실패" << std::endl;
        return;
    }

    // NetworkDefinition 생성 (EXPLICIT_BATCH 플래그 사용)
    uint32_t explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    INetworkDefinition *network = builder->createNetworkV2(explicitBatch);
    if (!network)
    {
        std::cerr << "TensorRT network 생성 실패" << std::endl;
        builder->destroy();
        return;
    }

    // ONNX Parser 생성
    nvonnxparser::IParser *parser = nvonnxparser::createParser(*network, logger);
    if (!parser)
    {
        std::cerr << "ONNX parser 생성 실패" << std::endl;
        network->destroy();
        builder->destroy();
        return;
    }

    // ONNX 모델 파싱
    if (!parser->parseFromFile(onnxModelPath.c_str(), static_cast<int>(ILogger::Severity::kWARNING)))
    {
        std::cerr << "ONNX 모델 파싱 실패" << std::endl;
        parser->destroy();
        network->destroy();
        builder->destroy();
        return;
    }

    // Builder Config 생성
    IBuilderConfig *config = builder->createBuilderConfig();
    if (!config)
    {
        std::cerr << "BuilderConfig 생성 실패" << std::endl;
        parser->destroy();
        network->destroy();
        builder->destroy();
        return;
    }

    config->setMaxWorkspaceSize(16 << 30); // 16GB 워크스페이스 설정

    // TensorRT 엔진 생성
    ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
    if (!engine)
    {
        std::cerr << "TensorRT 엔진 생성 실패" << std::endl;
        config->destroy();
        parser->destroy();
        network->destroy();
        builder->destroy();
        return;
    }

    // 엔진 직렬화 및 파일로 저장
    IHostMemory *serializedEngine = engine->serialize();
    if (!serializedEngine)
    {
        std::cerr << "엔진 직렬화 실패" << std::endl;
        engine->destroy();
        config->destroy();
        parser->destroy();
        network->destroy();
        builder->destroy();
        return;
    }

    std::ofstream engineFile(engineFilePath, std::ios::binary);
    if (!engineFile)
    {
        std::cerr << "엔진 파일 저장 실패" << std::endl;
        serializedEngine->destroy();
        engine->destroy();
        config->destroy();
        parser->destroy();
        network->destroy();
        builder->destroy();
        return;
    }

    engineFile.write(reinterpret_cast<const char *>(serializedEngine->data()), serializedEngine->size());

    // 모든 객체 정리
    serializedEngine->destroy();
    engine->destroy();
    config->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();
}

int main()
{
    std::string onnxModelPath = "../../enter_car_ws/weights/ver8n.onnx";
    std::string engineFilePath = "../../enter_car_ws/weights/ver8n.engine";

    convertONNXtoTRT(onnxModelPath, engineFilePath);

    return 0;
}
