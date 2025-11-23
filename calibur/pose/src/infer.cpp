#include <iostream>
#include <fstream>

#include <NvOnnxParser.h>

#include "infer.h"
#include "preprocess.h"
#include "postprocess.h"
#include "calibrator.h"
#include "utils.h"

using namespace nvinfer1;

YoloDetector::YoloDetector(const std::string trtFile)
    : trtFile_(trtFile)
{
    gLogger = Logger(ILogger::Severity::kERROR);
    cudaSetDevice(kGpuId);

    CHECK(cudaStreamCreate(&stream));

    // load or build engine
    get_engine();
    if (!engine) {
        std::cerr << "Engine is null after get_engine()" << std::endl;
        return;
    }

    // create execution context
    context = engine->createExecutionContext();
    if (!context) {
        std::cerr << "Failed to create execution context!" << std::endl;
        return;
    }

    // Get I/O tensor names from the engine
    const ICudaEngine& eng = context->getEngine();
    const char* inputName  = eng.getIOTensorName(0);
    const char* outputName = eng.getIOTensorName(1);

    // Fix input shape: [1, 3, kInputH, kInputW]
    Dims4 inputDims{1, 3, kInputH, kInputW};
    context->setInputShape(inputName, inputDims);

    // Query output shape after setting input shape
    Dims outDims = context->getTensorShape(outputName);  // e.g. [1, 56, 8400]
    if (outDims.nbDims < 3) {
        std::cerr << "Unexpected output dims nbDims = " << outDims.nbDims << std::endl;
        return;
    }

    // [1, 56, 8400] -> 8400 candidates in dim[2]
    OUTPUT_CANDIDATES = outDims.d[2];

    int outputSize = 1;
    for (int i = 0; i < outDims.nbDims; ++i) {
        outputSize *= outDims.d[i];
    }

    // prepare output data space on host
    outputData = new float[1 + kMaxNumOutputBbox * kNumBoxElement];

    // prepare input and output space on device
    vBufferD.resize(2, nullptr);
    CHECK(cudaMalloc(&vBufferD[0], 3 * kInputH * kInputW * sizeof(float)));       // input
    CHECK(cudaMalloc(&vBufferD[1], outputSize * sizeof(float)));                  // output

    CHECK(cudaMalloc(&transposeDevice, outputSize * sizeof(float)));
    CHECK(cudaMalloc(&decodeDevice, (1 + kMaxNumOutputBbox * kNumBoxElement) * sizeof(float)));
}

void YoloDetector::get_engine(){
    if (access(trtFile_.c_str(), F_OK) == 0){
        // load existing plan file
        std::ifstream engineFile(trtFile_, std::ios::binary);
        long int fsize = 0;

        engineFile.seekg(0, engineFile.end);
        fsize = engineFile.tellg();
        engineFile.seekg(0, engineFile.beg);
        std::vector<char> engineString(fsize);
        engineFile.read(engineString.data(), fsize);
        if (engineString.empty()) {
            std::cout << "Failed getting serialized engine!" << std::endl;
            return;
        }
        std::cout << "Succeeded getting serialized engine!" << std::endl;

        runtime = createInferRuntime(gLogger);
        engine  = runtime->deserializeCudaEngine(engineString.data(), fsize);
        if (engine == nullptr) {
            std::cout << "Failed loading engine!" << std::endl;
            return;
        }
        std::cout << "Succeeded loading engine!" << std::endl;
    } else {
        // build from ONNX
        IBuilder*            builder = createInferBuilder(gLogger);
        INetworkDefinition*  network = builder->createNetworkV2(
            1U << int(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
        IOptimizationProfile* profile = builder->createOptimizationProfile();
        IBuilderConfig*      config  = builder->createBuilderConfig();

        // TRT 10: use setMemoryPoolLimit instead of setMaxWorkspaceSize
        config->setMemoryPoolLimit(
            nvinfer1::MemoryPoolType::kWORKSPACE,
            1ull << 30
        );

        IInt8Calibrator*     pCalibrator = nullptr;
        if (bFP16Mode) {
            config->setFlag(BuilderFlag::kFP16);
        }
        if (bINT8Mode) {
            config->setFlag(BuilderFlag::kINT8);
            int batchSize = 8;
            pCalibrator   = new Int8EntropyCalibrator2(
                batchSize, kInputW, kInputH,
                calibrationDataPath.c_str(), cacheFile.c_str());
            config->setInt8Calibrator(pCalibrator);
        }

        nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, gLogger);
        if (!parser->parseFromFile(onnxFile.c_str(), int(gLogger.reportableSeverity))){
            std::cout << "Failed parsing .onnx file!" << std::endl;
            for (int i = 0; i < parser->getNbErrors(); ++i){
                auto* error = parser->getError(i);
                std::cout << int(error->code()) << ":" << error->desc() << std::endl;
            }
            return;
        }
        std::cout << "Succeeded parsing .onnx file!" << std::endl;

        ITensor* inputTensor = network->getInput(0);
        Dims4 fixedInputDims{1, 3, kInputH, kInputW};

        profile->setDimensions(inputTensor->getName(), OptProfileSelector::kMIN, fixedInputDims);
        profile->setDimensions(inputTensor->getName(), OptProfileSelector::kOPT, fixedInputDims);
        profile->setDimensions(inputTensor->getName(), OptProfileSelector::kMAX, fixedInputDims);
        config->addOptimizationProfile(profile);

        IHostMemory* engineString = builder->buildSerializedNetwork(*network, *config);
        if (!engineString) {
            std::cout << "Failed to build serialized network!" << std::endl;
            return;
        }
        std::cout << "Succeeded building serialized engine!" << std::endl;

        runtime = createInferRuntime(gLogger);
        engine  = runtime->deserializeCudaEngine(engineString->data(), engineString->size());
        if (engine == nullptr) {
            std::cout << "Failed building engine!" << std::endl;
            return;
        }
        std::cout << "Succeeded building engine!" << std::endl;

        if (bINT8Mode && pCalibrator != nullptr){
            delete pCalibrator;
        }

        // save plan file
        std::ofstream engineFile(trtFile_, std::ios::binary);
        engineFile.write(static_cast<char*>(engineString->data()), engineString->size());
        std::cout << "Succeeded saving .plan file!" << std::endl;

        delete engineString;
        delete parser;
        delete config;
        delete network;
        delete builder;
    }
}

YoloDetector::~YoloDetector(){
    cudaStreamDestroy(stream);

    for (int i = 0; i < 2; ++i){
        if (vBufferD[i]) {
            CHECK(cudaFree(vBufferD[i]));
        }
    }

    if (transposeDevice) CHECK(cudaFree(transposeDevice));
    if (decodeDevice)    CHECK(cudaFree(decodeDevice));

    delete[] outputData;

    delete context;
    delete engine;
    delete runtime;
}

std::vector<Detection> YoloDetector::inference(cv::Mat& img){
    if (img.empty()) return {};

    // put input on device, then letterbox、bgr to rgb、hwc to chw、normalize.
    preprocess(img, (float*)vBufferD[0], kInputH, kInputW, stream);

    // tensorrt inference (TensorRT 10 style)
    const ICudaEngine& eng = context->getEngine();
    const char* inputName  = eng.getIOTensorName(0);
    const char* outputName = eng.getIOTensorName(1);

    // (Re)set input shape in case you want to support dynamic shapes later
    Dims4 inputDims{1, 3, kInputH, kInputW};
    context->setInputShape(inputName, inputDims);

    // bind device buffers
    context->setInputTensorAddress(inputName,  vBufferD[0]);
    context->setOutputTensorAddress(outputName, vBufferD[1]);

    // enqueueV3 replaces enqueueV2
    context->enqueueV3(stream);

    // transpose [1, 56, 8400] -> [1, 8400, 56]
    transpose(
        (float*)vBufferD[1],
        transposeDevice,
        OUTPUT_CANDIDATES,
        4 + kNumClass + kNumKpt * kKptDims,
        stream
    );

    // convert [1, 8400, 56] to [58001] (1 + N * kNumBoxElement)
    int nk = kNumKpt * kKptDims;  // total keypoint values per detection
    decode(
        transposeDevice,
        decodeDevice,
        OUTPUT_CANDIDATES,
        kNumClass,
        nk,
        kConfThresh,
        kMaxNumOutputBbox,
        kNumBoxElement,
        stream
    );

    // cuda nms
    nms(decodeDevice, kNmsThresh, kMaxNumOutputBbox, kNumBoxElement, stream);

    CHECK(cudaMemcpyAsync(
        outputData,
        decodeDevice,
        (1 + kMaxNumOutputBbox * kNumBoxElement) * sizeof(float),
        cudaMemcpyDeviceToHost,
        stream
    ));
    cudaStreamSynchronize(stream);

    std::vector<Detection> vDetections;
    float *outPtr = outputData;
    for (int i = 0; i < 50; i++){
        std::cout << outPtr[i] << " ";
    }
    int count = std::min((int)outputData[0], kMaxNumOutputBbox);
    std::cout << "Total kept boxes after NMS: " << count << std::endl;
    for (int i = 0; i < count; i++){
        int pos      = 1 + i * kNumBoxElement;
        int keepFlag = (int)outputData[pos + 6];
        if (keepFlag == 1){
            Detection det{};
            memcpy(det.bbox, &outputData[pos], 4 * sizeof(float));
            det.conf    = outputData[pos + 4];
            det.classId = (int)outputData[pos + 5];
            memcpy(det.kpts, &outputData[pos + 7],
                   kNumKpt * kKptDims * sizeof(float));
            vDetections.emplace_back(det);
        }
    }

    for (size_t j = 0; j < vDetections.size(); j++){
        scale_bbox(img, vDetections[j].bbox);
        vDetections[j].vKpts = scale_kpt_coords(img, vDetections[j].kpts);
    }

    return vDetections;
}

void YoloDetector::draw_image(
    cv::Mat& img,
    std::vector<Detection>& inferResult,
    bool drawBbox,
    bool kptLine
){
    // draw inference result on image
    for (size_t j = 0; j < inferResult.size(); j++)
    {
        // draw bboxes
        if (drawBbox){
            cv::Scalar bboxColor(get_random_int(), get_random_int(), get_random_int());
            cv::Rect r(
                round(inferResult[j].bbox[0]),
                round(inferResult[j].bbox[1]),
                round(inferResult[j].bbox[2] - inferResult[j].bbox[0]),
                round(inferResult[j].bbox[3] - inferResult[j].bbox[1])
            );
            cv::rectangle(img, r, bboxColor, 2);

            std::string className = vClassNames[(int)inferResult[j].classId];
            std::string labelStr  = className + " " +
                                    std::to_string(inferResult[j].conf).substr(0, 4);

            cv::Size textSize = cv::getTextSize(
                labelStr, cv::FONT_HERSHEY_PLAIN, 1.2, 2, nullptr);
            cv::Point topLeft(r.x, r.y - textSize.height - 3);
            cv::Point bottomRight(r.x + textSize.width, r.y);
            cv::rectangle(img, topLeft, bottomRight, bboxColor, -1);
            cv::putText(img, labelStr, cv::Point(r.x, r.y - 2),
                        cv::FONT_HERSHEY_PLAIN, 1.2,
                        cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        }

        // draw key points
        int radius = std::min(img.rows, img.cols) / 100;
        cv::Scalar kptColor(get_random_int(), get_random_int(), get_random_int());

        std::vector<std::vector<float>> vScaledKpts = inferResult[j].vKpts;
        std::cout << "Number of keypoints: " << vScaledKpts.size() << std::endl;
        for (size_t k = 0; k < vScaledKpts.size(); k++){
            int   x    = (int)vScaledKpts[k][0];
            int   y    = (int)vScaledKpts[k][1];
            float conf = vScaledKpts[k][2];
            std::cout << "Keypoint " << k << ": (" << x << ", " << y << "), conf=" << conf << std::endl;
            if (x < 0 || x > img.cols || y < 0 || y > img.rows) continue;
            if (conf < 0.5f) continue;
            cv::circle(img, cv::Point(x, y), radius, kptColor, -1);
        }

        // draw skeleton between key points
        if (kptLine){
            int skeleton_width = std::min(img.rows, img.cols) / 300;
            cv::Scalar skeletonColor(get_random_int(), get_random_int(), get_random_int());
            for (size_t m = 0; m < skeleton.size(); m++){
                int kpt1_idx = skeleton[m][0] - 1;
                int kpt2_idx = skeleton[m][1] - 1;

                int   kpt1_x   = (int)vScaledKpts[kpt1_idx][0];
                int   kpt1_y   = (int)vScaledKpts[kpt1_idx][1];
                float kpt1_conf= vScaledKpts[kpt1_idx][2];

                int   kpt2_x   = (int)vScaledKpts[kpt2_idx][0];
                int   kpt2_y   = (int)vScaledKpts[kpt2_idx][1];
                float kpt2_conf= vScaledKpts[kpt2_idx][2];

                if (kpt1_conf < 0.5f || kpt2_conf < 0.5f) continue;
                if (kpt1_x > img.cols || kpt1_y > img.rows || kpt1_x < 0 || kpt1_y < 0) continue;
                if (kpt2_x > img.cols || kpt2_y > img.rows || kpt2_x < 0 || kpt2_y < 0) continue;

                cv::line(img,
                         cv::Point(kpt1_x, kpt1_y),
                         cv::Point(kpt2_x, kpt2_y),
                         skeletonColor, skeleton_width, cv::LINE_AA);
            }
        }
    }
}
