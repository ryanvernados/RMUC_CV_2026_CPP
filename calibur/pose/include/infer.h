#ifndef INFER_H
#define INFER_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <NvInfer.h>

#include "public.h"
#include "types.h"
#include "config.h"

using namespace nvinfer1;

class YoloDetector
{
public:
    explicit YoloDetector(const std::string trtFile);
    ~YoloDetector();
    YoloDetector(const YoloDetector&) = delete;
    YoloDetector& operator=(const YoloDetector&) = delete;
    YoloDetector(YoloDetector&&) = default;
    YoloDetector& operator=(YoloDetector&&) = default;

    std::vector<Detection> inference(cv::Mat& img);

    static void draw_image(
        cv::Mat& img,
        std::vector<Detection>& inferResult,
        bool drawBbox = true,
        bool kptLine  = true
    );

private:
    void get_engine();

private:
    Logger              gLogger;
    std::string         trtFile_;

    ICudaEngine*        engine   = nullptr;
    IRuntime*           runtime  = nullptr;
    IExecutionContext*  context  = nullptr;

    cudaStream_t        stream   = nullptr;

    float*              outputData     = nullptr;
    std::vector<void*>  vBufferD;          // [input, output]
    float*              transposeDevice = nullptr;
    float*              decodeDevice    = nullptr;

    int                 OUTPUT_CANDIDATES = 0;  // 8400: 80*80 + 40*40 + 20*20
};

#endif  // INFER_H
