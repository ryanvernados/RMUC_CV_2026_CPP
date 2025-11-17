#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>

int main() {
    std::string model_path = "./calibur/models/yolo11n-obb.onnx";

    cv::dnn::Net net = cv::dnn::readNetFromONNX(model_path);
    if (net.empty()) {
        std::cerr << "Failed to load model\n";
        return -1;
    }

    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat blob = cv::dnn::blobFromImage(
            frame, 1.0/255.0, cv::Size(640, 640),
            cv::Scalar(), true, false
        );

        net.setInput(blob);
        cv::Mat out = net.forward();

        // TODO: decode detections, draw boxes, etc.

        cv::imshow("test_yolo", frame);
        if (cv::waitKey(1) == 27) break; // ESC
    }
    return 0;
}
