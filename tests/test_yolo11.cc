#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <vector>

struct KeyPoint {
    cv::Point2f point;
    float confidence;
};

struct PoseDetection {
    cv::Rect bbox;
    float confidence;
    std::vector<KeyPoint> keypoints;  // 17 keypoints for COCO format
};

// COCO 17 keypoint names
const std::vector<std::string> KEYPOINT_NAMES = {
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle"
};

// Skeleton connections for drawing
const std::vector<std::pair<int, int>> SKELETON = {
    {0, 1}, {0, 2}, {1, 3}, {2, 4},           // Head
    {5, 6}, {5, 7}, {7, 9}, {6, 8}, {8, 10},  // Arms
    {5, 11}, {6, 12}, {11, 12},               // Torso
    {11, 13}, {13, 15}, {12, 14}, {14, 16}    // Legs
};

std::vector<PoseDetection> postprocess(
    const cv::Mat& output, 
    float conf_threshold = 0.25,
    float iou_threshold = 0.45,
    const cv::Size& original_size = cv::Size(640, 640)
) {
    std::vector<PoseDetection> detections;
    
    // Output shape: [1, 56, 8400] for pose
    // 56 = 4 (bbox) + 1 (conf) + 51 (17 keypoints * 3)
    int num_channels = output.size[1];  // 56
    int num_anchors = output.size[2];   // 8400
    
    const float* data = (float*)output.data;
    
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<std::vector<KeyPoint>> all_keypoints;
    
    for (int i = 0; i < num_anchors; i++) {
        // Extract data for this anchor
        float cx = data[i];                          // center x
        float cy = data[num_anchors + i];            // center y
        float w = data[2 * num_anchors + i];         // width
        float h = data[3 * num_anchors + i];         // height
        float confidence = data[4 * num_anchors + i]; // confidence
        
        if (confidence < conf_threshold) continue;
        
        // Convert to corner format
        int left = static_cast<int>(cx - w / 2);
        int top = static_cast<int>(cy - h / 2);
        int width = static_cast<int>(w);
        int height = static_cast<int>(h);
        
        boxes.push_back(cv::Rect(left, top, width, height));
        confidences.push_back(confidence);
        
        // Extract keypoints (17 keypoints * 3 channels each)
        std::vector<KeyPoint> keypoints;
        for (int k = 0; k < 17; k++) {
            float kpt_x = data[(5 + k * 3) * num_anchors + i];
            float kpt_y = data[(5 + k * 3 + 1) * num_anchors + i];
            float kpt_conf = data[(5 + k * 3 + 2) * num_anchors + i];
            
            keypoints.push_back({cv::Point2f(kpt_x, kpt_y), kpt_conf});
        }
        all_keypoints.push_back(keypoints);
    }
    
    // Apply NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, iou_threshold, indices);
    
    // Create final detections
    for (int idx : indices) {
        PoseDetection det;
        det.bbox = boxes[idx];
        det.confidence = confidences[idx];
        det.keypoints = all_keypoints[idx];
        detections.push_back(det);
    }
    
    return detections;
}

void drawPose(cv::Mat& frame, const PoseDetection& detection) {
    // Draw bounding box
    cv::rectangle(frame, detection.bbox, cv::Scalar(0, 255, 0), 2);
    
    // Draw confidence
    std::string label = cv::format("%.2f", detection.confidence);
    cv::putText(frame, label, 
                cv::Point(detection.bbox.x, detection.bbox.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    
    // Draw keypoints
    for (size_t i = 0; i < detection.keypoints.size(); i++) {
        const auto& kpt = detection.keypoints[i];
        if (kpt.confidence > 0.5) {  // Only draw confident keypoints
            cv::circle(frame, kpt.point, 5, cv::Scalar(0, 0, 255), -1);
            
            // Optional: draw keypoint labels
            // cv::putText(frame, std::to_string(i), kpt.point, 
            //             cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
        }
    }
    
    // Draw skeleton
    for (const auto& connection : SKELETON) {
        const auto& kpt1 = detection.keypoints[connection.first];
        const auto& kpt2 = detection.keypoints[connection.second];
        
        if (kpt1.confidence > 0.5 && kpt2.confidence > 0.5) {
            cv::line(frame, kpt1.point, kpt2.point, cv::Scalar(255, 0, 0), 2);
        }
    }
}

int main() {
    std::string model_path = "./calibur/models/yolo11n-pose.onnx";

    // Load model
    cv::dnn::Net net = cv::dnn::readNetFromONNX(model_path);
    if (net.empty()) {
        std::cerr << "Failed to load model\n";
        return -1;
    }

    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    
    // For CUDA (if available)
    // net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    // net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::Size original_size = frame.size();
        
        // Preprocess
        cv::Mat blob = cv::dnn::blobFromImage(
            frame, 1.0/255.0, cv::Size(640, 640),
            cv::Scalar(), true, false
        );

        // Inference
        net.setInput(blob);
        cv::Mat output = net.forward();
        
        // Print output shape (debug)
        std::cout << "Output shape: ";
        for (int i = 0; i < output.dims; i++) {
            std::cout << output.size[i] << " ";
        }
        std::cout << std::endl;

        // Post-process
        auto detections = postprocess(output, 0.25, 0.45, original_size);
        
        // Scale keypoints back to original image size
        float scale_x = (float)original_size.width / 640.0f;
        float scale_y = (float)original_size.height / 640.0f;
        
        for (auto& det : detections) {
            det.bbox.x *= scale_x;
            det.bbox.y *= scale_y;
            det.bbox.width *= scale_x;
            det.bbox.height *= scale_y;
            
            for (auto& kpt : det.keypoints) {
                kpt.point.x *= scale_x;
                kpt.point.y *= scale_y;
            }
            
            drawPose(frame, det);
        }

        // Display FPS
        static auto start = std::chrono::high_resolution_clock::now();
        auto end = std::chrono::high_resolution_clock::now();
        float fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        start = end;
        
        cv::putText(frame, cv::format("FPS: %.1f", fps), 
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                    1, cv::Scalar(0, 255, 0), 2);

        cv::imshow("YOLO11 Pose", frame);
        if (cv::waitKey(1) == 27) break; // ESC to exit
    }
    
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
