#include "utils.h"
#include "infer.h"


int run(char* imageDir){
    // get image file names for inferencing
    std::vector<std::string> file_names;
    if (read_files_in_dir(imageDir, file_names) < 0) {
        std::cout << "read_files_in_dir failed." << std::endl;
        return -1;
    }


    // create detector, and load engine plan
    std::string trtFile = "./yolo11s-pose.plan";
    YoloDetector detector(trtFile);

    // inference
    std::string imagePath = std::string(imageDir) + "/" + "yolo_tensorrt_test.png";
    cv::Mat img = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (img.empty()) return 1;
    auto start = std::chrono::system_clock::now();
    
    std::vector<Detection> res = detector.inference(img);
    for (auto r : res) {
        auto bbox = r.bbox;
        std::cout << "bbox: [" << bbox[0] << ", " << bbox[1] << ", " << bbox[2] << ", " << bbox[3] << "]" << std::endl;
        auto kpts = r.vKpts;
        for (auto k : kpts) {
            std::cout << "Keypoints: " ;
            for (int i = 0; i < k.size(); i++) {
                std::cout << k[i] << " ";
            }
            std::cout << std::endl;
        }
    }
    auto end = std::chrono::system_clock::now();
    int cost = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Image: " << "test" << " cost: " << cost << " ms."  << std::endl;
    // draw result on image
    std::cout << "===== Inference Result =====" << std::endl;
    YoloDetector::draw_image(img, res, true, false);
    std::cout << "============================" << std::endl;
    cv::imwrite("_yolo_tensorrt_test.png", img);

    return 0;
}


int main(int argc, char *argv[])
{
    if (argc != 2) {
        printf("This program need 1 argument\n");
        printf("Usage: ./main [image dir]\n");
        printf("Example: ./main ./images\n");
        return 1;
    }

    return run(argv[1]);\
}


