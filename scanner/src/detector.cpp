
#include "detector.h"

Detector::Detector(std::string assetsDir, DetectionMethod dm, float conf, float nms)
{

    this->detectionMethod = dm;

    if (dm == DetectionMethod::YOLO_V3)
    {
        std::string model = "../../YOLOV3/cfg/yolo_v3.cfg";
        std::string config = "../../YOLOV3/weight/yolov3.weights";

        this->net = cv::dnn::readNetFromDarknet(model, config);
        this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    else if (dm == DetectionMethod::YOLO_TINY)
    {
        std::string model = "../../YOLOV3/cfg/yolov3-tiny.cfg";
        std::string config = "../../YOLOV3/weight/yolov3-tiny.weights";

        this->net = cv::dnn::readNetFromDarknet(model, config);
        this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    else if (dm == DetectionMethod::MN_SSD)
    {
        std::string model = assetsDir + "/MobileNetSSD_deploy.prototxt.txt";
        std::string config = assetsDir + "/MobileNetSSD_deploy.caffemodel";

        this->net = cv::dnn::readNetFromCaffe(model, config);
    }

    this->confidence = conf;
    this->nmsThreshold = nms;

}