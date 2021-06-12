


#ifndef ANDROID_SCANNER_DETECTOR_H
#define ANDROID_SCANNER_DETECTOR_H

#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"

enum DetectionMethod{
	YOLO_V3,
	YOLO_TINY,
	MN_SSD
};

class Detector {

public:

    Detector(std::string, DetectionMethod, float, float);
	void detect(cv::Mat &, std::vector<cv::Rect> &);

private:
	cv::dnn::Net net;
	DetectionMethod detectionMethod;

	float confidence;
	float nmsThreshold;

	void yolov3PostProcess(cv::Mat&, const std::vector<cv::Mat> &, std::vector<cv::Rect> &);
	void ssdPostProcess(cv::Mat&, cv::Mat &, std::vector<cv::Rect> &);
};

#endif //ANDROID_SCANNER_DETECTOR_H