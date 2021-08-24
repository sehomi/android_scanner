


#ifndef ANDROID_SCANNER_DETECTOR_H
#define ANDROID_SCANNER_DETECTOR_H

#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "Logger.h"

enum DetectionMethod {
	YOLO_V3,
	YOLO_TINY,
	MN_SSD
};

struct Object
{
//	enum {DRAW, REMAIN, DELETE} action;
	enum {PERSON, CAR, FOV, SWEPT, MOVING} type;
	cv::Rect box;
	cv::Mat picture;
	double distance = -1;
	cv::Point2f center = cv::Point(0,0);
	bool show = true;
	Location location;
};

class Detector {

public:

    Detector(std::string, DetectionMethod, float, float);
//	void detect(cv::Mat&, std::vector<cv::Rect>&, std::vector<int>&);
	void detect(cv::Mat&, std::vector<Object>&);
//	void drawDetections(cv::Mat &, std::vector<cv::Rect> &);
	void drawDetections(cv::Mat &, std::vector<Object> &);

private:
	cv::dnn::Net net;
	DetectionMethod detectionMethod;

	float confidence;
	float nmsThreshold;

	std::string assets_dir;

//	void yolov3PostProcess(cv::Mat&, const std::vector<cv::Mat> &, std::vector<cv::Rect> &, std::vector<int> &);
//	void ssdPostProcess(cv::Mat&, cv::Mat &, std::vector<cv::Rect> &, std::vector<int>&);
	void yolov3PostProcess(cv::Mat&, const std::vector<cv::Mat> &, std::vector<Object> &);
	void ssdPostProcess(cv::Mat&, cv::Mat &, std::vector<Object> &);
	std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);
};

#endif //ANDROID_SCANNER_DETECTOR_H