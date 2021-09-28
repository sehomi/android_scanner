


#ifndef ANDROID_SCANNER_DETECTOR_H
#define ANDROID_SCANNER_DETECTOR_H

#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "Logger.h"

/**
  * \enum DetectionMethod
  * \brief Different objects detection methods: Yolo-v3, Tiny Yolo, MobileNet SSD
 */
enum DetectionMethod {
	YOLO_V3,
	YOLO_TINY,
	MN_SSD
};

/**
  * \scanner_module \ingroup Scanner_Module
  * \struct Object
  * \brief Includes a set of information fields that may be used for different mapped objects
  *
  * This structure is widely used as data storage unit for calculations done on various types of objects
  * such as camera corner point locations, swept point locations, detected objects within image and moving
  * objects. Indeed, every single object which is mapped to the UI online map is treated as an Object
  * instance
 */
struct Object
{
	// TODO: integrate actions into objects visualization
    /**
    * \enum ObjectAction
    * \brief Determines whether an object must be drawn, deleted or remained on the UI online map
    */
	enum ObjectAction
	        {
	        REMAIN,
	        ADD,
	        UPDATE
	        };
    ObjectAction action = REMAIN;

    /**
    * \enum ObjectType
    * \brief Determines whether type of an Object instance which is to be mapped on the UI online map
    */
	enum ObjectType
	        {
	        PERSON,
	        CAR,
	        FOV,
	        SWEPT,
	        MOVING
	        } type;

	int lastIdx = -1;
    cv::Rect box;								/**< The object's bounding box in the image */
    cv::Mat picture;							/**< The object's cropped image */
    double distance = -1;						/**< The object's distance to the reference point */
    cv::Point2f center = cv::Point(0,0);	/**< Center of the area related to the object in the image */
    bool show = true;							/**< The object's permission to be drawn on map */
    Location location;							/**< The object's corresponding location */
};

/**
  * \scanner_module \ingroup Scanner_Module
  *
  * \class Detector
  *
  * \brief Receiving an image at the input, detects desired objects within, using the desired detection method
  *
  * This class handles the following tasks:
  * 1 - Detects the desired objects based on the detection method (Yolo-v3, Tiny Yolo or MobileNet SSD)
  *     which is determined once at first
  * 2 - Saves a bounding box along with type(person, car, ...) for each objects within the Object structure
  * 3 - Draws a bounding box around each detected object on the input image
  *
  * Call the function detect() to detect the existing objects within the input image and get the additional
  * details inside the objects list in output
  * Call the function drawDetections() to draw bounding boxes around the detected objects on the input image
  *
  * \sa class Scanner, class Sweeper, class Logger, class MotionDetector
 */
class Detector {

public:

    Detector(std::string, DetectionMethod, float, float);
    void detect(cv::Mat&, std::vector<Object>&);
	void drawDetections(cv::Mat &, std::vector<Object> &);

private:
	cv::dnn::Net net;
	DetectionMethod detectionMethod;

	float confidence;
	float nmsThreshold;

	std::string assets_dir;

	void yolov3PostProcess(cv::Mat&, const std::vector<cv::Mat> &, std::vector<Object> &);
	void ssdPostProcess(cv::Mat&, cv::Mat &, std::vector<Object> &);
	std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);
};

#endif //ANDROID_SCANNER_DETECTOR_H