//
// Created by a on 6/7/2021.
//

#ifndef ANDROID_SCANNER_MOTIONDETECTOR_H
#define ANDROID_SCANNER_MOTIONDETECTOR_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <android/log.h>
#include "Logger.h"
#include "UTM.h"
#include <math.h>
#include "detector.h"

// TODO: It is much better that the MotionDetector class inherits Scanner in order to access its focal length and fov

/**
  * \scanner_module \ingroup Scanner_Module
  * \class MotionDetector
  * \brief Receiving an image at input, detects desired moving objects within, using dense optical flow approach
  *
  * The functionality of this class is based on a main assumption: The camera is in a fixed position. Otherwise,
  * the relative motion of the camera with respect to the surroundings leads to widely fake motion detections.
  * This class handles the following tasks:
  * 1 - Detects the desired objects based on the dense optical flow method
  * 2 - Normalizes the pixel velocity image so that an image with each pixel showing a metric velocity of
  *     the corresponding point on the ground is generated
  * 3 - Generates a motion map; A gray scale image in which the brighter a pixel is, the faster the corresponding
  *     point on the ground moves
  * 4 - Provides a list of moving objects in the output, containing a bounding box for each
  *
  * Call the function detect() to detect the moving objects within the input image and get the additional
  * details inside the objects list in output
  *
  * \sa class Scanner, class Sweeper, class Logger, class MotionDetector
 */
class MotionDetector{

    cv::Mat old_frame;
    float hva, fl = 0.0, minimumDetectionSpeed = 1.0, objectSizeUpLimit = 0.25, objectSizeLowLimit = 0.002;
    bool focalLengthSet = false, active = false;
    float objMaxSpeed = 3.0;

    void visualize(const cv::Mat&, const cv::Mat&, const cv::Mat&, cv::Mat&, cv::Mat&, std::vector<Object>&, const std::vector<Object>&, double);
    void setFocalLength(int);
    void calcNormCoeffMat(const std::vector<Object>&, double, double, double, cv::Mat&, cv::Mat&, double&);
    void generateMovingRects(cv::Mat&, cv::Mat&, std::vector<Object>&, const cv::Mat&, const cv::Mat&, const std::vector<Object>&, double, cv::Mat&);           //mm//
    void metricNormalize(Mat &);
    void saturateBox(int, int, cv::Rect&);
    void calcObjectsVelocities(Object&, const std::vector<Object>&, double, double, double);
    double calcTwoVectorsAngle(double, double, double, double);

public:

    MotionDetector(float);
    void detect(ImageSet&, cv::Mat&, std::vector<Object>&, const std::vector<Object>&, bool);
};

#endif //ANDROID_SCANNER_MOTIONDETECTOR_H
