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
//#include <Eigen/Geometry>
//#include "Eigen/Core"

// TODO: It is much better that the MotionDetector class inherits Scanner in order to access its focal length and fov
class MotionDetector{

    cv::Mat old_frame;
    float hva, fl = 0.0, minimumDetectionSpeed = 0.75;
    bool focalLengthSet = false;

    void visualize(const cv::Mat&, const cv::Mat&, const cv::Mat&, cv::Mat&);
    void setFocalLength(int);
    void calcNormCoeffMat(const std::vector<Location>&, double, double, double, cv::Mat&, cv::Mat&);

public:

    MotionDetector(float);
    void detect(ImageSet&, cv::Mat&, std::vector<cv::Rect>&, std::vector<Location>);
};


#endif //ANDROID_SCANNER_SCANNER_H
