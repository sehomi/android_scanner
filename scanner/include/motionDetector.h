//
// Created by a on 6/7/2021.
//

#ifndef ANDROID_SCANNER_MOTIONDETECTOR_H
#define ANDROID_SCANNER_MOTIONDETECTOR_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <android/log.h>
//#include <math.h>


class MotionDetector{

    cv::Mat old_frame;

    void visualize(const cv::Mat&, cv::Mat&);

public:

    void detect(cv::Mat&, cv::Mat&);
};


#endif //ANDROID_SCANNER_SCANNER_H
