//
// Created by a on 6/7/2021.
//

#ifndef ANDROID_SCANNER_SCANNER_H
#define ANDROID_SCANNER_SCANNER_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "detector.h"

using namespace cv;

class Scanner{

public:
    int x;
    Scanner(std::string);
    void myFlip(Mat src);
    void myBlur(Mat src, float sigma);
};


#endif //ANDROID_SCANNER_SCANNER_H
