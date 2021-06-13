//
// Created by a on 6/7/2021.
//

#include "scanner.h"
//#include <opencv2/imgproc.hpp>

Scanner::Scanner(std::string assetsDir) {
    detector = new Detector(assetsDir, DetectionMethod::MN_SSD, 0.4, 0.4);
    return;
}

void Scanner::myFlip(Mat src) {
    flip(src, src, 0);
}

void Scanner::myBlur(Mat src, float sigma) {
    GaussianBlur(src, src, Size(), sigma);
}
