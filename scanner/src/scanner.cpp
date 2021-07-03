//
// Created by a on 6/7/2021.
//

#include "scanner.h"
//#include <opencv2/imgproc.hpp>

Scanner::Scanner(std::string assetsDir, DetectionMethod dm) {

    if (dm == MN_SSD)
        detector = new Detector(assetsDir, DetectionMethod::MN_SSD, 0.4, 0.4);
    else if (dm == YOLO_V3)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_V3, 0.4, 0.4);
    else if (dm == YOLO_TINY)
        detector = new Detector(assetsDir, DetectionMethod::YOLO_TINY, 0.4, 0.4);

    return;
}

void Scanner::myFlip(Mat src) {
    flip(src, src, 0);
}

void Scanner::myBlur(Mat src, float sigma) {
    GaussianBlur(src, src, Size(), sigma);
}
