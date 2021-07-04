//
// Created by a on 6/7/2021.
//

#ifndef ANDROID_SCANNER_SCANNER_H
#define ANDROID_SCANNER_SCANNER_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <android/log.h>
#include "GeographicLib/UTMUPS.hpp"
#include <math.h>

#include "detector.h"
#include "logger.h"


#define PI 3.141592

using namespace cv;
//using namespace std;

struct Marker
{
    enum {DRAW, REMAIN, DELETE} action;
    enum {PERSON, CAR, FOV} type;
    Eigen::VectorXd pos;
};

class Scanner{

    float f, cx, cy, res, RAD, beta;
    int max_dist;
    std::vector<Eigen::VectorXd> object_poses;
    std::vector<Marker> markers;

    void camToMap(std::vector<Rect>&, const ImageSet&, std::vector<Eigen::VectorXd>&);
    void toDirectionVector(std::vector<Rect>&, std::vector<Eigen::VectorXd>&);
    void scaleVector1(Eigen::VectorXd, Eigen::VectorXd&, double);
    bool scaleVector2(Eigen::VectorXd, Eigen::VectorXd&, double);
    void associate(std::vector<Eigen::VectorXd>&);
    Eigen::Quaternion<double> eulerToQuat(double, double, double);
    std::vector<double> gpsToUtm(double, double, double&, double&);
    void camToInertiaMat(double, double, double, double, double, double, Eigen::Matrix3d&);
    void calcDirVec(float, float, Eigen::VectorXd&);

public:

    Detector* detector;
    Logger* logger;

//    Scanner(std::string);
//    Scanner(std::string, float&, float&, float&, float&, int maxdist);
    Scanner(std::string, DetectionMethod, float , float , float , float , int);
    void myFlip(Mat src);
    void myBlur(Mat src, float sigma);
    void scan();
    void calcFov(std::vector<Eigen::VectorXd>&);
};


#endif //ANDROID_SCANNER_SCANNER_H
