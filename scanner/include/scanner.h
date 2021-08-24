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
//#include "GeographicLib/UTMUPS.hpp"
#include <math.h>

#include "detector.h"
#include "Logger.h"
#include "sweeper.h"
#include "UTM.h"
#include "motionDetector.h"

using namespace cv;

struct Marker
{
    enum {DRAW, REMAIN, DELETE} action;
    Location pos;
};


class Scanner{

    float res, RAD, hva;
    float f, cx, cy;
    int max_dist, zone;
    bool initialInfoSet = false, isSouth = false;
    std::vector<Location> objectPoses;
    std::vector<Object> fovPoses;
    std::vector<Marker> markers;
    Location userLocation = {.x = -1.0, .y = -1.0}, firstLocation = {.x = -1.0, .y = -1.0};

//    void camToMap(std::vector<Rect>&, const ImageSet&, std::vector<Location>&);
//    void camToMap(std::vector<Object>&, const ImageSet&, std::vector<Location>&);
    void camToMap(std::vector<Object>&, const ImageSet&);
    void toDirectionVector(std::vector<Rect>&, std::vector<Eigen::VectorXd>&);
    bool scaleVector(Eigen::VectorXd, Eigen::VectorXd&, double);
    void associate(const std::vector<Location>&);
    void gpsToUtm(double, double, double&, double&);
    void eulerToRotationMat(double, double, double, Eigen::Matrix3d&);
    void calcDirVec(float, float, Eigen::VectorXd&);
//    void utmToGps(std::vector<Eigen::VectorXd>, std::vector<Location>&);    // before
    void utmToGps(std::vector<Object>&);                                 // after
    void setInitialInfo(ImageSet&);
//    void imageToMap(double, double, double, double, double, double, std::vector<Point2f>, std::vector<Location>&, std::vector<bool>&);  // before
    void imageToMap(double, double, double, double, double, double, std::vector<Object>&);                                            // after
    void calcDistances(std::vector<Object>&);

public:

    Detector *detector;
    Logger *logger;
    SweeperGeometry::Sweeper *sweeper;
    MotionDetector *motionDetector;

//    Scanner(std::string);
//    Scanner(std::string, float&, float&, float&, float&, int maxdist);
    Scanner(std::string, std::string, DetectionMethod, bool, float, int);
    void setReferenceLoc(double, double, bool);
//    bool scan(std::vector<Location>&, Mat&, Mat&,int, bool);
    bool scan(std::vector<Object>&, Mat&, Mat&, int, bool);
    //    bool scan(ImageSet&, Mat&, std::vector<Location>&, Mat&, std::vector<Location>&);
//    bool scan(ImageSet&, Mat&, std::vector<Location>&, Mat&);
    bool scan(ImageSet&, Mat&, Mat&, std::vector<Object>&);
//    bool calcFov(std::vector<Location>&, std::vector<Location>&);
    bool calcFov(std::vector<Object>&);
    //    bool calcFov(std::vector<Location>&, std::vector<Location>&, ImuSet&, ImageSet&);
    bool calcFov(std::vector<Object>&, ImageSet&);
};


#endif //ANDROID_SCANNER_SCANNER_H
