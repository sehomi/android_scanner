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
#include <math.h>

#include "grid.h"
#include "nasagridsquare.h"
#include "utils.h"

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
    double lastProcessStamp = -1; double lastProcessImgSetStamp = -1;
    std::string assets_dir;
    int max_dist, zone;
    bool initialInfoSet = false, isSouth = false, useElev = false;
    Grid<NasaGridSquare> *grid;
    std::vector<Location> objectPoses;
    std::vector<Object> fovPoses;
    std::vector<Marker> markers;
    Location userLocation = {.x = -1.0, .y = -1.0}, firstLocation = {.x = -1.0, .y = -1.0};


    void camToMap(std::vector<Object>&, const ImageSet&);
    void toDirectionVector(std::vector<Rect>&, std::vector<Eigen::VectorXd>&);
    bool scaleVector(Eigen::VectorXd, Eigen::VectorXd&, double);
    void associate(std::vector<Location>&);
    void gpsToUtm(double, double, double&, double&);
    void eulerToRotationMat(double, double, double, Eigen::Matrix3d&);
    void calcDirVec(float, float, Eigen::VectorXd&);
    void utmToGps(std::vector<Object>&);
    void setInitialInfo(ImageSet&);
    bool elevDiff(double, double, double&);
    void imageToMap(double, double, double, double, double, double, std::vector<Object>&);
    void calcDistances(std::vector<Object>&);

public:

    Detector *detector;
    Logger *logger;
    SweeperGeometry::Sweeper *sweeper;
    MotionDetector *motionDetector;

    Scanner(std::string, std::string, DetectionMethod, int, float, int);
    bool scan(std::vector<Object>&, Mat&, Mat&, int, bool);
    bool scan(ImageSet&, Mat&, Mat&, std::vector<Object>&, double);
    bool calcFov(std::vector<Object>&);
    bool calcFov(std::vector<Object>&, ImageSet&);
};


#endif //ANDROID_SCANNER_SCANNER_H
