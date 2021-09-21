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

#include "detector.h"
#include "Logger.h"
#include "sweeper.h"
#include "UTM.h"
#include "motionDetector.h"

/** \defgroup Scanner_Module Scanner module
*
*  * The "Scanner module" is the calculation core of the app.
*  It includes the following classes:
*  -  Scanner,
*  -  Detector,
*  -  Logger,
*  -  SweeperGeometry::Sweeper,
*  -  MotionDetector
*
*  \code
*  #include "scanner.h"
*  \endcode
*/

using namespace cv;

struct Marker
{
    enum {DRAW, REMAIN, DELETE} action;
    Location pos;
};

/**
* \scanner_module \ingroup Scanner_Module
*
* \class Scanner
*
* \brief Manages the procedure of detecting objects and motions, and generates an online map
*
* - This class handles the following tasks:
*     -# Having camera info and IMU data, maps the camera FOV borders on the online map
*     -# As camera moves, calculates the area swept by camera FOV since beginning, using "sweeper" class member
*     -# Synchronizes the multi-thread sensor data (GPS, IMU, Camera) using "Logger" class member
*     -# Detects the desired objects (persons, cars, ...) using "detector" class member
*     -# Detects moving objects using "motionDetector" class member
*     -# Calculates each object's position on map based on its position in the image
*     -# Updates the existing map objects based on the last camera image
*
* - Call the function calcFov() to map the most recent FOV borders as well as previously swept area
* - Call the function scan() to detect objects OR motion, and then map the desired objects on the online map
*
* \sa class Sweeper, class Logger, class Detector, class MotionDetector
*/
class Scanner{

    float res, RAD, hva;
    float f, cx, cy;
    int max_dist, zone;
    bool initialInfoSet = false, isSouth = false;
    std::vector<Location> objectPoses;
    std::vector<Object> fovPoses;
    std::vector<Marker> markers;
    Location userLocation = {.x = -1.0, .y = -1.0}, firstLocation = {.x = -1.0, .y = -1.0};


    void camToMap(std::vector<Object>&, const ImageSet&);
    bool scaleVector(Eigen::VectorXd, Eigen::VectorXd&, double);
    void associate(const std::vector<Location>&);
    void gpsToUtm(double, double, double&, double&);
    void eulerToRotationMat(double, double, double, Eigen::Matrix3d&);
    void calcDirVec(float, float, Eigen::VectorXd&);
    void utmToGps(std::vector<Object>&);
    void setInitialInfo(ImageSet&);
    void imageToMap(double, double, double, double, double, double, std::vector<Object>&);
    void calcDistances(std::vector<Object>&);

public:

    Detector *detector;
    Logger *logger;
    SweeperGeometry::Sweeper *sweeper;
    MotionDetector *motionDetector;


    Scanner(std::string, std::string, DetectionMethod, bool, float, int);
    void setReferenceLoc(double, double, bool);
    bool scan(std::vector<Object>&, Mat&, Mat&, int, bool);
    bool scan(ImageSet&, Mat&, Mat&, std::vector<Object>&);
    bool calcFov(std::vector<Object>&);
    bool calcFov(std::vector<Object>&, ImageSet&);
};


#endif //ANDROID_SCANNER_SCANNER_H
