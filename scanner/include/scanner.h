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


//#define PI 3.14159265

using namespace cv;
//using namespace std;

struct Marker
{
    enum {DRAW, REMAIN, DELETE} action;
    enum {PERSON, CAR, FOV} type;
    Location pos;
};

class Scanner{

    float res, RAD, hva;
    float f, cx, cy;
    int max_dist, zone;
    bool camInfoSet = false, isSouth = false;
    std::vector<Location> objectPoses;
    std::vector<Marker> markers;

    void camToMap(std::vector<Rect>&, const ImageSet&, std::vector<Location>&);
    void toDirectionVector(std::vector<Rect>&, std::vector<Eigen::VectorXd>&);
    bool scaleVector(Eigen::VectorXd, Eigen::VectorXd&, double);
    void associate(const std::vector<Location>&);
    void gpsToUtm(double, double, double&, double&);
    void eulerToRotationMat(double, double, double, Eigen::Matrix3d&);
    void calcDirVec(float, float, Eigen::VectorXd&);
    void utmToGps(std::vector<Eigen::VectorXd>, std::vector<Location>&);
    void setCamInfo(Mat&);
    void imageToMap(double, double, double, double, double, double, std::vector<Point2f>, std::vector<Location>&, std::vector<bool>&);

public:

    Detector* detector;
    Logger* logger;
    SweeperGeometry::Sweeper* sweeper;

//    Scanner(std::string);
//    Scanner(std::string, float&, float&, float&, float&, int maxdist);
    Scanner(std::string, std::string, DetectionMethod, float, float, float, float, int);
    Scanner(std::string, std::string, DetectionMethod, bool, float, int);
    void myFlip(Mat src);
    void myBlur(Mat src, float sigma);
    bool scan();
    bool scan(ImageSet&, Mat&);
    bool calcFov(std::vector<Location>&);
    bool calcFov(std::vector<Location>&, std::vector<Location>&, ImuSet&, ImageSet&);
//    void readFromLog(std::string);
};


#endif //ANDROID_SCANNER_SCANNER_H
