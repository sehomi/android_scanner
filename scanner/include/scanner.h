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
    enum {PERSON, CAR, FOV} type;
    Location pos;
};

class Scanner{

    float res, RAD, hva;
    float f, cx, cy;
    double lastProcessStamp = -1; double lastProcessImgSetStamp = -1;
    std::string assets_dir;
    int max_dist, zone;
    bool camInfoSet = false, isSouth = false, locationInitialized = false, useElev = false;
    Location initLoc;
    Grid<NasaGridSquare> *grid;
    std::vector<Location> objectPoses, fovPoses;
    std::vector<Marker> markers;

//    std::vector<int> DET_MODES{0, 1};

    void camToMap(std::vector<Rect>&, const ImageSet&, std::vector<Location>&);
    void toDirectionVector(std::vector<Rect>&, std::vector<Eigen::VectorXd>&);
    bool scaleVector(Eigen::VectorXd, Eigen::VectorXd&, double);
    void associate(std::vector<Location>&);
    void gpsToUtm(double, double, double&, double&);
    void eulerToRotationMat(double, double, double, Eigen::Matrix3d&);
    void calcDirVec(float, float, Eigen::VectorXd&);
    void utmToGps(std::vector<Eigen::VectorXd>, std::vector<Location>&);
    void setCamInfo(Mat&);
    void cropObjectImage(Mat&, std::vector<Rect>&, std::vector<Location>&, float);
    void calcDistanceFromHome(std::vector<Location>&);
    bool elevDiff(double, double, double&);
    void imageToMap(double, double, double, double, double, double, std::vector<Point2f>, std::vector<Location>&, std::vector<bool>&);

public:

    Detector *detector;
    Logger *logger;
    SweeperGeometry::Sweeper *sweeper;
    MotionDetector *motionDetector;

//    Scanner(std::string);
//    Scanner(std::string, float&, float&, float&, float&, int maxdist);
    Scanner(std::string, std::string, DetectionMethod, float, float, float, float, int);
    Scanner(std::string, std::string, DetectionMethod, int, float, int);
    void myFlip(Mat src);
    void myBlur(Mat src, float sigma);
    bool scan(std::vector<Location>&, Mat&, Mat&,int, bool);
    bool scan(ImageSet&, Mat&, std::vector<Location>&, Mat&, std::vector<Location>&, double);
    bool calcFov(std::vector<Location>&, std::vector<Location>&);
    bool calcFov(std::vector<Location>&, std::vector<Location>&, ImuSet&, ImageSet&);
    double elev();
    double elev(ImuSet &);
//    void readFromLog(std::string);
};


#endif //ANDROID_SCANNER_SCANNER_H
