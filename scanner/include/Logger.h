#ifndef ANDROID_SCANNER_LOGGER_H
#define ANDROID_SCANNER_LOGGER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include "time.h"
#include <android/log.h>
#include <fstream>

using namespace cv;
//using namespace std;

#define PI 3.14159265

struct Image
{
    Mat image = Mat::zeros(Size(480, 640),CV_8UC1);
    float time = 0;
} ;

struct Location
{
    double lat = 0.0;
    double lng = 0.0;
    double alt = 0.0;
    double time = 0;
} ;

struct Orientation
{
    double roll = 0.0;
    double pitch = 0.0;
    double azimuth = 0.0;
    double time = 0;
} ;

struct ImageSet
{
    Mat image;
    double lat;
    double lng;
    double alt;
    double roll;
    double pitch;
    double azimuth;
    double time;
};

struct ImuSet
{
    double lat;
    double lng;
    double alt;
    double roll;
    double pitch;
    double azimuth;
    double time;
};

class Logger {

    std::string logsDir;

    Location refLoc;
//    Orientation orn;
    ImageSet imgSet;
//    ImuSet imuSet;

    std::vector<Location> locationBuffer;
    std::vector<Orientation> orientationBuffer;
    int locBufLen = 5, ornBufLen = 40, counter;

    bool logMode = true;

    std::ofstream logFile;

    void bufferLocation(Location);
    void bufferOrientation(Orientation);
    void writeImageSet(const ImageSet&);

//    void setImageSet(image);

public:

    Image img;

    Logger(std::string);
    void setImage(Mat&, float);
    void setLocation(double, double, double, float);
    void setOrientation(double, double, double, float);
    bool getImageSet(ImageSet&);
    bool getImuSet(ImuSet&);
    void disableLogMode();
    void enableLogMode();
};

#endif //ANDROID_SCANNER_SCANNER_H