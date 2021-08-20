

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

#define PI 3.14159265358979


struct Image
{
    Mat image = Mat::zeros(Size(480, 640),CV_8UC1);
    double time = 0;
};

struct Location
{
    double lat = 0.0;
    double lng = 0.0;
    double alt = 0.0;
    double time = 0;
    double x = 0.0;
    double y = 0.0;
};

struct Orientation
{
    double roll = 0.0;
    double pitch = 0.0;
    double azimuth = 0.0;
    double time = 0;
};

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
    bool logMode;
    std::ofstream logFile;
    std::ifstream iLogFile;

    Location refLoc;

    std::vector<Location> locationBuffer;
    std::vector<Orientation> orientationBuffer;
    int locBufLen = 5, ornBufLen = 40, counter;
    std::string prelogged_dir;

    void bufferLocation(Location);
    void bufferOrientation(Orientation);
    void writeImageSet(const ImageSet&);

public:

    Image img;
    bool readFromLog;

    Logger(std::string, bool, bool, std::string);
    void setImage(Mat&, double);
    void setLocation(double, double, double, double);
    bool setOrientation(double, double, double, double);
    bool getImageSet(ImageSet&);
    bool getImuSet(ImuSet&);
    void disableLogMode();
    void enableLogMode();
    void readData(std::string, Mat&, ImuSet&);
    bool getImageSetFromLogger(ImageSet &, ImuSet &);
};

#endif