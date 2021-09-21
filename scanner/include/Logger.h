

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

/**
  * \scanner_module \ingroup Scanner_Module
  * \struct Image
  * \brief Includes fields for an image and its corresponding time to capture
 */
struct Image
{
    Mat image = Mat::zeros(Size(480, 640),CV_8UC1);     /**< The image itself */
    double time = 0;                                        /**< The time in which image is captured */
};

/**
  * \scanner_module \ingroup Scanner_Module
  * \struct Location
  * \brief Includes full information fields for GPS and UTM location
 */
struct Location
{
    double lat = 0.0;   /**< GPS latitude */
    double lng = 0.0;   /**< GPS longitude */
    double alt = 0.0;   /**< GPS altitude */
    double time = 0;    /**< The time in which location data is received from sensor */
    double x = 0.0;     /**< UTM x */
    double y = 0.0;     /**< UTM y */
    int zone = 0;       /**< UTM zone number */
};

/**
  * \scanner_module \ingroup Scanner_Module
  * \struct Orientation
  * \brief Includes full information fields for orientation received by IMU sensor
 */
struct Orientation
{
    double roll = 0.0;      /**< Roll angle */
    double pitch = 0.0;     /**< Pitch angle */
    double azimuth = 0.0;   /**< Azimuth angle */
    double time = 0;        /**< The time in which orientation data is received from sensor */
};

/**
  * \scanner_module \ingroup Scanner_Module
  * \struct ImageSet
  * \brief Includes full information fields for a set of synchronized IMU, GPS, and Camera data
 */
struct ImageSet
{
    Mat image;              /**< The image captured by the camera */
    double lat = 0.0;       /**< GPS latitude corresponding to the image */
    double lng = 0.0;       /**< GPS longitude corresponding to the image */
    double alt = 0.0;       /**< GPS altitude corresponding to the image */
    double roll = 0.0;      /**< Roll angle corresponding to the image */
    double pitch = 0.0;     /**< Pitch angle corresponding to the image */
    double azimuth = 0.0;   /**< Azimuth angle corresponding to the image */
    double time = 0;        /**< The time in which image is captured by the camera */
};

/**
  * \scanner_module \ingroup Scanner_Module
  * \struct ImuSet
  * \brief Includes full information fields for a set of synchronized IMU and GPS data
 */
struct ImuSet
{
    double lat = 0.0;       /**< GPS latitude corresponding to the orientation */
    double lng = 0.0;       /**< GPS longitude corresponding to the orientation */
    double alt = 0.0;       /**< GPS altitude corresponding to the orientation */
    double roll = 0.0;      /**< Roll angle */
    double pitch = 0.0;     /**< Pitch angle */
    double azimuth = 0.0;   /**< Azimuth angle */
    double time = 0;        /**< The time in which the orientation is received by IMU */
};

/**
  * \scanner_module \ingroup Scanner_Module
  * \class Logger
  * \brief Synchronizes the multi-thread sensor data
  *
  * This class is responsible for synchronizing GPS data, IMU data and camera images in such a way that
  * the nearest location, orientation and image in terms of time to receive are assigned together in a
  * unique ImageSet structure instance. It also does this task for synchronizing GPS and IMU data without
  * image.
  * - This class handles the following tasks:
  *     -#  Buffers IMU data and GPS data as they are received
  *     -#  In one functionality, for each image, finds the nearest IMU and GPS data in terms of time
  *         to receive
  *     -#  In another functionality, for each IMU data, finds the nearest GPS data in terms of time
  *         to receive
  *     -#  If desired, saves the synchronized data in a specific directory within device memory
  *
  * Call the function getImageSet() to get an ImageSet instance with synchronized image, IMU data and GPS
  * data
  * Call the function getImuSet() to get an ImuSet instance with synchronized IMU data and GPS data
  * Call the function getImageSetFromLogger() to get an ImageSet instance from an existing pre-logged
  * directory
  *
  * \sa class Scanner, class Sweeper, class Detector, class MotionDetector
 */
class Logger {

    std::string logsDir;
    bool logMode;
    std::ofstream logFile;
    std::ifstream iLogFile;

    std::vector<Location> locationBuffer;
    std::vector<Orientation> orientationBuffer;
    int locBufLen = 5, ornBufLen = 40, counter;
    std::string prelogged_dir;

    void bufferLocation(Location);
    void bufferOrientation(Orientation);
    void writeImageSet(const ImageSet&);
    void readData(std::string, Mat&, ImuSet&);

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
    bool getImageSetFromLogger(ImageSet &, ImuSet &);
};

#endif