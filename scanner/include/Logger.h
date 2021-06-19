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

using namespace cv;
using namespace std;

struct image
{
    Mat image = Mat::zeros(Size(480, 640),CV_8UC1);
    float time = 0;
} ;
struct location
{
    double lat = 0.0;
    double lng = 0.0;
    double time = 0;
} ;
struct orientation
{
    double roll = 0.0;
    double pitch = 0.0;
    double azimuth = 0.0;
    double time = 0;
} ;

class Logger {

    image img;
    location loc;
    orientation orn;

    vector<location> locationBuffer;
    vector<orientation> orientationBuffer;
    int locBufLen = 5, ornBufLen = 40;

    void bufferLocation(location);
    void bufferOrientation(orientation);
    void setImageSet(image);

public:

    void setImage(Mat, float);
    void setLocation(double, double, float );
    void setOrientation(double, double, double, float);
    struct imageSet
    {
        Mat image;
        double lat;
        double lng;
        float roll;
        float pitch;
        float azimuth;
        float time;
    } imgSet;

};
