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


class Logger {

    struct image
    {
        Mat image;
        float time;
    } img;
    struct location
    {
        double lat;
        double lng;
        float time;
    } loc;
    struct orientation
    {
        float roll;
        float pitch;
        float azimuth;
        float time;
    } orn;

    vector<location> locationBuffer;
    vector<orientation> orientationBuffer;
    int locBufLen = 5, ornBufLen = 40;

    void bufferLocation(location);
    void bufferOrientation(orientation);
    void setImageSet(image);

public:

    void setImage(Mat, float);
    void setLocation(float, float, float );
    void setOrientation(float, float, float, float);
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
