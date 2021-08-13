//
// Created by a on 6/7/2021.
//

#include "motionDetector.h"


MotionDetector::MotionDetector(float hva_)
{
    hva = hva_;
}

void MotionDetector::setFocalLength(int w)
{
    fl = (float) (0.5 * w * (1.0 / tan((hva/2.0)*PI/180)));
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----2", "md--v1: %s", std::to_string(fl).c_str());
    focalLengthSet = true;
}

void MotionDetector::detect(ImageSet &imgSt, cv::Mat &output, std::vector<cv::Rect> &rects, std::vector<Location> fov)
{
    if (old_frame.empty())
    {
        cv::cvtColor(imgSt.image, old_frame, cv::COLOR_RGB2GRAY);
        output = cv::Mat::zeros(imgSt.image.rows, imgSt.image.cols, CV_8UC3);

        if (!focalLengthSet){
            setFocalLength(imgSt.image.rows);
        }

        return;
    }

    cv::Mat frame = imgSt.image.clone();

    cv::Mat new_frame, flow(old_frame.size(), CV_32FC2);
    cv::cvtColor(frame, new_frame, cv::COLOR_BGR2GRAY);

    cv::calcOpticalFlowFarneback(old_frame, new_frame, flow,  0.5, 3, 15, 3, 5, 1.2, 0);
    cv::cvtColor(frame, old_frame, cv::COLOR_RGB2GRAY);

    cv::Mat xNormalizationCoeff(old_frame.size(), CV_64FC1), yNormalizationCoeff(old_frame.size(), CV_64FC1);
    // TODO: Using aircraft velocity data, this function must be called once every time the camera is fixed to detect motions, not real-time!
    calcNormCoeffMat(fov, imgSt.lat, imgSt.lng, imgSt.alt, xNormalizationCoeff, yNormalizationCoeff);

    visualize(flow, xNormalizationCoeff, yNormalizationCoeff, output);
}

void MotionDetector::visualize(const cv::Mat &flow, const cv::Mat &xNormalizationCoeff, const cv::Mat &yNormalizationCoeff, cv::Mat &output)
{
    cv::Mat flow_parts[2], flow_parts_d[2], magnitude, angle;
    cv::split(flow, flow_parts);
    flow_parts[0].convertTo(flow_parts_d[0], CV_64F);
    flow_parts[1].convertTo(flow_parts_d[1], CV_64F);

    cv::Mat metricFlowX(old_frame.size(), CV_64FC1), metricFlowY(old_frame.size(), CV_64FC1);

    metricFlowX = flow_parts_d[0].mul(xNormalizationCoeff);
    metricFlowY = flow_parts_d[1].mul(yNormalizationCoeff);

    cv::cartToPolar(metricFlowX, metricFlowY, magnitude, angle, true);
    cv::threshold(magnitude, output, minimumDetectionSpeed, 255, THRESH_BINARY);
    output.convertTo(output, CV_8U);
}

void MotionDetector::calcNormCoeffMat(const std::vector<Location> &fov, double lat, double lng, double alt, cv::Mat &xNormalizationCoeff, cv::Mat &yNormalizationCoeff)
{
    // This function calculates the normalization coefficient matrix
    double x, y;
    LatLonToUTMXY(lat, lng, 0, x, y);

    double v1_1 = fov[0].x-fov[1].x, v1_2 = fov[0].y-fov[1].y, v2_1 = fov[2].x-fov[1].x, v2_2 = fov[2].y-fov[1].y;
    double v1dotv2 = v1_1*v2_1 + v1_2*v2_2;
    double alpha = (PI/2) - acos(v1dotv2/(sqrt(pow(v1_1,2)+pow(v1_2,2))*sqrt(pow(v2_1,2)+pow(v2_2,2))));
//    __android_log_print(ANDROID_LOG_VERBOSE, "android_scanner----2", "md-v1: %s", "one2");
    int rows = old_frame.rows, cols = old_frame.cols;
    for (int i=0; i<rows; i++) {
        double rowFirstX = fov[0].x + i*(fov[3].x - fov[0].x)/rows;
        double rowFirstY = fov[0].y + i*(fov[3].y - fov[0].y)/rows;
        double rowLastX = fov[1].x + i*(fov[2].x - fov[1].x)/rows;
        double rowLastY = fov[1].y + i*(fov[2].y - fov[1].y)/rows;
        for (int j=0; j<cols; j++) {
            double X = rowFirstX + j*(rowLastX - rowFirstX)/cols;
            double Y = rowFirstY + j*(rowLastY - rowFirstY)/cols;
            double h = sqrt(pow(((double)j-((double)cols/2)),2) + pow(((double)i-((double)rows/2)),2) + pow(fl,2));
            double xCoeff = sqrt(pow(x-X, 1)+pow(y-Y, 2)+pow(alt,2))/h;
            xNormalizationCoeff.at<double>(i,j) = xCoeff;
            yNormalizationCoeff.at<double>(i,j) = xCoeff/cos(((double)j/((double)cols/2))*alpha);
        }
    }
}

