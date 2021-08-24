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
    focalLengthSet = true;
}

void MotionDetector::detect(ImageSet &imgSt, cv::Mat &output, std::vector<Object> &objects, const std::vector<Object> &fov)
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

    generateMovingRects(imgSt.image, output, objects);
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

void MotionDetector::calcNormCoeffMat(const std::vector<Object> &fov, double lat, double lng, double alt, cv::Mat &xNormalizationCoeff, cv::Mat &yNormalizationCoeff)
{
    // This function calculates the normalization coefficient matrix
    double x, y;
    LatLonToUTMXY(lat, lng, 0, x, y);

    double v1_1 = fov[0].location.x-fov[1].location.x, v1_2 = fov[0].location.y-fov[1].location.y, v2_1 = fov[2].location.x-fov[1].location.x, v2_2 = fov[2].location.y-fov[1].location.y;
    double v1dotv2 = v1_1*v2_1 + v1_2*v2_2;
    double alpha = (PI/2) - acos(v1dotv2/(sqrt(pow(v1_1,2)+pow(v1_2,2))*sqrt(pow(v2_1,2)+pow(v2_2,2))));
    int rows = old_frame.rows, cols = old_frame.cols;
    for (int i=0; i<rows; i++) {
        double rowFirstX = fov[0].location.x + i*(fov[3].location.x - fov[0].location.x)/rows;
        double rowFirstY = fov[0].location.y + i*(fov[3].location.y - fov[0].location.y)/rows;
        double rowLastX = fov[1].location.x + i*(fov[2].location.x - fov[1].location.x)/rows;
        double rowLastY = fov[1].location.y + i*(fov[2].location.y - fov[1].location.y)/rows;
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

void MotionDetector::generateMovingRects(cv::Mat &input, cv::Mat &output, std::vector<Object> &objects)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(output, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0) );

    objects.clear();
    double area, areaRatio, imArea = output.rows * output.cols;
    for (auto & contour : contours) {
        area = cv::contourArea(contour);
        areaRatio = area / imArea;
        if (areaRatio < objectSizeUpLimit && areaRatio > objectSizeLowLimit)
        {
            Object obj;
            obj.box = cv::boundingRect(contour);
            obj.picture = input(obj.box);
            obj.type = Object::MOVING;
            objects.push_back(obj);
        }
    }
}

