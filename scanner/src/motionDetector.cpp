//
// Created by a on 6/7/2021.
//

#include "motionDetector.h"


void MotionDetector::detect(cv::Mat &frame, cv::Mat &output)
{
    if (old_frame.empty())
    {
        cv::cvtColor(frame, old_frame, cv::COLOR_RGB2GRAY);
        return;
    }

    cv::Mat new_frame, flow(old_frame.size(), CV_32FC2);
    cv::cvtColor(frame, new_frame, cv::COLOR_BGR2GRAY);

    cv::calcOpticalFlowFarneback(old_frame, new_frame, flow,  0.5, 3, 15, 3, 5, 1.2, 0);
    cv::cvtColor(frame, old_frame, cv::COLOR_RGB2GRAY);

    visualize(flow, output);
}

void MotionDetector::visualize(const cv::Mat &flow, cv::Mat &output_bgr)
{
    // visualization
    cv::Mat flow_parts[2], magnitude, angle, magn_norm, _hsv[3], hsv, hsv8;
    cv::split(flow, flow_parts);

    cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
    cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));

    //build hsv image
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    cv::merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    cv::cvtColor(hsv8, output_bgr, cv::COLOR_HSV2BGR);
}