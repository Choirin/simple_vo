#pragma once
#ifndef INCLUDE_FRAME_H
#define INCLUDE_FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>
#ifdef ANDROID_NDK
#include <android/log.h>
#endif

#define FEATUREDETECTOR ORB
//#define FEATUREDETECTOR AKAZE
//#define FEATUREDETECTOR BRISK

#define MATCHEDTHRESHOLD 10

class Frame
{
private:
    bool bObject;
    cv::Mat K;
    cv::Mat distCoeffs;

    cv::Mat mGray;
    std::vector<cv::KeyPoint> mKeypoint;
    cv::Mat mDescriptor;

    // Camera pose
    cv::Mat R;
    cv::Mat t;

public:
    Frame(cv::Mat mGray);
    ~Frame();
};

#endif //INCLUDE_FRAME_H
