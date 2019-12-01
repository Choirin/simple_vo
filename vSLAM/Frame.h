#pragma once
#ifndef INCLUDE_FRAME_H
#define INCLUDE_FRAME_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#ifdef ANDROID_NDK
#include <android/log.h>
#else
#include <opencv2/highgui/highgui.hpp>
#endif

#define FEATUREDETECTOR                  ORB
//#define FEATUREDETECTOR                  AKAZE
//#define FEATUREDETECTOR                  BRISK

#define MATCHEDTHRESHOLD             10

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
