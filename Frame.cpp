#include "Frame.h"


Frame::Frame(cv::Mat mGray)
{
	mGray.copyTo(this->mGray);

	cv::Ptr<cv::FEATUREDETECTOR> algorithm = cv::FEATUREDETECTOR::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::FAST_SCORE, 31, 20);
	//cv::Ptr<cv::FEATUREDETECTOR> algorithm = cv::FEATUREDETECTOR::create();

	algorithm->detect(mGray, mKeypoint);
	if (mKeypoint.size() == 0) return;

	algorithm->compute(mGray, mKeypoint, mDescriptor);
	if (mDescriptor.empty()) return;
}


Frame::~Frame()
{
}
