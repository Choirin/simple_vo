#ifndef FEATUREDETECT_DECOMPOSER_H
#define FEATUREDETECT_DECOMPOSER_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <opencv2/highgui/highgui.hpp>

//#define DEBUG

#define FEATUREDETECTOR                  ORB
//#define FEATUREDETECTOR                  AKAZE
//#define FEATUREDETECTOR                  BRISK

#define MATCHER_THRESHOLD_DISTANCE       100
#define MATCHER_THRESHOLD_GOOD_RATIO     0.90

#define MATCHEDTHRESHOLD                 8
#define MATCH_UPDATETHRESHOLD            60
#define PARALLAXANGLE_THRESHOLD          cos(0.1 * CV_PI / 180)

#define KEYFRAMEPOINTS_TRHESHOLD         30
#define KEYFRAMEPOINTSLENGTH_THRESHOLD   28

#define KEYFRAME_MINIMUM_UPDATE_FRAME    1

class Decomposer {
private:
    cv::Mat K;
	cv::Mat distCoeffs;

	// Current Frame
	cv::Mat mFrame;
	std::vector<cv::KeyPoint> mKeypoint;
	cv::Mat mDescriptor;

	// Keyframe
    cv::Mat mKeyframe;
    cv::Mat mKeyframeDescriptor;
    std::vector<cv::KeyPoint> mKeyframeKeypoint;

	// Candidate Keyframe
	cv::Mat mCandidateFrame;
	cv::Mat mCandidateDescriptor;
	std::vector<cv::KeyPoint> mCandidateKeypoint;

	std::vector<cv::Mat> mKeypointPos;
	cv::Mat mMask;
    cv::Point2f center;
    float direction;
    bool bRecognized;

    bool bHasHomography;

    int imageWidth;
    float angleOfView;
    float pixelDistance;

    float mFrameEuler[3];
    float mCurrentEuler[3];
    float mMatchedEuler[3];

	cv::Mat mKeyframeR, mKeyframet;
	cv::Mat mCandidateR, mCandidatet;
	cv::Mat currentR, currentt;

    std::vector<cv::DMatch> mMatch;

    bool updateKeyframe;
	int framesAfterKeyframe;

    void cameraPoseFromHomography(const cv::Mat& H, cv::Mat& pose);
    void cameraPoseFromFundamental(const cv::Mat& F, cv::Mat& pose);

	cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
	void updatePose(cv::Mat R, cv::Mat t);

	float parallaxAngle(cv::Mat camera1, cv::Mat camera2, cv::Mat point);
	
public:
    Decomposer();

    int matchFeatures(cv::Mat mScene);
	int track(cv::Mat mScene);
	int insertKeyframe(void);
	int drawHomography(cv::Mat mScene);

    void setFrameEuler(float *Euler){
        float *ptr = Euler;
        mFrameEuler[0] = *(ptr++);
        mFrameEuler[1] = *(ptr++);
        mFrameEuler[2] = *(ptr++);
    }
    void setCurrentEuler(float *Euler){
        float *ptr = Euler;
        mCurrentEuler[0] = *(ptr++);
        mCurrentEuler[1] = *(ptr++);
        mCurrentEuler[2] = *(ptr++);
    }

	void updateKey(void) {
		updateKeyframe = true;
	}
	cv::Point2f getPosition2(void) {
		return cv::Point2f(currentt.at<double>(0, 0), currentt.at<double>(2, 0));
	}
	cv::Mat getPosition(void) {
		return currentt;
	}
	std::vector<cv::Mat> getPointCloud(void) {
		return mKeypointPos;
	}
};

#endif //FEATUREDETECT_DECOMPOSER_H
