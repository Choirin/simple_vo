//
// Created by kohei on 2016/03/02.
//

#include "Decomposer.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifdef ANDROID_NDK
#include <android/log.h>
#else
#include <iostream>
#include <fstream>
#include <iomanip>
#include <windows.h>
#endif

//#define UNDISTORTION

Decomposer::Decomposer() {
    // Initialize flags
    bRecognized = false;
    bHasHomography = false;

    direction = 0;

    imageWidth = 720;

    updateKeyframe = true;
	framesAfterKeyframe = 0;

#define KITTI
#ifdef IPHONE
    K = (cv::Mat_<double>(3,3) << 691.0, 0.0, 360.0, 0.0, 691.0, 270.0, 0.0, 0.0, 1.0);
	K = K * imageWidth / 720;
	distCoeffs = (cv::Mat_<double>(4, 1) << 0.006637, -0.052151, 0.002196, -0.000496);
#endif
#ifdef XPERIAZ3
	K = (cv::Mat_<double>(3, 3) << 988.833, 0.0, 633.399, 0.0, 990.107, 346.812, 0.0, 0.0, 1.0);
	K = K * imageWidth / 1280;
	distCoeffs = (cv::Mat_<double>(1, 4) << 0.006637, -0.052151, 0.002196, -0.000496);
#endif
#ifdef ACTIONCAM
	K = (cv::Mat_<double>(3, 3) << 21.8 * 1920 / 36, 0.0, 960.0, 0.0, 21.8 * 1920 / 36, 540.0, 0.0, 0.0, 1.0);
	K = K * imageWidth / 1920;
#endif
#ifdef KITTI
	imageWidth = 1242;
	K = (cv::Mat_<double>(3, 3) << 721.5377, 0.0, 609.5593, 0.0, 721.5377, 172.854, 0.0, 0.0, 1.0);
	K = K * imageWidth / 1242;
	distCoeffs = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
#endif
	K.at<double>(2, 2) = 1.0;
#ifndef ANDROID_NDK
	std::cout << "K=" << K << std::endl;
	std::cout << "distortion=" << distCoeffs << std::endl;
#endif
	currentR = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
	currentt = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
    return;
}

cv::Vec3f Decomposer::rotationMatrixToEulerAngles(cv::Mat &R) {
	float sy = sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) + R.at<double>(0, 0) * R.at<double>(0, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular) {
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	} else {
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return cv::Vec3f(x, y, z);
}

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
	cv::Mat euler(3, 1, CV_64F);

	double m00 = rotationMatrix.at<double>(0, 0);
	double m02 = rotationMatrix.at<double>(0, 2);
	double m10 = rotationMatrix.at<double>(1, 0);
	double m11 = rotationMatrix.at<double>(1, 1);
	double m12 = rotationMatrix.at<double>(1, 2);
	double m20 = rotationMatrix.at<double>(2, 0);
	double m22 = rotationMatrix.at<double>(2, 2);

	double x, y, z;

	// Assuming the angles are in radians.
	if (m10 > 0.998) { // singularity at north pole
		x = 0;
		y = CV_PI / 2;
		z = atan2(m02, m22);
	}
	else if (m10 < -0.998) { // singularity at south pole
		x = 0;
		y = -CV_PI / 2;
		z = atan2(m02, m22);
	}
	else
	{
		x = atan2(-m12, m11);
		y = asin(m10);
		z = atan2(-m20, m00);
	}

	euler.at<double>(0) = x;
	euler.at<double>(1) = y;
	euler.at<double>(2) = z;

	return euler;
}

void Decomposer::updatePose(cv::Mat R, cv::Mat t) {
	/*
	cv::Mat P = cv::Mat::eye(4, 4, R.type());
	P(cv::Range(0, 3), cv::Range(0, 3)) = R * 1.0;
	P(cv::Range(0, 3), cv::Range(3, 4)) = t * 1.0;
	*/

	double t1 = t.at<double>(0, 0);
	double t2 = t.at<double>(1, 0);
	double t3 = t.at<double>(2, 0);
	//t /= sqrt(t1 * t1 + t2 * t2 + t3 * t3);
	double scale = 1.0;
#if 0
	currentR = R * currentR;
	currentt = currentt + scale * (currentR * t);
#else
	currentR = R.t() * currentR;
	currentt = currentt - scale * (currentR * t);
#endif

	//std::cout << "prvpos=" << currentt.t() << std::endl;
	//currentt = P * currentt;
	//std::cout << "P=" << P << std::endl;
	std::cout << "pos=" << currentt.t() << ", " << rot2euler(currentR).t() << std::endl;
}


int Decomposer::track(cv::Mat mScene) {
	bRecognized = false;

	if (0 == mScene.size().width) return 0;
	// グレイスケール化
	cvtColor(mScene, mScene, cv::COLOR_BGR2GRAY);
	// INTER_LINER（バイリニア補間）でのサイズ変更
	cv::resize(mScene, mFrame, cv::Size(imageWidth, mScene.size().height * imageWidth / mScene.size().width), 0, 0);
	//cv::undistort(mScene, mFrame, K, distCoeffs);
#ifdef ANDROID_NDK
	__android_log_print(ANDROID_LOG_DEBUG, "Decomposer:", "matchSize: %d, %d", mFrame.size().width, mFrame.size().height);
#endif

	// 特徴量設定
	cv::Ptr<cv::FEATUREDETECTOR> algorithm = cv::FEATUREDETECTOR::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::FAST_SCORE, 31, 20);
	//cv::Ptr<cv::FEATUREDETECTOR> algorithm = cv::FEATUREDETECTOR::create();

	std::vector<cv::KeyPoint> keypoint;
	cv::Mat descriptor;

	// 特徴点抽出
	algorithm->detect(mFrame, keypoint);
	if (keypoint.size() == 0)return 0;

	// 特徴記述
	algorithm->compute(mFrame, keypoint, descriptor);
	if (keypoint.size() == 0)return 0;

	mKeypoint = keypoint;
	mDescriptor = descriptor;

	cv::namedWindow("mKeyframe", cv::WINDOW_AUTOSIZE);
	cv::imshow("mKeyframe", mKeyframe);

	framesAfterKeyframe++;
	//cvWaitKey(0);

	std::vector<std::vector<cv::DMatch> > knnmatch_points;
	cv::BFMatcher matcher(cv::NORM_HAMMING);
	matcher.knnMatch(mKeyframeDescriptor, mDescriptor, knnmatch_points, 2);

	//対応点を絞る
	std::vector<cv::DMatch> match;
	//KeyPoint -> Point2d
	std::vector<cv::Point2f> match_point1, match_point_undistort1;
	std::vector<cv::Point2f> match_point2, match_point_undistort2;
	std::vector<cv::Point3f> keypointPos;
	for (size_t i = 0; i < knnmatch_points.size(); ++i) {
		double distance1 = knnmatch_points[i][0].distance;
		double distance2 = knnmatch_points[i][1].distance;
#if 0
		std::cout << "dist=" << distance1 << std::endl;
#endif

		//第二候補点から距離値が離れている点のみ抽出（いい点だけ残す）
		if ((distance1 < MATCHER_THRESHOLD_DISTANCE) && (distance1 <= distance2 * MATCHER_THRESHOLD_GOOD_RATIO)) {
			// キーフレーム中のキーポイントになっている点のみを残す
			if (mKeypointPos[knnmatch_points[i][0].queryIdx].empty()) continue;
			match.push_back(knnmatch_points[i][0]);
			match_point1.push_back(mKeyframeKeypoint[knnmatch_points[i][0].queryIdx].pt);
			match_point2.push_back(mKeypoint[knnmatch_points[i][0].trainIdx].pt);
			cv::Point3f position(mKeypointPos[knnmatch_points[i][0].queryIdx].at<float>(0, 0), mKeypointPos[knnmatch_points[i][0].queryIdx].at<float>(1, 0), mKeypointPos[knnmatch_points[i][0].queryIdx].at<float>(2, 0));
			keypointPos.push_back(position);

#if 0
			std::cout << "position" << position << std::endl;
			std::cout << "match_point1" << mKeyframeKeypoint[knnmatch_points[i][0].queryIdx].pt << std::endl;
			std::cout << "mKe" << mKeypoint[knnmatch_points[i][0].trainIdx].pt << std::endl;
#endif
		}
	}
	//#define UNDISTORTION
#ifdef UNDISTORTION
	std::cout << match_point1.size() << std::endl;
	cv::undistortPoints(match_point1, match_point_undistort1, K, distCoeffs);
	std::cout << match_point_undistort1.size() << std::endl;
	std::cout << match_point1[1] << ", " << match_point_undistort1[1] << std::endl;
	cv::undistortPoints(match_point2, match_point_undistort2, K, distCoeffs);
#else
	match_point_undistort1 = match_point1;
	match_point_undistort2 = match_point2;
#endif
#ifdef ANDROID_NDK
	__android_log_print(ANDROID_LOG_DEBUG, "Decomposer:", "match.size:%d", match.size());
#endif

	// 十分な対応点がある
	std::cout << "match.size():" << match.size() << std::endl;
	//cvWaitKey(0);
	if (match.size() > MATCHEDTHRESHOLD) {
		cv::Mat E, R, t, inliers;
#ifndef ANDROID_NDK
		long int start = GetTickCount();
#endif
		cv::Mat rvec;
#ifdef UNDISTORTION
		cv::Mat K = cv::Mat::eye(3, 3, this->K.type());
		cv::Mat distCoeffs = cv::Mat::zeros(1, 4, this->distCoeffs.type());
#endif
		rvec = rot2euler(currentR);
		t = currentt;
		cv::solvePnPRansac(keypointPos, match_point_undistort2, K, distCoeffs, rvec, t, false, 100, 8.0f, 0.95, inliers, cv::SOLVEPNP_ITERATIVE);
		//cv::solvePnP(keypointPos, match_point_undistort2, K, distCoeffs, rvec, t);
		cv::Rodrigues(rvec, R);
		// camera coords to world coords
#if 1
		R = R.t();
		t = -R * t;
#endif
//#define SHOWMATCH
#ifndef ANDROID_NDK
		cv::Mat dest;
#ifdef SHOWMATCH
		std::vector<cv::DMatch> inlierMatch;
		std::vector<cv::Point2f> inlier_points1, inlier_points2;
		for (size_t i = 0; i < mask.rows; ++i) {
			uchar *inliner = mask.ptr<uchar>(i);
			if (inliner[0] == 1) {
				inlierMatch.push_back(match[i]);
				inlier_points1.push_back(mKeyframeKeypoint[match[i].queryIdx].pt);
				inlier_points2.push_back(mKeypoint[match[i].trainIdx].pt);
			}
		}
		//std::cout << "inlierMatch.size=" << inlierMatch.size() << std::endl;
		cv::Mat dest;
		cv::drawMatches(mKeyframe, mKeyframeKeypoint, mFrame, mKeypoint, inlierMatch, dest);
		cv::namedWindow("feature matches", cv::WINDOW_AUTOSIZE);
		cv::imshow("feature matche", dest);
		cvWaitKey(50);
#else
		std::vector<cv::DMatch> inlierMatch;
		std::vector<cv::Point2f> inlier_points1, inlier_points2;
		cv::drawKeypoints(mFrame, mKeypoint, dest);
		for (size_t i = 0; i < inliers.rows; i++) {
			size_t n = inliers.at<int>(i, 0);
			inlierMatch.push_back(match[n]);
			inlier_points1.push_back(mKeyframeKeypoint[match[n].queryIdx].pt);
			inlier_points2.push_back(mKeypoint[match[n].trainIdx].pt);
			cv::line(dest, match_point1[n], match_point2[n], cv::Scalar(255, 0, 0));
		}
		cv::imshow("features", dest);
#endif
#endif
#if 0
#ifndef ANDROID_NDK
		std::cout << "PnPR=" << R << std::endl;
		std::cout << "PnPt=" << t << std::endl;
		std::cout << "inlier_points1.size=" << inlier_points1.size() << std::endl;
		std::cout << "time:" << GetTickCount() - start << std::endl;
#endif
#endif

#if 0
		if (inliers.rows < 10) {
			return 0;
		}
		currentR = R;
		currentt = t;
#else
		currentR = R;
		currentt = t;
		static int count = 0;
		if (updateKeyframe) {
			if (2 < count) {
				insertKeyframe();
				std::cout << "insertKeyframe" << std::endl;
			}
		} else if ((KEYFRAME_MINIMUM_UPDATE_FRAME < framesAfterKeyframe) && (inliers.rows < MATCH_UPDATETHRESHOLD)/**/) {
			mFrame.copyTo(mCandidateFrame);
			mCandidateKeypoint = mKeypoint;
			mDescriptor.copyTo(mCandidateDescriptor);
			currentR.copyTo(mCandidateR);
			currentt.copyTo(mCandidatet);
			updateKeyframe = true;
			count = 0;
			std::cout << "updateKeyframe" << std::endl;
		}
		count++;
#endif
	}
	else {
		return 0;
	}

	return match.size();
}


int Decomposer::insertKeyframe(void) {
	bRecognized = false;

	std::vector<std::vector<cv::DMatch> > knnmatch_points;
	cv::BFMatcher matcher(cv::NORM_HAMMING);
	matcher.knnMatch(mCandidateDescriptor, mDescriptor, knnmatch_points, 2);

	//対応点を絞る
	std::vector<cv::DMatch> match;
	//KeyPoint -> Point2d
	std::vector<cv::Point2f> match_point1, match_point_undistort1;
	std::vector<cv::Point2f> match_point2, match_point_undistort2;
	for (size_t i = 0; i < knnmatch_points.size(); ++i) {
		double distance1 = knnmatch_points[i][0].distance;
		double distance2 = knnmatch_points[i][1].distance;

		//第二候補点から距離値が離れている点のみ抽出（いい点だけ残す）
		if ((distance1 < MATCHER_THRESHOLD_DISTANCE) && (distance1 <= distance2 * MATCHER_THRESHOLD_GOOD_RATIO)) {
			match.push_back(knnmatch_points[i][0]);
			match_point1.push_back(mCandidateKeypoint[knnmatch_points[i][0].queryIdx].pt);
			match_point2.push_back(mKeypoint[knnmatch_points[i][0].trainIdx].pt);
		}
	}
//#define UNDISTORTION
#ifdef UNDISTORTION
	std::cout << match_point1.size() << std::endl;
	cv::undistortPoints(match_point1, match_point_undistort1, K, distCoeffs);
	std::cout << match_point_undistort1.size() << std::endl;
	std::cout << match_point1[1] << ", " << match_point_undistort1[1] << std::endl;
	cv::undistortPoints(match_point2, match_point_undistort2, K, distCoeffs);
#else
	match_point_undistort1 = match_point1;
	match_point_undistort2 = match_point2;
#endif
#ifdef ANDROID_NDK
	__android_log_print(ANDROID_LOG_DEBUG, "Decomposer:", "match.size:%d", match.size());
#endif

	// 十分な対応点がある
	if (match.size() > MATCHEDTHRESHOLD) {
		cv::Mat E, R, t, mask;
#ifndef ANDROID_NDK
		long int start = GetTickCount();
#endif
#if 0
#ifdef UNDISTORTION
		double focal = 1.0;
		cv::Point2d pp(0, 0);
		E = cv::findEssentialMat(match_point_undistort1, match_point_undistort2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
		std::cout << "time:" << GetTickCount() - start << std::endl;
		recoverPose(E, match_point_undistort1, match_point_undistort2, R, t, focal, pp, mask);
#else
		E = cv::findEssentialMat(match_point_undistort1, match_point_undistort2, K, cv::RANSAC, 0.999, 1.0, mask);
		std::cout << "time:" << GetTickCount() - start << std::endl;
		recoverPose(E, match_point_undistort1, match_point_undistort2, K, R, t, mask);
#endif
#else
		E = cv::findEssentialMat(match_point_undistort1, match_point_undistort2, K, cv::RANSAC, 0.999, 1.0, mask);
		std::cout << "time:" << GetTickCount() - start << std::endl;
		// recoverPoseから姿勢を出すのではなく、前回のKeyframeとのマッチが取れた特徴点からsolvePnP
		// 以下の逆算をしたい(R, tを求める)
#if 1
		std::cout << "mCandidateR" << mCandidateR << std::endl;
		std::cout << "currentR" << currentR << std::endl;
		std::cout << "mCandidatet" << mCandidatet << std::endl;
		std::cout << "currentt" << currentt << std::endl;
		//	currentR = R.t() * prevR;
		//	currentt = prevt - (currentR * t);
		R = (currentR * mCandidateR.t()).t();
		t = mCandidateR.t() * (mCandidatet - currentt);

		std::cout << "R=" << R << std::endl;
		std::cout << "t=" << t << std::endl;
		std::cout << std::endl;
		cv::Mat R_, t_;
		recoverPose(E, match_point_undistort1, match_point_undistort2, K, R_, t_, mask);
		std::cout << "R_=" << R_ << std::endl;
		std::cout << "t_=" << t_ << std::endl;
		std::cout << std::endl;
		std::cout << "I=" << R_ * R.t() << std::endl;
		std::cout << std::endl;
		std::cout << "currentR_" << R.t() * mCandidateR << std::endl;
		std::cout << "currentt_" << mCandidatet - (R.t() * mCandidateR * t) << std::endl;
		std::cout << std::endl;
#else
		//	currentR = R * prevR;
		//	currentt = prevt + (currentR * t);
		R = currentR * mCandidateR.t();
		t = currentR.t() * (R*currentt - mCandidatet);
#endif
#if 1
		double t1, t2, t3;
		t1 = t.at<double>(0, 0);
		t2 = t.at<double>(1, 0);
		t3 = t.at<double>(2, 0);
		double scale1 = sqrt(t1 * t1 + t2 * t2 + t3 * t3);
		recoverPose(E, match_point_undistort1, match_point_undistort2, K, R, t, mask);
		t1 = t.at<double>(0, 0);
		t2 = t.at<double>(1, 0);
		t3 = t.at<double>(2, 0);
		double scale2 = sqrt(t1 * t1 + t2 * t2 + t3 * t3);
		t *= (1.0 / scale2);
#endif
#endif
//#define SHOWMATCH
#ifndef ANDROID_NDK
		cv::Mat dest;
		std::vector<cv::DMatch> inlierMatch;
		std::vector<cv::Point2f> inlier_points1, inlier_points2;
		mFrame.copyTo(dest);
		//cv::drawKeypoints(mFrame, mKeypoint, dest);
		for (size_t i = 0; i < mask.rows; ++i) {
			uchar *inliner = mask.ptr<uchar>(i);
			if (inliner[0] == 1) {
				inlierMatch.push_back(match[i]);
				inlier_points1.push_back(mCandidateKeypoint[match[i].queryIdx].pt);
				inlier_points2.push_back(mKeypoint[match[i].trainIdx].pt);
				cv::line(dest, match_point1[i], match_point2[i], cv::Scalar(255, 0, 0));
				cv::circle(dest, match_point2[i], 5, cv::Scalar(128, 128, 0), 1, 8, 0);
			}
		}
		cv::imshow("features", dest);
#endif
		if (inlierMatch.size() < MATCHEDTHRESHOLD) {
			return 0;
		}

		std::vector<cv::Point2f> inlier_points1_distorted = inlier_points1;
		std::vector<cv::Point2f> inlier_points2_distorted = inlier_points2;
		cv::undistortPoints(inlier_points1, inlier_points1, K, distCoeffs);
		cv::undistortPoints(inlier_points2, inlier_points2, K, distCoeffs);
		// Triangulation
		cv::Mat P0 = cv::Mat::eye(3, 4, R.type());
		cv::Mat P1 = cv::Mat::eye(3, 4, R.type());
		cv::hconcat(R, t, P1);
		std::cout << P1 << std::endl;
		std::cout << t << std::endl;
		cv::Mat pnts3D(4, inlier_points1.size(), CV_32FC1);
		triangulatePoints(P0, P1, inlier_points1, inlier_points2, pnts3D);
		pnts3D.row(0) /= pnts3D.row(3);
		pnts3D.row(1) /= pnts3D.row(3);
		pnts3D.row(2) /= pnts3D.row(3);
		pnts3D.row(3) /= pnts3D.row(3);

#if 1
#ifndef ANDROID_NDK
		std::cout << "time:" << GetTickCount() - start << std::endl;
#endif
#endif
		std::vector<cv::Point2f> point_debug;
		std::vector<cv::Point3f> keypointPos_debug;
		std::vector<cv::Mat> keypointPos(mCandidateKeypoint.size());
		size_t count = 0;
		std::cout << "match" << match.size() << std::endl;
		std::cout << "mask" << mask.rows << std::endl;
		for (size_t i = 0; i < mask.rows; ++i) {
			uchar *inliner = mask.ptr<uchar>(i);
			if (inliner[0] == 1) {
				cv::Mat points(3, 1, CV_64FC1);
				points.at<double>(0, 0) = pnts3D.at<float>(0, count);// (cv::Range(0, 3), cv::Range(count, count + 1));
				points.at<double>(1, 0) = pnts3D.at<float>(1, count);
				points.at<double>(2, 0) = pnts3D.at<float>(2, count);
				count++;
				// check parallax angle of triangulation
				float cosangle = parallaxAngle(cv::Mat::zeros(cv::Size(1, 3), CV_64FC1), t, points);
				if (cosangle > PARALLAXANGLE_THRESHOLD) continue;
				if (KEYFRAMEPOINTSLENGTH_THRESHOLD < cv::norm(points, cv::NORM_L2)) continue;
				//cv::Mat position = currentR.t() * points - currentt;
				cv::Mat position = mCandidateR * points + mCandidatet;
				cv::Mat pointsf(3, 1, CV_32FC1);
				pointsf.at<float>(0, 0) = position.at<double>(0, 0);
				pointsf.at<float>(1, 0) = position.at<double>(1, 0);
				pointsf.at<float>(2, 0) = position.at<double>(2, 0);
				keypointPos[match[i].queryIdx] = pointsf;// pnts3D(cv::Range(0, 3), cv::Range(count, count + 1));
				point_debug.push_back(mCandidateKeypoint[match[i].queryIdx].pt);
				//point_debug.push_back(mKeypoint[match[i].trainIdx].pt);
				keypointPos_debug.push_back(cv::Point3f(pointsf.at<float>(0, 0), pointsf.at<float>(1, 0), pointsf.at<float>(2, 0)));
			}
		}

		// いい点がたくさん取れなかったら捨てる
		size_t keypoints_count = keypointPos_debug.size();
		std::cout << "keypointPos_debug.size()" << keypointPos_debug.size() << std::endl;
		if (keypointPos_debug.size() < KEYFRAMEPOINTS_TRHESHOLD) return 0;
#if 0
		cv::Mat rvec, tvec, inliers;
		for (size_t n = 0; n < point_debug.size(); n++) {
			std::cout << point_debug[n] << "," << keypointPos_debug[n] << std::endl;
		}
		cv::solvePnPRansac(keypointPos_debug, point_debug, K, distCoeffs, rvec, tvec, false, 100, 20.0f, 0.95, inliers, cv::SOLVEPNP_ITERATIVE);

		cv::Rodrigues(rvec, R_);
		std::cout << R_ << std::endl;
		std::cout << tvec << std::endl;

		std::vector<cv::Point2f> inlier_points;
		cv::drawKeypoints(mFrame, mKeypoint, dest);
		std::cout << "inliers:" << inliers.rows << std::endl;
#endif

		mMatch = match;
		mKeypointPos = keypointPos;

		mCandidateFrame.copyTo(mKeyframe);
		mKeyframeKeypoint = mCandidateKeypoint;
		mCandidateDescriptor.copyTo(mKeyframeDescriptor);
		mCandidateR.copyTo(mKeyframeR);
		mCandidatet.copyTo(mKeyframet);
		updateKeyframe = false;
		framesAfterKeyframe = 0;
	} else {
		return 0;
	}

	return match.size();
}


int Decomposer::matchFeatures(cv::Mat mScene) {
    bRecognized = false;

    if (0 == mScene.size().width) return 0;
	// グレイスケール化
	cvtColor(mScene, mScene, cv::COLOR_BGR2GRAY);
	// INTER_LINER（バイリニア補間）でのサイズ変更
    cv::resize(mScene, mFrame, cv::Size(imageWidth, mScene.size().height * imageWidth / mScene.size().width), 0, 0);
	//cv::undistort(mScene, mFrame, K, distCoeffs);
#ifdef ANDROID_NDK
    __android_log_print(ANDROID_LOG_DEBUG, "Decomposer:", "matchSize: %d, %d", mFrame.size().width, mFrame.size().height);
#endif

    // 特徴量設定
    cv::Ptr<cv::FEATUREDETECTOR> algorithm = cv::FEATUREDETECTOR::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::FAST_SCORE, 31, 20);
	//cv::Ptr<cv::FEATUREDETECTOR> algorithm = cv::FEATUREDETECTOR::create();

#if 1
	std::vector<cv::KeyPoint> keypoint;
	cv::Mat descriptor;

    // 特徴点抽出
    algorithm->detect(mFrame, keypoint);
    if(keypoint.size() == 0)return 0;

    // 特徴記述
    algorithm->compute(mFrame, keypoint, descriptor);
    if(keypoint.size() == 0)return 0;

    mKeypoint = keypoint;
    mDescriptor = descriptor;
#else
	std::vector<cv::KeyPoint> keypoint;
	cv::Mat descriptor;

	mKeypoint.clear();

	size_t x = mFrame.size().width / 128;
	size_t y = mFrame.size().height / 180;
	for (size_t i = 0; i < x; i++) {
		for (size_t j = 0; j < y; j++) {
			// 特徴点抽出
			algorithm->detect(mFrame(cv::Range(i * 128, (i + 1) * 128), cv::Range(j * 180, (j + 1) * 180)), keypoint);
			if (keypoint.size() == 0)continue;
			for (size_t k = 0; k < keypoint.size(); k++) {
				mKeypoint.push_back(keypoint[k]);
			}
		}
	}

	// 特徴記述
	algorithm->compute(mFrame, mKeypoint, descriptor);
	if (keypoint.size() == 0)return 0;

	mKeypoint = keypoint;
	mDescriptor = descriptor;
#endif

    if(updateKeyframe) {
		mFrame.copyTo(mKeyframe);
        mKeyframeKeypoint = mKeypoint;
		mDescriptor.copyTo(mKeyframeDescriptor);
        updateKeyframe = false;
		framesAfterKeyframe = 0;
		return 0;
    }

    std::vector<std::vector<cv::DMatch> > knnmatch_points;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.knnMatch(mKeyframeDescriptor, mDescriptor, knnmatch_points, 2);

    //対応点を絞る
    std::vector<cv::DMatch> match;
    //KeyPoint -> Point2d
    std::vector<cv::Point2f> match_point1, match_point_undistort1;
    std::vector<cv::Point2f> match_point2, match_point_undistort2;
    for (size_t i = 0; i < knnmatch_points.size(); ++i) {
        double distance1 = knnmatch_points[i][0].distance;
        double distance2 = knnmatch_points[i][1].distance;
#if 0
		std::cout << "dist=" << distance1 << std::endl;
#endif

        //第二候補点から距離値が離れている点のみ抽出（いい点だけ残す）
        if ((distance1 < MATCHER_THRESHOLD_DISTANCE) && (distance1 <= distance2 * MATCHER_THRESHOLD_GOOD_RATIO)) {
            match.push_back(knnmatch_points[i][0]);
            match_point1.push_back(mKeyframeKeypoint[knnmatch_points[i][0].queryIdx].pt);
            match_point2.push_back(mKeypoint[knnmatch_points[i][0].trainIdx].pt);
#if 0
#ifdef ANDROID_NDK
            __android_log_print(ANDROID_LOG_DEBUG, "Decomposer:", "dist1:%.2f,dist2:%.2f", distance1, distance2);
#else
			std::cout << "dist=" << distance1 << std::endl;
#endif
#endif
        }
    }
//#define UNDISTORTION
#ifdef UNDISTORTION
	std::cout << match_point1.size() << std::endl;
	cv::undistortPoints(match_point1, match_point_undistort1, K, distCoeffs);
	std::cout << match_point_undistort1.size() << std::endl;
	std::cout << match_point1[1] << ", " << match_point_undistort1[1] << std::endl;
	cv::undistortPoints(match_point2, match_point_undistort2, K, distCoeffs);
#else
	match_point_undistort1 = match_point1;
	match_point_undistort2 = match_point2;
#endif
#ifdef ANDROID_NDK
    __android_log_print(ANDROID_LOG_DEBUG, "Decomposer:", "match.size:%d", match.size());
#endif

    // 十分な対応点がある
	if (match.size() > MATCHEDTHRESHOLD) {
		cv::Mat E, R, t, mask;
#ifndef ANDROID_NDK
		long int start = GetTickCount();
#endif
#ifdef UNDISTORTION
		double focal = 1.0;
		cv::Point2d pp(0, 0);
		E = cv::findEssentialMat(match_point_undistort1, match_point_undistort2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
		std::cout << "time:" << GetTickCount() - start << std::endl;
		recoverPose(E, match_point_undistort1, match_point_undistort2, R, t, focal, pp, mask);
#else
		E = cv::findEssentialMat(match_point_undistort1, match_point_undistort2, K, cv::RANSAC, 0.999, 1.0, mask);
		std::cout << "time:" << GetTickCount() - start << std::endl;
		recoverPose(E, match_point_undistort1, match_point_undistort2, K, R, t, mask);
#endif
//#define SHOWMATCH
#ifndef ANDROID_NDK
		cv::Mat dest;
		std::vector<cv::DMatch> inlierMatch;
		std::vector<cv::Point2f> inlier_points1, inlier_points2;
		cv::drawKeypoints(mFrame, mKeypoint, dest);
		for (size_t i = 0; i < mask.rows; ++i) {
			uchar *inliner = mask.ptr<uchar>(i);
			if (inliner[0] == 1) {
				inlierMatch.push_back(match[i]);
				inlier_points1.push_back(mKeyframeKeypoint[match[i].queryIdx].pt);
				inlier_points2.push_back(mKeypoint[match[i].trainIdx].pt);
				cv::line(dest, match_point1[i], match_point2[i], cv::Scalar(255, 0, 0));
			}
		}
		cv::imshow("features", dest);
#endif
		if (inlierMatch.size() < 10) {
			return 0;
		}
#define TRIANGULATION
#ifdef TRIANGULATION
		std::vector<cv::Point2f> inlier_points1_distorted = inlier_points1;
		std::vector<cv::Point2f> inlier_points2_distorted = inlier_points2;
		cv::undistortPoints(inlier_points1, inlier_points1, K, distCoeffs);
		cv::undistortPoints(inlier_points2, inlier_points2, K, distCoeffs);
		// Triangulation
		cv::Mat P0 = cv::Mat::eye(3, 4, R.type());
		cv::Mat P1 = cv::Mat::eye(3, 4, R.type());
		cv::hconcat(R, t, P1);
#endif
		std::cout << P1 << std::endl;
		std::cout << t << std::endl;
		cv::Mat pnts3D(4, inlier_points1.size(), CV_32FC1);
		triangulatePoints(P0, P1, inlier_points1, inlier_points2, pnts3D);
#if 1
		pnts3D.row(0) /= pnts3D.row(3);
		pnts3D.row(1) /= pnts3D.row(3);
		pnts3D.row(2) /= pnts3D.row(3);
		pnts3D.row(3) /= pnts3D.row(3);
#else
		cv::Mat triangulatedPoints3D;
		cv::transpose(pnts3D, triangulatedPoints3D);
		cv::convertPointsFromHomogeneous(triangulatedPoints3D, triangulatedPoints3D);
#endif
#if 0
#ifndef ANDROID_NDK
		std::cout << "pnts3D=" << pnts3D.t() << std::endl;
#endif
#endif
#if 1
#ifndef ANDROID_NDK
		std::cout << "R=" << R << std::endl;
		std::cout << "t=" << t << std::endl;
		std::cout << "time:" << GetTickCount() - start << std::endl;
#endif
#endif
		std::vector<cv::Point2f> point_debug;
		std::vector<cv::Point3f> keypointPos_debug;
		std::vector<cv::Mat> keypointPos(mKeyframeKeypoint.size());
		size_t count = 0;
		std::cout << "match" << match.size() << std::endl;
		std::cout << "mask" << mask.rows << std::endl;
		for (size_t i = 0; i < mask.rows; ++i) {
			uchar *inliner = mask.ptr<uchar>(i);
			if (inliner[0] == 1) {
				cv::Mat points(3, 1, CV_64FC1);
				points.at<double>(0, 0) = pnts3D.at<float>(0, count);// (cv::Range(0, 3), cv::Range(count, count + 1));
				points.at<double>(1, 0) = pnts3D.at<float>(1, count);
				points.at<double>(2, 0) = pnts3D.at<float>(2, count);
				count++;
				// check parallax angle of triangulation
				float cosangle = parallaxAngle(cv::Mat::zeros(cv::Size(1, 3), CV_64FC1), t, points);
				if (cosangle > PARALLAXANGLE_THRESHOLD) continue;
				if (KEYFRAMEPOINTSLENGTH_THRESHOLD < cv::norm(points, cv::NORM_L2)) continue;
				//cv::Mat position = currentR.t() * points - currentt;
				cv::Mat position = currentR * points + currentt;
				cv::Mat pointsf(3, 1, CV_32FC1);
				pointsf.at<float>(0, 0) = position.at<double>(0, 0);
				pointsf.at<float>(1, 0) = position.at<double>(1, 0);
				pointsf.at<float>(2, 0) = position.at<double>(2, 0);
				keypointPos[match[i].queryIdx] = pointsf;// pnts3D(cv::Range(0, 3), cv::Range(count, count + 1));
				point_debug.push_back(mKeyframeKeypoint[match[i].queryIdx].pt);
				//point_debug.push_back(mKeypoint[match[i].trainIdx].pt);
				keypointPos_debug.push_back(cv::Point3f(pointsf.at<float>(0, 0), pointsf.at<float>(1, 0), pointsf.at<float>(2, 0)));
			}
		}

//#define DEBUG
#ifdef DEBUG
		cv::Mat rvec, tvec, R_, inliers;
		std::ofstream ofile;
		ofile.open("debug.txt");
		for (size_t n = 0; n < point_debug.size(); n++) {
			std::cout << point_debug[n] << "," << keypointPos_debug[n] << std::endl;
			ofile << point_debug[n] << "," << keypointPos_debug[n] << std::endl;
		}
		ofile.close();
		cv::solvePnPRansac(keypointPos_debug, point_debug, K, distCoeffs, rvec, tvec, false, 100, 20.0f, 0.95, inliers, cv::SOLVEPNP_ITERATIVE);

		cv::Rodrigues(rvec, R_);
		std::cout << R_ << std::endl;
		std::cout << tvec << std::endl;

		std::vector<cv::Point2f> inlier_points;
		cv::drawKeypoints(mFrame, mKeypoint, dest);
		std::cout << "inliers:" << inliers.rows << std::endl;
#endif
		updatePose(R, t);
		mMatch = match;
		mKeypointPos = keypointPos;
	} else {
		return 0;
	}


    return match.size();
}

float Decomposer::parallaxAngle(cv::Mat camera1, cv::Mat camera2, cv::Mat point) {
	cv::Mat diff1, diff2;
	diff1 = camera1 - point;
	diff2 = camera2 - point;

#ifdef DEBUG
	std::cout << point << std::endl;
	std::cout << diff1 << std::endl;
	std::cout << diff2 << std::endl;
#endif

	double norm1 = cv::norm(diff1, cv::NORM_L2);
	double norm2 = cv::norm(diff2, cv::NORM_L2);

	double cosangle = diff1.dot(diff2) / norm1 / norm2;

#ifdef DEBUG
	std::cout << "angle" << acos(cosangle) * 180 / CV_PI << std::endl;
#endif

	return cosangle;

}

int Decomposer::drawHomography(cv::Mat mScene)
{
    std::vector<cv::Point2d> mvDestSceneCorners(4);
    // 本当は同期処理が必要！
    // keypointを描画
#if 0
    __android_log_print(ANDROID_LOG_DEBUG, "Decomposer:", "cvtColor");
    cvtColor(mScene, mScene, cv::COLOR_BGRA2GRAY);
    if(!bKeypoint)return 0;
    if(mCurrentKeypoint.size() == 0)return 0;
    __android_log_print(ANDROID_LOG_DEBUG, "Decomposer:", "drawKeypoints");
    cv::drawKeypoints(
            mScene, // 入力画像
            mCurrentKeypoint, // 特徴点
            mScene, // 出力画像
            cv::Scalar(255, 255, 255, 127),
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS // 描画のオプション  DRAW_RICH_KEYPOINTSを選んだ場合は、キーポイントのサイズと方向が描画される
    );
    return 0;
#endif
#if 1
    for (int i = 0; i < mKeypoint.size(); i++){
        cv::Point2f pt_cur, pt_prev;
        pt_cur = mKeypoint[i].pt * mScene.size().width / imageWidth;
        cv::circle(mScene, pt_cur, 3, cv::Scalar(0, 0, 255));
    }
#endif
#if 1
    for (int i = 0; i < (int)mMatch.size(); i++) {
        cv::Point2f pt_cur, pt_prev;
        pt_cur = mKeypoint[mMatch[i].trainIdx].pt * mScene.size().width / imageWidth;
        pt_prev = mKeyframeKeypoint[mMatch[i].queryIdx].pt * mScene.size().width / imageWidth;
        cv::line(mScene, pt_cur, pt_prev, cv::Scalar(255, 0, 0));
    }
#endif
    return 0;
}

void Decomposer::cameraPoseFromHomography(const cv::Mat& H, cv::Mat& pose) {
    cv::Mat A = K.inv() * H * K;
#ifndef ANDROID_NDK
	std::cout << "A=" << A << std::endl;
#endif

    //Perfrom SVD on A
    cv::SVD decomp = cv::SVD(A);

    //U
    cv::Mat U = decomp.u;
    //S
    float d1 = decomp.w.at<double>(0, 0);
    float d2 = decomp.w.at<double>(1, 0);
    float d3 = decomp.w.at<double>(2, 0);
    //V
    cv::Mat Vt = decomp.vt;
    cv::Mat V = decomp.vt.t(); //Needs to be decomp.vt.t(); (transpose once more)

#ifdef ANDROID_NDK
	__android_log_print(ANDROID_LOG_DEBUG, "SVD:", "d=%.2f, %.2f, %.2f",
                        d1, d2, d3);
#endif

    float s = cv::determinant(U) * cv::determinant(Vt);

    // Equation (12)
    float d1_2 = d1 * d1;
    float d2_2 = d2 * d2;
    float d3_2 = d3 * d3;
    float x1_ = sqrt((d1_2 - d2_2) / (d1_2 - d3_2));
    float x3_ = sqrt((d2_2 - d3_2) / (d1_2 - d3_2));
    float x1[] = {+x1_, +x1_, -x1_, -x1_};// epsilon1 * x1_, where epsilon1 = +/-1
    float x3[] = {+x3_, -x3_, +x3_, -x3_};// epsilon3 * x3_, where epsilon3 = +/-1

    std::vector<cv::Mat> R;
    std::vector<cv::Mat> t;
    cv::Mat R_(3, 3, CV_64F, cv::Scalar(0));
    cv::Mat t_(3, 1, CV_64F, cv::Scalar(0));
    // Case d' > 0
    float st_ = sqrt((d1_2 - d2_2) * (d2_2 - d3_2)) / ((d1 + d3) * d2);
    float st[] = {+st_, -st_, -st_, +st_};// epsilon1 * epsilon3 * st_
    float ct = (d2_2 + d1 * d3) / ((d1 + d3) * d2);
    for (int i = 0; i < 4; i++) {
        R_.at<double>(0, 0) = ct;
        //R_.at<double>(0, 1) = 0;
        R_.at<double>(0, 2) = -st[i];
        //R_.at<double>(1, 0) = 0;
        R_.at<double>(1, 1) = 1;
        //R_.at<double>(1, 2) = 0;
        R_.at<double>(2, 0) = +st[i];
        //R_.at<double>(2, 1) = 0;
        R_.at<double>(2, 2) = ct;
        t_.at<double>(0, 0) = (d1 - d3) * x1[i];
        //t_.at<double>(1, 0) = 0;
        t_.at<double>(2, 0) = (d3 - d1) * x3[i];

        R.push_back(s * U * R_ * Vt);
        t.push_back(U * t_);
#ifdef ANDROID_NDK
        __android_log_print(ANDROID_LOG_DEBUG, "SVD:", "t=%.2f, %.2f, %.2f",
                            t[i].at<double>(0, 0),
                            t[i].at<double>(1, 1),
                            t[i].at<double>(2, 2));
#endif
    }

    // Case d' < 0
    float sp_ = sqrt((d1_2 - d2_2) * (d2_2 - d3_2)) / ((d1 - d3) * d2);
    float sp[] = {+st_, -st_, -st_, +st_};// epsilon1 * epsilon3 * st_
    float cp = (d1 * d3 - d2_2) / ((d1 - d3) * d2);
    for (int i = 0; i < 4; i++) {
        R_.at<double>(0, 0) = cp;
        //R_.at<double>(0, 1) = 0;
        R_.at<double>(0, 2) = sp[i];
        //R_.at<double>(1, 0) = 0;
        R_.at<double>(1, 1) = -1;
        //R_.at<double>(1, 2) = 0;
        R_.at<double>(2, 0) = sp[i];
        //R_.at<double>(2, 1) = 0;
        R_.at<double>(2, 2) = -cp;
        t_.at<double>(0, 0) = (d1 + d3) * x1[i];
        //t_.at<double>(1, 0) = 0;
        t_.at<double>(2, 0) = (d1 + d3) * x3[i];

        R.push_back(s * U * R_ * Vt);
        t.push_back(U * t_);
#ifdef ANDROID_NDK
        __android_log_print(ANDROID_LOG_DEBUG, "SVD:", "t=%.2f, %.2f, %.2f",
                            t[i + 4].at<double>(0, 0),
                            t[i + 4].at<double>(1, 1),
                            t[i + 4].at<double>(2, 2));
#endif
    }

#ifdef ANDROID_NDK
    __android_log_print(ANDROID_LOG_DEBUG, "SVD:", "s=%.2f", s);

    __android_log_print(ANDROID_LOG_DEBUG, "SVD:", "t=%.2f, %.2f, %.2f",
                        t[0].at<double>(0, 0),
                        t[0].at<double>(1, 1),
                        t[0].at<double>(2, 2));
#endif
#ifndef ANDROID_NDK
	std::cout << "R=" << R[0] << std::endl;
	std::cout << "t=" << t[0] << std::endl;
#endif
}

void Decomposer::cameraPoseFromFundamental(const cv::Mat& F, cv::Mat& pose) {
    cv::Mat E = K.t() * F * K;

    //Perfrom SVD on E
    cv::SVD decomp = cv::SVD(E);
#ifndef ANDROID_NDK
	std::cout << "U=" << decomp.u << std::endl;
	std::cout << "S=" << decomp.w << std::endl;
	std::cout << "V=" << decomp.vt << std::endl;
#endif

    //U
    cv::Mat U = decomp.u;
    //S
    cv::Mat S(3, 3, CV_64F, cv::Scalar(0));
    S.at<double>(0, 0) = decomp.w.at<double>(0, 0);
    S.at<double>(1, 1) = decomp.w.at<double>(1, 0);
    S.at<double>(2, 2) = decomp.w.at<double>(2, 0);
    //V
    cv::Mat V = decomp.vt; //Needs to be decomp.vt.t(); (transpose once more)
    //W
    cv::Mat W(3, 3, CV_64F, cv::Scalar(0));
    W.at<double>(0, 1) = -1;
    W.at<double>(1, 0) = 1;
    W.at<double>(2, 2) = 1;

    // 実際はW.t()を使った場合、tが+/-で計4通りになる
    cv::Mat R = U * W * V.t();
    cv::Mat t = U.col(2);

    if (cv::determinant(R) < 0) R = -R;
#ifndef ANDROID_NDK
	std::cout << "R=" << R << std::endl;
	std::cout << "t=" << U.col(2) << std::endl;
#endif

    cv::hconcat(R, U.col(2), pose);
}
