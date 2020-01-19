#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include "decomposer.h"

int main(int argc, char *argv[])
{
    const char sTemplate[] = "/Users/kohei/data/dataset/sequences/00/image_0/%06d.png";
    const size_t start = 0;
    const size_t stop = 4980;
    Decomposer decomposer;
    size_t step = 1;

    std::setw(2);
    std::setprecision(2);

    cv::Mat traj = cv::Mat::zeros(1200, 1200, CV_8UC3);
    double scale = 1;
    cv::Point2f offset(600, 600);
#if 1
#if 0
    char sFrame[256];
    sprintf(sFrame, sTemplate, start);
    cv::Mat frame = cv::imread(sFrame, cv::IMREAD_COLOR);
    decomposer.updateKey();
    decomposer.matchFeatures(frame);
    static bool initialized = false;
    for (size_t i = start + 5; i < stop; i += step) {
        sprintf(sFrame, sTemplate, i);
        cv::Mat frame = cv::imread(sFrame, cv::IMREAD_COLOR);
#if 0
        cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
        cv::imshow("frame", frame);
#endif

        if (false == initialized) {
            std::cout << "initializing..." << std::endl;
            decomposer.matchFeatures(frame);
            initialized = true;
        } else {
            if (decomposer.track(frame) == 0) {
                break;
            }
        }

        cv::Point2f position = decomposer.getPosition2() * scale;
        position.y = -position.y;
        position += offset;
        cv::circle(traj, position, 1, CV_RGB(255, 0, 0), 2);

        positions.push_back(decomposer.getPosition());

        std::vector<cv::Mat> keypointPos = decomposer.getPointCloud();
        for (size_t j = 0; j < keypointPos.size(); j++) {
            if (keypointPos[j].empty()) continue;
            keypoints.push_back(keypointPos[j]);
            cv::Point2f point(keypointPos[j].at<float>(0, 0), -keypointPos[j].at<float>(2, 0));
            point = point * scale + offset;
            cv::circle(traj, point, 1, CV_RGB(0, 255, 0), 1);
        }

        imshow("Trajectory", traj);
        cv::waitKey(1);
    }
#else
    for (size_t i = start; i < stop; i += step)
    {
        char sKeyframe[256], sFrame[256];
        sprintf(sKeyframe, sTemplate, (int)i);
        sprintf(sFrame, sTemplate, (int)(i + step));
        cv::Mat keyframe = cv::imread(sKeyframe, cv::IMREAD_COLOR);
        cv::Mat frame = cv::imread(sFrame, cv::IMREAD_COLOR);

        decomposer.matchFeatures(keyframe);
        decomposer.matchFeatures(frame);

        decomposer.updateKey();

        cv::Point2f position = decomposer.getPosition2() * scale;
        position.y = -position.y;
        position += offset;
        cv::circle(traj, position, 1, CV_RGB(255, 0, 0), 2);

        std::vector<cv::Mat> keypointPos = decomposer.getPointCloud();
        for (size_t j = 0; j < keypointPos.size(); j++)
        {
            if (keypointPos[j].empty())
                continue;
            // X,Y
            cv::Point2f point(keypointPos[j].at<float>(0, 0), -keypointPos[j].at<float>(2, 0));
            point = point * scale + offset;
            cv::circle(traj, point, 1, CV_RGB(0, 255, 0), 1);
        }

        imshow("Trajectory", traj);
        cv::waitKey(1);
    }
#endif
#else
    for (size_t i = start; i < stop; i += step)
    {
        char sKeyframe[256], sFrame[256];
        sprintf(sKeyframe, sTemplate, i);
        cv::Mat keyframe = cv::imread(sKeyframe, cv::IMREAD_COLOR);

        float imageWidth = 3840;
        float imageHeight = 2160;
        cv::resize(keyframe, keyframe, cv::Size(imageWidth, imageHeight), 0, 0);

        cv::Mat K = (cv::Mat_<double>(3, 3) << 988.833, 0.0, 633.399, 0.0, 990.107, 346.812, 0.0, 0.0, 1.0);
        K = K * imageWidth / 1280;
        K.at<double>(2, 2) = 1.0;
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 4) << 0.006637, -0.052151, 0.002196, -0.000496);
        cv::Mat undistorted;
        long int time = GetTickCount();
        cv::undistort(keyframe, undistorted, K, distCoeffs);
        std::cout << "time:" << GetTickCount() - time << std::endl;
        imshow("keyframe", keyframe);
        imshow("undistorted", undistorted);

        cv::waitKey(1);
    }
#endif

    return 0;
}
