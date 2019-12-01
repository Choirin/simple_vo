// vSLAM.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <conio.h>
#include <windows.h>

#include "Decomposer.h"
#define SHOWOPENGL

#ifdef SHOWOPENGL
#include "glut.h"

#pragma comment(lib, "glut32.lib")
#endif

#ifdef SHOWOPENGL
double rt[16];
static double view_pos = 8.0;
static double vertical_pos = 0.0;
static double horizontal_pos = 0.0;
std::vector<cv::Mat> positions;
std::vector<cv::Mat> keypoints;

static GLdouble ax = 0.0, ay = 1.0, az = 0.0;
static GLdouble angle = 0.0;
#define SCALE 360.0
static double sx, sy;
static int cx, cy;
static double ca;

static long int center_pos = 0;

void display()
{
	double scale = 5;
	//glClearColor(0.f, 0.f, 0.f, 1.f);
	//glClear(GL_COLOR_BUFFER_BIT);

	cv::Point3f center(positions[center_pos].at<double>(0, 0), positions[center_pos].at<double>(1, 0), positions[center_pos].at<double>(2, 0));;

	// モデルビュー変換行列の設定
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// 視点の移動（物体の方を奥に移動）
	gluLookAt(
		0.0, 0.0, view_pos,
		0.0, 0.0, 0.0,
		0.0, 1.0, 0.0);

	glTranslated(horizontal_pos, vertical_pos, 0.0);

	// 描画処理
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBegin(GL_POINTS);
	glPointSize(2);
	long int count;
	count = keypoints.size();
	for (long int i = 0; i < count; i++) {
		cv::Point3f point(keypoints[i].at<float>(0, 0), keypoints[i].at<float>(1, 0), keypoints[i].at<float>(2, 0));
		point -= center;
		point /= scale;
		glColor3f(0.0, 0.8, 0.0);
		glVertex3f(point.x, point.y, point.z);
	}

	glPointSize(4);
	count = positions.size();
	for (long int i = 0; i < count; i++) {
		cv::Point3f point(positions[i].at<double>(0, 0), positions[i].at<double>(1, 0), positions[i].at<double>(2, 0));
		point -= center;
		point /= scale;
		glColor3f(0.8, 0.0, 0.0);
		glVertex3f(point.x, point.y, point.z);
	}
	glEnd();

	// ダブルバッファリング
	glutSwapBuffers();

	glFlush();
}

void idle(void)
{
	glutPostRedisplay();
}
static void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case '\033':
		/* ESC か q か Q をタイプしたら終了 */
		exit(0);
		break;
	case 'f':
		view_pos -= .2;
		break;
	case 'd':
		view_pos -= .02;
		break;
	case 's':
		view_pos += .02;
		break;
	case 'a':
		view_pos += .2;
		break;
	case 'y':// 縦方向
		vertical_pos += .2;
		break;
	case 'n':
		vertical_pos -= .2;
		break;
	case 'j':// 横方向
		horizontal_pos += .2;
		break;
	case 'g':
		horizontal_pos -= .2;
		break;
	case 'o':
		center_pos += 9;
		if (positions.size() <= center_pos) center_pos = positions.size() - 1;
		break;
	case 'i':
		center_pos -= 9;
		if (center_pos < 0) center_pos = 0;
		break;
	default:
		break;
	}
}

void init_gl(void) {
}

#endif

int main(int argc, char* argv[]) {
//	const char sTemplate[256] = "D:/Hawaii/ActionCam/jpeg/scene_%04d.jpg"; const size_t start = 500; const size_t stop = 20000;
//	const char sTemplate[] = "E:/dataset/160621b/jpeg/scene_%04d.jpg"; const size_t start = 0; const size_t stop = 2256;
//	const char sTemplate[] = "D:/dataset/160621c/jpeg/scene_%04d.jpg"; const size_t start = 50; const size_t stop = 3124;
//	const char sTemplate[] = "D:/dataset/160621d/jpeg/scene_%04d.jpeg"; const size_t start = 1000; const size_t stop = 2000;
//	const char sTemplate[] = "./data4/IMG_%04d.JPG"; const size_t start = 1902; const size_t stop = 1958;
	const char sTemplate[] = "D:/data_odometry_gray/dataset/sequences/10/image_0/%06d.png"; const size_t start = 0; const size_t stop = 4000;
//	const char sTemplate[] = "E:/data_odometry_gray/dataset/sequences/16/image_0/%06d.png"; const size_t start = 00; const size_t stop = 4980;
//	const char sTemplate[] = "C:/Users/kohei/Downloads/data_odometry_gray/dataset/sequences/13/image_0/%06d.png"; const size_t start = 0; const size_t stop = 4980;
//	const char sTemplate[] = "G:/image_0/%06d.png"; const size_t start = 0; const size_t stop = 4980;
	Decomposer decomposer;
	size_t step = 1;

	std::setw(2);
	std::setprecision(2);

	cv::Mat traj = cv::Mat::zeros(1200, 1200, CV_8UC3);
	double scale = 2;
	cv::Point2f offset(300, 500);
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
		cvWaitKey(1);
	}
#else
	for (size_t i = start; i < stop; i+=step) {
		char sKeyframe[256], sFrame[256];
		sprintf(sKeyframe, sTemplate, i);
		sprintf(sFrame, sTemplate, i + step);
		cv::Mat keyframe = cv::imread(sKeyframe, cv::IMREAD_COLOR);
		cv::Mat frame = cv::imread(sFrame, cv::IMREAD_COLOR);

		decomposer.matchFeatures(keyframe);
		decomposer.matchFeatures(frame);

		decomposer.updateKey();

		cv::Point2f position = decomposer.getPosition2() * scale;
		position.y = -position.y;
		position += offset;
		cv::circle(traj, position, 1, CV_RGB(255,0,0), 2);

		positions.push_back(decomposer.getPosition());

		std::vector<cv::Mat> keypointPos = decomposer.getPointCloud();
		for (size_t j = 0; j < keypointPos.size(); j++) {
			if (keypointPos[j].empty()) continue;
			keypoints.push_back(keypointPos[j]);
			// X,Y
			cv::Point2f point(keypointPos[j].at<float>(0, 0), -keypointPos[j].at<float>(2, 0));
			point = point * scale + offset;
			cv::circle(traj, point, 1, CV_RGB(0, 255, 0), 1);
		}

		imshow("Trajectory", traj);
		cvWaitKey(1);
	}
#endif
#else
	for (size_t i = start; i < stop; i += step) {
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

		cvWaitKey(1);
	}
#endif

#ifdef SHOWOPENGL
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_MULTISAMPLE);
	glutInitWindowSize(640, 480);
	glutCreateWindow(argv[0]);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	glutDisplayFunc(display);
	//glClearColor(1.0, 1.0, 1.0, 0.0);

	init_gl();
	glutMainLoop();

    return 0;
#else
	getch();
	return 0;
#endif
}

