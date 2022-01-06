#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <math.h>
#include <time.h>
#include <iomanip>

#define _USE_MATH_DEFINES //<cmath>에서 M_PI 사용하려고...
#include <cmath> 
using namespace std;

#include <opencv2\opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2\cudafeatures2d.hpp>

#define NOMINMAX  //windows.h 헤더 파일에서 min과 max를 전처리기 매크로로 정의를 해서 발생하는 문제를 해결하기 위함.

#if _DEBUG
#pragma comment(lib, "opencv_world345d.lib")
#endif
#pragma comment(lib, "opencv_world345.lib")
using namespace cv;

void main()
{
	cv::Mat img = cv::imread("F:/_RGBD_Dataset7_Scenes/fire/sequence1/frame-000000.color.png"); //IMREAD_ANYDEPTH
	cv::Mat depth_map = cv::imread("F:/_RGBD_Dataset7_Scenes/fire/sequence1/frame-000000.depth.png", cv::IMREAD_ANYDEPTH); //IMREAD_ANYDEPTH

	cv::Mat img2 = cv::imread("F:/_RGBD_Dataset7_Scenes/fire/sequence1/frame-000001.color.png"); //IMREAD_ANYDEPTH
	cv::Mat depth_map2 = cv::imread("F:/_RGBD_Dataset7_Scenes/fire/sequence1/frame-000001.depth.png", cv::IMREAD_ANYDEPTH); //IMREAD_ANYDEPTH

	float fx = 585;
	float fy = 585;
	float cx = 320;
	float cy = 240;
	int w = depth_map.cols;
	int h = depth_map.rows;

	vector<Point2f> vec_point2ds;
	vector<Point3f> vec_point3ds;


	cv::Mat dep_img_meter(depth_map.size().height, depth_map.size().width, CV_32F);
	depth_map.convertTo(depth_map, CV_16U);
	//uint16 --> float (mm->m)
	for (int u = 0; u < h; u++) {
		for (int v = 0; v < w; v++) {
			dep_img_meter.at<float>(u, v) = (1.0f / 1000.0f)*depth_map.at<uint16_t>(u, v);
		}
	}

	float * depth_ptr = dep_img_meter.ptr<float>(0);
	//uint16 --> float (mm->m)
	for (int y = 0; y <h; y++) {
		for (int x = 0; x < w; x++) {

			float d = depth_ptr[y*h + x];

			float xx = (cx - x) / fx;
			float yy = (cy - y) / fy;
			float z_3d = d / sqrt(1.0 + xx * xx + yy * yy);
			float x_3d = xx * z_3d;
			float y_3d = yy * z_3d;

			int dbug = 0;

			vec_point2ds.push_back(Point2f(x, y));

		}
	}





	//cv::Mat pic16bit;
	//tmp2.convertTo(pic16bit, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
	cv::namedWindow("img");
	cv::imshow("img", img);
	cv::waitKey(0);
}

