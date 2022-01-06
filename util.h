////////////////////////////////////////////////////////////////////////////
//	File:		    util.h
//	Author:		    Changchang Wu (ccwu@cs.washington.edu)
//	Description :   some utility functions for reading/writing SfM data
//
//  Copyright (c) 2011  Changchang Wu (ccwu@cs.washington.edu)
//    and the University of Washington at Seattle 
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//  Version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <time.h>
#include <iomanip>
#include <algorithm>

#define _USE_MATH_DEFINES //<cmath>에서 M_PI 사용하려고...
#include <cmath> 
using namespace std;
#include "DataInterface.h"

#include "dbscan.h"
#include "kmeans.h"

#include <opencv2\opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2\cudafeatures2d.hpp>

#define NOMINMAX  //windows.h 헤더 파일에서 min과 max를 전처리기 매크로로 정의를 해서 발생하는 문제를 해결하기 위함.

//#if _DEBUG
//	#pragma comment(lib, "opencv_world343d.lib")
//#endif
//	#pragma comment(lib, "opencv_world343.lib")
#if _DEBUG
#pragma comment(lib, "opencv_world345d.lib")
#endif
#pragma comment(lib, "opencv_world345.lib")
using namespace cv;



//File loader supports .nvm format and bundler format
bool LoadModelFile(const char* name, string dataname, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx, 
              vector<string>& names, vector<int>& ptc); 
void SaveNVM(const char* filename, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx, 
              vector<string>& names, vector<int>& ptc);
void SaveBundlerModel(const char* filename, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx);

//////////////////////////////////////////////////////////////////
void AddNoise(vector<CameraT>& camera_data, vector<Point3D>& point_data, float percent);
void AddStableNoise(vector<CameraT>& camera_data, vector<Point3D>& point_data,
                    const vector<int>& ptidx, const vector<int>& camidx, float percent);
bool RemoveInvisiblePoints( vector<CameraT>& camera_data, vector<Point3D>& point_data,
                            vector<int>& ptidx, vector<int>& camidx, 
                            vector<Point2D>& measurements, vector<string>& names, vector<int>& ptc);

void GetQuaternionRotationByPnP(cv::Mat R_matrix, cv::Mat t_matrix, float q2[4]);
bool EstimatePoseByPnP(vector<Point2d> features_2d, vector<Point3d> features_3d, double focal_length,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix);

bool EstimatePoseByPnP(vector<Point2d> list_points2d, vector<Point3d> list_points3d, double focal_length,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix)
{
	double f = focal_length;
	const double params[] = { f,   // fx
							  f,  // fy
							  0,      // cx
							  0 };    // cy

	A_matrix.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
	A_matrix.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
	A_matrix.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
	A_matrix.at<double>(1, 2) = params[3];
	A_matrix.at<double>(2, 2) = 1;

	//micro 웹캠... 왜곡계수
	//double k1 = 0.022774;
	//double k2 = -0.041311;
	//double p1 = -0.0055;
	//double p2 = -0.0009367;
	double k1 = 0;
	double k2 = -0;
	double p1 = -0;
	double p2 = -0;
	double d[] = { k1, k2, p1, p2 };
	cv::Mat distCoeffs(4, 1, CV_64FC1, d);

	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	bool useExtrinsicGuess = false;
	//bool correspondence = cv::solvePnP(list_points3d, list_points2d, _A_matrix, distCoeffs, rvec, tvec,
	//	useExtrinsicGuess, SOLVEPNP_ITERATIVE);
	bool correspondence = cv::solvePnPRansac(list_points3d, list_points2d, A_matrix, distCoeffs, rvec, tvec);

	if (correspondence)
	{
		//R|t TEST
		Rodrigues(rvec, R_matrix);
		t_matrix = tvec;

		return correspondence;
	}
	else
		return correspondence;
}


void GetQuaternionRotationByPnP(cv::Mat R_matrix, cv::Mat t_matrix, float q[4])
{
	float r00 = R_matrix.at<double>(0, 0);
	float r01 = R_matrix.at<double>(0, 1);
	float r02 = R_matrix.at<double>(0, 2);
	float r10 = R_matrix.at<double>(1, 0);
	float r11 = R_matrix.at<double>(1, 1);
	float r12 = R_matrix.at<double>(1, 2);
	float r20 = R_matrix.at<double>(2, 0);
	float r21 = R_matrix.at<double>(2, 1);
	float r22 = R_matrix.at<double>(2, 2);
	float t1 = t_matrix.at<double>(0);
	float t2 = t_matrix.at<double>(1);
	float t3 = t_matrix.at<double>(2);

	q[0] = 1 + r00 + r11 + r22;
	if (q[0] > 0.000000001)
	{
		q[0] = sqrt(q[0]) / 2.0;
		q[1] = (r21 - r12) / (4.0 *q[0]);
		q[2] = (r02 - r20) / (4.0 *q[0]);
		q[3] = (r10 - r01) / (4.0 *q[0]);
	}
	else
	{
		double s;
		if (r00 > r11 && r00 > r22)
		{
			s = 2.0 * sqrt(1.0 + r00 - r11 - r22);
			q[1] = 0.25 * s;
			q[2] = (r01 + r10) / s;
			q[3] = (r02 + r20) / s;
			q[0] = (r12 - r21) / s;
		}
		else if (r11 > r22)
		{
			s = 2.0 * sqrt(1.0 + r11 - r00 - r22);
			q[1] = (r01 + r10) / s;
			q[2] = 0.25 * s;
			q[3] = (r12 + r21) / s;
			q[0] = (r02 - r20) / s;
		}
		else
		{
			s = 2.0 * sqrt(1.0 + r22 - r00 - r11);
			q[1] = (r02 + r20) / s;
			q[2] = (r12 + r21) / s;
			q[3] = 0.25f * s;
			q[0] = (r01 - r10) / s;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
bool LoadNVM(ifstream& in, string dataname, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
              vector<string>& names, vector<int>& ptc)
{
    int rotation_parameter_num = 4; 
    bool format_r9t = false;
    string token;
    if(in.peek() == 'N') 
    {
        in >> token; //file header
        if(strstr(token.c_str(), "R9T"))
        {
            rotation_parameter_num = 9;    //rotation as 3x3 matrix
            format_r9t = true;
        }
    }
   

	//kmeans();


    int ncam = 0, npoint = 0, nproj = 0;   
    // read # of cameras
    in >> ncam;  if(ncam <= 1) return false; 

    //read the camera parameters
    camera_data.resize(ncam); // allocate the camera data
    names.resize(ncam);
	float radial_distortion = 0;
    for(int i = 0; i < ncam; ++i)
    {
        double f, q[9], c[3], d[2];
        in >> token >> f ;
        for(int j = 0; j < rotation_parameter_num; ++j) in >> q[j]; 
        in >> c[0] >> c[1] >> c[2] >> d[0] >> d[1];

		//q는 쿼터니언, c는 translation


        camera_data[i].SetFocalLength(f);
        if(format_r9t) 
        {
            camera_data[i].SetMatrixRotation(q);
            camera_data[i].SetTranslation(c);
        }
        else
        {
            //older format for compability -> 쿼터니언과 카메라 중심을 R과 T로 변환 시켜줌
            camera_data[i].SetQuaternionRotation(q);        //quaternion from the file
            camera_data[i].SetCameraCenterAfterRotation(c); //camera center from the file
        }
        camera_data[i].SetNormalizedMeasurementDistortion(d[0]); 
        names[i] = token;
		radial_distortion = d[0];
    }

    //////////////////////////////////////
    in >> npoint;   if(npoint <= 0) return false; 

    //read image projections and 3D points.
	//vector<Mat> featureImages;
	vector<vector<Point2d>> features_2d;
	vector<vector<Point3d>> features_3d;
	//featureImages.resize(ncam);
	features_2d.resize(ncam);
	features_3d.resize(ncam);
    point_data.resize(npoint); 
    for(int i = 0; i < npoint; ++i)
    {
        float pt[3]; int cc[3], npj;
        in  >> pt[0] >> pt[1] >> pt[2] 
            >> cc[0] >> cc[1] >> cc[2] >> npj;


        for(int j = 0; j < npj; ++j)
        {
            int cidx, fidx; float imx, imy;
            in >> cidx >> fidx >> imx >> imy;

            camidx.push_back(cidx);    //camera index
            ptidx.push_back(i);        //point index

            //add a measurment to the vector
            measurements.push_back(Point2D(imx, imy));
            nproj ++;

			//float pt_2d[2] = { 0 };
			//camera_data[cidx].GetProjectionPoint(pt, pt_2d);

			features_2d[cidx].push_back(Point2d(imx, imy));
			features_3d[cidx].push_back(Point3d(pt[0], pt[1], pt[2]));

			/*
			// 실험... 같은 3d가 이미지 별로 어디에 맺히는지...
			////float r2 = camera_data[cidx].radial * (imx*imx + imy * imy);
			////float r2 = radial_distortion * (imx*imx + imy * imy);
			//float r2 = 0;
			//float reproj_err = xx - (1 + r2)*imx + yy - (1 + r2)*imy;
			//printf("camera index:%d, (reporj-x:%f, y:%f), (measure-x:%f, y:%f), (reproj_err:%f)\n", cidx, xx, yy, imx, imy, reproj_err);
			//printf("reproj_err:%f\n", reproj_err);

			//이미지 받아오기, 특징점과 재사영점 비교 확인
			string img_path = "F:/RPNet_test/data/absolute_cambridge/" + dataname + names[cidx];
			//string img_path = "F:/RPNet_test/data/absolute_cambridge/KingsCollege/" + names[cidx];
			int nExt = img_path.rfind("jpg");
			int nName = img_path.rfind("/") + 1;
			string strModExt("png");
			string strReName;
			strReName = img_path.substr(0, nName);
			strReName += img_path.substr(nName, nExt - nName);
			strReName += strModExt;
			Mat img = imread(strReName);

			//이미지 좌표 변환 이미지 센터가 원점인 것에서 왼쪽 상단이 0인 것으로...
			int c_w = img.cols/2; 
			int c_h = img.rows/2;
			//int c_w = 1920;
			//int c_h = 1080;

			//int reproj_x = xx + c_w + 0.5;
			//int reproj_y = yy + c_h + 0.5;
			int feature_x = imx + c_w + 0.5;
			int feature_y = imy + c_h + 0.5;

			//circle(img, Point(reproj_x, reproj_y), 10, Scalar(255, 0, 0), -1);
			circle(img, Point(feature_x, feature_y), 10, Scalar(0, 0, 255), -1);

			//resize(img, img, Size(img.cols / 4, img.rows / 4));

			imshow(names[cidx], img);
			waitKey(0);
			int debug = 0;
			*/
			
        }
        point_data[i].SetPoint(pt); 
        ptc.insert(ptc.end(), cc, cc + 3); 
    }

	printf("finish!\n");


	/*
	KingsCollege, OldHospital, ShopFacade, StMarysChurch
	*/

	float error_T_sum = 0, error_R_sum = 0;
	for (int i = 0; i < ncam; ++i)
	{
		//string img_path = "F:/RPNet_test/data/absolute_cambridge/ShopFacade/sfm/" + names[i];
		string img_path = "F:/RPNet_test/data/absolute_cambridge/" + dataname + names[i];

		cout << names[i] << endl;

		int nExt = img_path.rfind("jpg");  
		int nName = img_path.rfind("/") + 1;
		string strModExt("png");
		string strReName;
		strReName = img_path.substr(0, nName);
		strReName += img_path.substr(nName, nExt - nName);
		strReName += strModExt;

		Mat img = imread(strReName);
		int c_w = img.cols/2; 
		int c_h = img.rows/2;

		int size = features_2d[i].size();
		for (int j = 0; j < size; ++j)
		{
			int feature_x = features_2d[i][j].x + c_w + 0.5;
			int feature_y = features_2d[i][j].y + c_h + 0.5;
			circle(img, Point(feature_x, feature_y), 4, Scalar(0, 0, 255), -1);
		}


		vector<cv::Point2d> list_points2d = features_2d[i];
		vector<cv::Point3d> list_points3d = features_3d[i];

		/** The calibration matrix */
		cv::Mat A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
		cv::Mat R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
		cv::Mat t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix

		/** The computed projection matrix */
		cv::Mat _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);

		bool correspondence = EstimatePoseByPnP(list_points2d, list_points3d, camera_data[i].f, A_matrix, R_matrix, t_matrix);


		float t1 = t_matrix.at<double>(0);
		float t2 = t_matrix.at<double>(1);
		float t3 = t_matrix.at<double>(2);

		float _t1 = camera_data[i].t[0];
		float _t2 = camera_data[i].t[1];
		float _t3 = camera_data[i].t[2];

		float q[4] = { 0 };
		GetQuaternionRotationByPnP(R_matrix, t_matrix, q);

		float q2[4];
		camera_data[i].GetQuaternionRotation(q2);

		//float theta = acos(0) * 180/ M_PI;
		float dot_prod = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3];
		if (dot_prod > 1) dot_prod = 1;
		float theta2 = acos(dot_prod) * 180 / M_PI;

		float err1 = t1 - _t1; float err2 = t2 - _t2; float err3 = t3 - _t3;
		float error_x = sqrt(pow(t1 - _t1, 2) + pow(t2 - _t2, 2) + pow(t3 - _t3, 2));

		error_T_sum += error_x;
		error_R_sum += theta2;

		//printf("err1:%f, err2:%f, err3:%f\n", err1, err2, err3);
		printf("Error XYZ (m):%f, theta:%f\n", error_x, theta2);

		//correspondence = false;
		if (correspondence)
		{
			cout << "Correspondence found" << endl;

			_P_matrix.at<double>(0, 0) = R_matrix.at<double>(0, 0);
			_P_matrix.at<double>(0, 1) = R_matrix.at<double>(0, 1);
			_P_matrix.at<double>(0, 2) = R_matrix.at<double>(0, 2);
			_P_matrix.at<double>(1, 0) = R_matrix.at<double>(1, 0);
			_P_matrix.at<double>(1, 1) = R_matrix.at<double>(1, 1);
			_P_matrix.at<double>(1, 2) = R_matrix.at<double>(1, 2);
			_P_matrix.at<double>(2, 0) = R_matrix.at<double>(2, 0);
			_P_matrix.at<double>(2, 1) = R_matrix.at<double>(2, 1);
			_P_matrix.at<double>(2, 2) = R_matrix.at<double>(2, 2);
			_P_matrix.at<double>(0, 3) = t_matrix.at<double>(0);
			_P_matrix.at<double>(1, 3) = t_matrix.at<double>(1);
			_P_matrix.at<double>(2, 3) = t_matrix.at<double>(2);


			//cv::Mat weighted_img = cv::Mat::zeros(img.rows, img.cols, CV_32FC3);
			cv::Mat weighted_img = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
			cv::Mat gaussian_img = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);
			float* ptr_gaussian_img = gaussian_img.ptr<float>(0);
			for (int j = 0; j < size; ++j)
			{
				float Point3f_x = features_3d[i][j].x;
				float Point3f_y = features_3d[i][j].y;
				float Point3f_z = features_3d[i][j].z;

				// 3D point vector [x y z 1]'
				cv::Mat Point3f_vec = cv::Mat(4, 1, CV_64FC1);
				Point3f_vec.at<double>(0) = Point3f_x;
				Point3f_vec.at<double>(1) = Point3f_y;
				Point3f_vec.at<double>(2) = Point3f_z;
				Point3f_vec.at<double>(3) = 1;

				// 2D point vector [u v 1]'
				cv::Mat Point2f_vec = cv::Mat(3, 1, CV_64FC1);
				Point2f_vec = A_matrix * _P_matrix * Point3f_vec;

				// Normalization of [u v]'
				cv::Point2f Point2f;
				Point2f.x = (float)(Point2f_vec.at<double>(0) / Point2f_vec.at<double>(2));
				Point2f.y = (float)(Point2f_vec.at<double>(1) / Point2f_vec.at<double>(2));
				int pnp_feature_x = Point2f.x + c_w + 0.5;
				int pnp_feature_y = Point2f.y + c_h + 0.5;

				int feature_x = features_2d[i][j].x + c_w + 0.5;
				int feature_y = features_2d[i][j].y + c_h + 0.5;

				float pt[3];
				pt[0] = features_3d[i][j].x;
				pt[1] = features_3d[i][j].y;
				pt[2] = features_3d[i][j].z;

				float pt_2d[2] = { 0 };
				camera_data[i].GetProjectionPoint(pt, pt_2d);
				float xx = pt_2d[0];
				float yy = pt_2d[1];

				//float r2 = camera_data[i].radial * (features_2d[i][j].x*features_2d[i][j].x + features_2d[i][j].y * features_2d[i][j].y);
				//float r2 = radial_distortion * (imx*imx + imy * imy);
				float r2 = 0;
				float reproj_err = xx - (1 + r2)*features_2d[i][j].x + yy - (1 + r2)*features_2d[i][j].y;
				//printf("camera index:%d, (reporj-x:%f, y:%f), (measure-x:%f, y:%f), (reproj_err:%f), ",
				//	i, xx, yy, features_2d[i][j].x, features_2d[i][j].y, reproj_err);
				//printf("reproj_err:%f\n", reproj_err);

				//float pnp_reproj_err = Point2f.x - (1 + r2)*features_2d[i][j].x + Point2f.y - (1 + r2)*features_2d[i][j].y;
				//printf("PnP camera index:%d, (reporj-x:%f, y:%f), (measure-x:%f, y:%f), (reproj_err:%f), ",
				//	i, Point2f.x, Point2f.y, features_2d[i][j].x, features_2d[i][j].y, reproj_err);
				//printf("reproj_err:%f\n", pnp_reproj_err);
				//printf("\n");


				//특징점을 기준으로 가우시안분포로 이미지 패치를 결정하여, 이미지를 만든다.
				//가우시안 마스크의 크기는 보통 6*표준편차-1의 값
				float tmp = 3;
				if (fabs(reproj_err) < 0.1)		tmp = 10;
				else if(fabs(reproj_err) < 1.0) tmp = 7;
				else if (fabs(reproj_err) < 5) tmp = 4;

				//tmp = 1.0f / fabs(reproj_err);
				tmp = 1.0f/fabs(reproj_err)*512;
				tmp = log2(tmp) + 5;
				//int nHalf = tmp;

				//float tmp2 = fabs( log2(fabs(reproj_err)) );
				float sigma = tmp;
				int nHalf = std::min(std::max( (sigma * 6 - 1) / 2, 3.0f), 100.0f);
				//printf("nHalf:%d\n", nHalf);


				float sum = 0;
				for (int n = -nHalf; n <= nHalf; ++n)
				{
					int yy = feature_y + n;
					yy = min(max(yy, 0), img.rows - 1);
					for (int m = -nHalf; m <= nHalf; ++m)
					{
						float w = std::expf(-((m * m) + (n * n)) / (2.0f * sigma * sigma)) / (std::sqrt(2.0f * 3.1415f) * sigma);
						int xx = feature_x + m;
						xx = min(max(xx, 0), img.cols - 1);
						
						w = 1;
						float val = ptr_gaussian_img[yy*img.cols + xx];
						if (w > val) ptr_gaussian_img[yy*img.cols + xx] = w;
						sum += w;
					}
				}
				//for (int n = -nHalf; n <= nHalf; ++n)
				//{
				//	int yy = feature_y + n;
				//	yy = min(max(yy, 0), img.rows - 1);
				//	for (int m = -nHalf; m <= nHalf; ++m)
				//	{
				//		int xx = feature_x + m;
				//		xx = min(max(xx, 0), img.cols - 1);
				//		//ptr_gaussian_img[yy*img.cols + xx] = ptr_gaussian_img[yy*img.cols + xx] / sum;
				//	}
				//}

			} //size => features 개수

			uchar* ptr_img = img.ptr<uchar>(0);
			//float* ptr_weighted_img = weighted_img.ptr<float>(0);
			uchar* ptr_weighted_img = weighted_img.ptr<uchar>(0);
			for (int y = 0; y < img.rows; ++y)
			{
				for (int x = 0; x < img.cols; ++x)
				{
					float w = ptr_gaussian_img[y*img.cols + x];
					ptr_weighted_img[y*img.cols * 3 + x * 3 + 0] = ptr_img[y*img.cols * 3 + x * 3 + 0] * w;
					ptr_weighted_img[y*img.cols * 3 + x * 3 + 1] = ptr_img[y*img.cols * 3 + x * 3 + 1] * w;
					ptr_weighted_img[y*img.cols * 3 + x * 3 + 2] = ptr_img[y*img.cols * 3 + x * 3 + 2] * w;

				}
			}
			//cv::resize(weighted_img, weighted_img, Size(img.cols / 4, img.rows / 4));


			string save_path = "F:/featuresMap_By_reprojetion_error_2/" + dataname + names[i];
			//imshow("test", weighted_img);
			//imwrite(save_path, weighted_img);
			//waitKey(0);
		}  //if correspondence
			

		//imshow(names[i], img);
		imshow("ttt", img);
		//string save_path = "ShopFacade/" + names[i];
		//imwrite(save_path, img);
		waitKey(0);
	} // ncam

	printf("error_T_avr : %f, error_R_avr : %f\n", error_T_sum / ncam, error_R_sum / ncam);

    ///////////////////////////////////////////////////////////////////////////////
    std::cout << ncam << " cameras; " << npoint << " 3D points; " << nproj << " projections\n";

    return true;
}


void SaveNVM(const char* filename, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx, 
              vector<string>& names, vector<int>& ptc)
{
    std::cout << "Saving model to " << filename << "...\n"; 
    ofstream out(filename);

    out << "NVM_V3_R9T\n" << camera_data.size() << '\n' << std::setprecision(12);
    if(names.size() < camera_data.size()) names.resize(camera_data.size(),string("unknown"));
    if(ptc.size() < 3 * point_data.size()) ptc.resize(point_data.size() * 3, 0);

    ////////////////////////////////////
    for(size_t i = 0; i < camera_data.size(); ++i)
    {
        CameraT& cam = camera_data[i];
        out << names[i] << ' ' << cam.GetFocalLength() << ' ';
        for(int j  = 0; j < 9; ++j) out << cam.m[0][j] << ' ';
        out << cam.t[0] << ' ' << cam.t[1] << ' ' << cam.t[2] << ' '
            << cam.GetNormalizedMeasurementDistortion() << " 0\n"; 
    }

    out << point_data.size() << '\n';

    for(size_t i = 0, j = 0; i < point_data.size(); ++i)
    {
        Point3D& pt = point_data[i];
        int * pc = &ptc[i * 3];
        out << pt.xyz[0] << ' ' << pt.xyz[1] << ' ' << pt.xyz[2] << ' ' 
            << pc[0] << ' ' << pc[1] << ' ' << pc[2] << ' '; 

        size_t je = j;
        while(je < ptidx.size() && ptidx[je] == (int) i) je++;
        
        out << (je - j) << ' ';

        for(; j < je; ++j)    out << camidx[j] << ' ' << " 0 " << measurements[j].x << ' ' << measurements[j].y << ' ';
        
        out << '\n';
    }
}


bool LoadBundlerOut(const char* name, ifstream& in, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
              vector<string>& names, vector<int>& ptc)
{
    int rotation_parameter_num = 9; 
    string token;
    while(in.peek() == '#') std::getline(in, token);

    char listpath[1024], filepath[1024];
    strcpy(listpath, name);
    char* ext = strstr(listpath, ".out");
    strcpy(ext, "-list.txt\0"); 

    ///////////////////////////////////
    ifstream listin(listpath);
    if(!listin.is_open())
    {
        listin.close();       listin.clear(); 
        char * slash = strrchr(listpath, '/');
        if(slash == NULL) slash = strrchr(listpath, '\\');
        slash = slash ? slash + 1 : listpath; 
        strcpy(slash, "image_list.txt"); 
        listin.open(listpath);
    }
    if(listin) std::cout << "Using image list: " << listpath << '\n';

    // read # of cameras
    int ncam = 0, npoint = 0, nproj = 0; 
    in >> ncam >> npoint;  
    if(ncam <= 1 || npoint <= 1) return false; 
    std::cout << ncam << " cameras; " << npoint << " 3D points;\n";

    //read the camera parameters
    camera_data.resize(ncam); // allocate the camera data
    names.resize(ncam);

    bool det_checked = false;
    for(int i = 0; i < ncam; ++i)
    {
        float f, q[9], c[3], d[2];
        in >> f >> d[0] >> d[1];
        for(int j = 0; j < rotation_parameter_num; ++j) in >> q[j]; 
        in >> c[0] >> c[1] >> c[2];

        camera_data[i].SetFocalLength(f);
        camera_data[i].SetInvertedR9T(q, c);
        camera_data[i].SetProjectionDistortion(d[0]); 

        if(listin >> filepath && f != 0)
        {
            char* slash = strrchr(filepath , '/');
            if(slash == NULL) slash = strchr(filepath, '\\');
            names[i] = (slash? (slash + 1) : filepath);
            std::getline(listin, token);

            if(!det_checked)
            {
                float det = camera_data[i].GetRotationMatrixDeterminant();
                std::cout << "Check rotation matrix: " << det << '\n';
                det_checked = true;
            }
        }else
        {
            names[i] = "unknown";
        }
    }


    //read image projections and 3D points.
    point_data.resize(npoint); 
    for(int i = 0; i < npoint; ++i)
    {
        float pt[3]; int cc[3], npj;
        in  >> pt[0] >> pt[1] >> pt[2] 
            >> cc[0] >> cc[1] >> cc[2] >> npj;
        for(int j = 0; j < npj; ++j)
        {
            int cidx, fidx; float imx, imy;
            in >> cidx >> fidx >> imx >> imy;

            camidx.push_back(cidx);    //camera index
            ptidx.push_back(i);        //point index

            //add a measurment to the vector
            measurements.push_back(Point2D(imx, -imy));
            nproj ++;
        }
        point_data[i].SetPoint(pt[0], pt[1], pt[2]); 
        ptc.insert(ptc.end(), cc, cc + 3); 
    }
    ///////////////////////////////////////////////////////////////////////////////
    std::cout << ncam << " cameras; " << npoint << " 3D points; " << nproj << " projections\n";
    return true;
}

void SaveBundlerOut(const char* filename, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
			  vector<string>& names, vector<int>& ptc)
{
    char listpath[1024];   strcpy(listpath, filename);
    char* ext = strstr(listpath, ".out"); if(ext == NULL) return;
    strcpy(ext, "-list.txt\0"); 

	ofstream out(filename);
	out << "# Bundle file v0.3\n";
    out << std::setprecision(12); //need enough precision
	out << camera_data.size() << " " << point_data.size() << '\n';

	//save camera data
	for(size_t i = 0; i < camera_data.size(); ++i)
    {
        float q[9], c[3];
		CameraT& ci = camera_data[i];
		out << ci.GetFocalLength() << ' ' << ci.GetProjectionDistortion() << " 0\n";
		ci.GetInvertedR9T(q, c);
        for(int j = 0; j < 9; ++j) out << q[j] << (((j % 3) == 2)? '\n' : ' '); 
        out << c[0] << ' ' << c[1] << ' ' << c[2] << '\n';
	}
	///
	for(size_t i = 0, j = 0; i < point_data.size(); ++i)
	{
		int npj = 0, *ci = &ptc[i * 3]; Point3D& pt = point_data[i];
		while(j + npj < point_data.size() && ptidx[j + npj] == ptidx[j]) npj++;
		///////////////////////////
		out << pt.xyz[0] << ' ' << pt.xyz[1] << ' ' << pt.xyz[2] << '\n';
		out << ci[0] << ' ' << ci[1] << ' ' << ci[2] << '\n';
		out << npj << ' ';
		for(int k = 0; k < npj; ++k) out << camidx[j + k] << " 0 " 
										 << measurements[j + k].x << ' ' << -measurements[j + k].y << '\n';
		out << '\n'; j += npj;
	}

	ofstream listout(listpath);
	for(size_t i = 0; i < names.size(); ++i) listout << names[i] << '\n';
}

template<class CameraT, class Point3D>
bool LoadBundlerModel(ifstream& in, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx)
{
    // read bundle data from a file
    size_t ncam = 0, npt = 0, nproj = 0;
    if(!(in >> ncam >> npt >> nproj)) return false;
    ///////////////////////////////////////////////////////////////////////////////
    std::cout << ncam << " cameras; " << npt << " 3D points; " << nproj << " projections\n";


    camera_data.resize(ncam);
    point_data.resize(npt);
    measurements.resize(nproj);
    camidx.resize(nproj);
    ptidx.resize(nproj);

    for(size_t i = 0; i < nproj; ++i)    
    {
        double x, y;    int cidx, pidx;
        in >> cidx >> pidx >> x >> y;
        if(((size_t) pidx) == npt && camidx.size() > i) 
        {
            camidx.resize(i);
            ptidx.resize(i);
            measurements.resize(i);
            std::cout << "Truncate measurements to " << i << '\n';
        }else if(((size_t) pidx) >= npt)
        {
            continue;
        }else
        {
            camidx[i] = cidx;    ptidx[i] = pidx;
            measurements[i].SetPoint2D(x, -y);
        }
    }

    for(size_t i = 0; i < ncam; ++i)
    {
        double p[9];
        for(int j = 0; j < 9; ++j) in >> p[j];
        CameraT& cam = camera_data[i];
        cam.SetFocalLength(p[6]);
        cam.SetInvertedRT(p, p + 3);
        cam.SetProjectionDistortion(p[7]); 
    }

    for(size_t i = 0; i < npt; ++i)
    {
        double pt[3];
        in >> pt[0] >> pt[1] >> pt[2];
        point_data[i].SetPoint(pt);
    }
    return true;
}

void SaveBundlerModel(const char* filename, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx)
{
    std::cout << "Saving model to " << filename << "...\n"; 
    ofstream out(filename);
    out << std::setprecision(12); //need enough precision
    out <<  camera_data.size() << ' ' << point_data.size() << ' ' << measurements.size() << '\n';
    for(size_t i = 0; i < measurements.size(); ++i)
    {
        out << camidx[i] << ' ' << ptidx[i] << ' ' << measurements[i].x << ' ' << -measurements[i].y << '\n';
    }

    for(size_t i = 0; i < camera_data.size(); ++i)
    {
        CameraT& cam = camera_data[i];
        double r[3], t[3]; cam.GetInvertedRT(r, t);
        out << r[0] << ' ' << r[1] << ' ' << r[2] << ' ' 
            << t[0] << ' ' << t[1] << ' ' << t[2] << ' ' << cam.f 
            << ' '  << cam.GetProjectionDistortion() << " 0\n";
    }

    for(size_t i = 0; i < point_data.size(); ++i)
    {
        Point3D& pt = point_data[i];
        out << pt.xyz[0] << ' ' << pt.xyz[1] << ' ' << pt.xyz[2] << '\n';
    }
}

bool LoadModelFile(const char* name, string dataname, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx, 
              vector<string>& names, vector<int>& ptc)
{
    if(name == NULL)return false;
    ifstream in(name); 

    std::cout << "Loading cameras/points: " << name <<"\n" ;
    if(!in.is_open()) return false;

    if(strstr(name, ".nvm"))return LoadNVM(in, dataname, camera_data, point_data, measurements, ptidx, camidx, names, ptc);
    else if(strstr(name, ".out")) return LoadBundlerOut(name, in, camera_data, point_data, measurements, ptidx, camidx, names, ptc);
    else return LoadBundlerModel(in, camera_data, point_data, measurements, ptidx, camidx);

    
}


float random_ratio(float percent)
{
    return (rand() % 101 - 50) * 0.02f * percent + 1.0f;
}

void AddNoise(vector<CameraT>& camera_data, vector<Point3D>& point_data, float percent)
{
    std::srand((unsigned int) time(NULL));
    for(size_t i = 0; i < camera_data.size(); ++i)
    {
        camera_data[i].f *= random_ratio(percent);
        camera_data[i].t[0] *= random_ratio(percent);
        camera_data[i].t[1] *= random_ratio(percent);
        camera_data[i].t[2] *= random_ratio(percent);
        double e[3];
        camera_data[i].GetRodriguesRotation(e);
        e[0] *= random_ratio(percent);
        e[1] *= random_ratio(percent);
        e[2] *= random_ratio(percent);
        camera_data[i].SetRodriguesRotation(e);
    }

    for(size_t i = 0; i < point_data.size(); ++i)
    {
        point_data[i].xyz[0] *= random_ratio(percent);
        point_data[i].xyz[1] *= random_ratio(percent);
        point_data[i].xyz[2] *= random_ratio(percent);
    }
}

void AddStableNoise(vector<CameraT>& camera_data, vector<Point3D>& point_data,
                    const vector<int>& ptidx, const vector<int>& camidx, float percent)
{
    ///
    std::srand((unsigned int) time(NULL));
    //do not modify the visibility status..
    vector<float> zz0(ptidx.size());
    vector<CameraT> backup = camera_data;
    vector<float> vx(point_data.size()), vy(point_data.size()), vz(point_data.size());
    for(size_t i = 0; i < point_data.size(); ++i)
    {
        Point3D& pt = point_data[i];
        vx[i] = pt.xyz[0];
        vy[i] = pt.xyz[1];
        vz[i] = pt.xyz[2];
    }

    //find out the median location of all the 3D points.
    size_t median_idx = point_data.size() / 2;

    std::nth_element(vx.begin(), vx.begin() + median_idx, vx.end());
    std::nth_element(vy.begin(), vy.begin() + median_idx, vy.end());
    std::nth_element(vz.begin(), vz.begin() + median_idx, vz.end());
    float cx = vx[median_idx], cy = vy[median_idx], cz = vz[median_idx];

    for(size_t i  = 0; i < ptidx.size(); ++i)
    {
        CameraT& cam = camera_data[camidx[i]];
        Point3D& pt  = point_data[ptidx[i]];
        zz0[i] = cam.m[2][0] * pt.xyz[0] + cam.m[2][1] * pt.xyz[1] + cam.m[2][2] * pt.xyz[2] + cam.t[2];
    }

    vector<float> z2 = zz0;    median_idx = ptidx.size() / 2;
    std::nth_element(z2.begin(), z2.begin() + median_idx, z2.end());
    float mz = z2[median_idx]; // median depth
    float dist_noise_base = mz * 0.2f;

    /////////////////////////////////////////////////
    //modify points first..
    for(size_t i = 0; i < point_data.size(); ++i)
    {
        Point3D& pt = point_data[i];
        pt.xyz[0] = pt.xyz[0] - cx + dist_noise_base * random_ratio(percent);
        pt.xyz[1] = pt.xyz[1] - cy + dist_noise_base * random_ratio(percent);
        pt.xyz[2] = pt.xyz[2] - cz + dist_noise_base * random_ratio(percent);
    }
    
    vector<bool> need_modification(camera_data.size(), true);
    int invalid_count = 0, modify_iteration = 1;

    do
    {
        if(invalid_count)  std::cout << "NOTE" << std::setw(2) << modify_iteration 
                                << ": modify " << invalid_count << " camera to fix visibility\n";

        //////////////////////////////////////////////////////
        for(size_t i = 0; i < camera_data.size(); ++i)
        {
            if(!need_modification[i])continue;
            CameraT & cam = camera_data[i];
            double e[3], c[3];   cam = backup[i];
            cam.f *= random_ratio(percent);

            ///////////////////////////////////////////////////////////
            cam.GetCameraCenter(c);
            c[0] = c[0] - cx + dist_noise_base * random_ratio(percent);
            c[1] = c[1] - cy + dist_noise_base * random_ratio(percent);
            c[2] = c[2] - cz + dist_noise_base * random_ratio(percent);

            ///////////////////////////////////////////////////////////
            cam.GetRodriguesRotation(e);
            e[0] *= random_ratio(percent);
            e[1] *= random_ratio(percent);
            e[2] *= random_ratio(percent);

            ///////////////////////////////////////////////////////////
            cam.SetRodriguesRotation(e);
            cam.SetCameraCenterAfterRotation(c);
        }
        vector<bool>  invalidc(camera_data.size(), false);

        invalid_count = 0;
        for(size_t i = 0; i < ptidx.size(); ++i)
        {
            int cid = camidx[i];
            if(need_modification[cid] ==false) continue;
            if(invalidc[cid])continue;
             CameraT& cam = camera_data[cid];
             Point3D& pt  = point_data[ptidx[i]];
             float z = cam.m[2][0] * pt.xyz[0] + cam.m[2][1] * pt.xyz[1] + cam.m[2][2] * pt.xyz[2] + cam.t[2];
             if (z * zz0[i] > 0)continue;
             if (zz0[i] == 0 && z > 0) continue;
             invalid_count++; 
             invalidc[cid]  = true;
        }

        need_modification = invalidc;
        modify_iteration++;

    }while(invalid_count && modify_iteration < 20);

}

void ExamineVisiblity(const char* input_filename )
{

    //////////////
    vector<CameraD> camera_data;
    vector<Point3B> point_data; 
    vector<int> ptidx, camidx;
    vector<Point2D> measurements;
    ifstream in (input_filename);
    LoadBundlerModel(in, camera_data, point_data, measurements, ptidx, camidx);

    ////////////////
    int count = 0; double d1 = 100, d2 = 100;
    std::cout << "checking visibility...\n";
    vector<double> zz(ptidx.size());
    for(size_t i  = 0; i < ptidx.size(); ++i)
    {
        CameraD& cam = camera_data[camidx[i]];
        Point3B& pt  = point_data[ptidx[i]];
        double dz = cam.m[2][0] * pt.xyz[0] + cam.m[2][1] * pt.xyz[1] + cam.m[2][2] * pt.xyz[2] + cam.t[2];
        //double dx = cam.m[0][0] * pt.xyz[0] + cam.m[0][1] * pt.xyz[1] + cam.m[0][2] * pt.xyz[2] + cam.t[0];
        //double dy = cam.m[1][0] * pt.xyz[0] + cam.m[1][1] * pt.xyz[1] + cam.m[1][2] * pt.xyz[2] + cam.t[1];

        ////////////////////////////////////////
        float c[3];       cam.GetCameraCenter(c);

        CameraT camt; camt.SetCameraT(cam);
        Point3D ptt; ptt.SetPoint(pt.xyz);
        double fz = camt.m[2][0] * ptt.xyz[0] + camt.m[2][1] * ptt.xyz[1] + camt.m[2][2] * ptt.xyz[2] + camt.t[2];
        double fz2 = camt.m[2][0] * (ptt.xyz[0] - c[0]) + camt.m[2][1] * (ptt.xyz[1] - c[1])
                                                       + camt.m[2][2] * (ptt.xyz[2] - c[2]);


        //if(dz == 0 && fz == 0) continue;

        if(dz * fz <= 0 || fz == 0)
        {
            std::cout << "cam " << camidx[i] //<<// "; dx = " << dx << "; dy = " << dy 
                      << "; double: " << dz << "; float " << fz << "; float2 " << fz2 << "\n";
            //std::cout << cam.m[2][0] << " "<<cam.m[2][1]<< " " <<  cam.m[2][2] << " "<<cam.t[2] << "\n";
            //std::cout << camt.m[2][0] << " "<<camt.m[2][1]<< " " <<  camt.m[2][2] << " "<<camt.t[2] << "\n";
            //std::cout << cam.m[2][0] - camt.m[2][0] << " " <<cam.m[2][1] - camt.m[2][1]<< " " 
            //          << cam.m[2][2] - camt.m[2][2] << " " <<cam.t[2] - camt.t[2]<< "\n";
        }

        zz[i] = dz;
        d1  = std::min(fabs(dz), d1);
        d2 = std::min(fabs(fz), d2);
    }

    std::cout   << count << " points moved to wrong side " 
                << d1 << ", " << d2  <<"\n";
}

bool RemoveInvisiblePoints( vector<CameraT>& camera_data, vector<Point3D>& point_data,
                            vector<int>& ptidx, vector<int>& camidx,
                            vector<Point2D>& measurements, vector<string>& names, vector<int>& ptc)
{
    vector<float> zz(ptidx.size());
    for(size_t i  = 0; i < ptidx.size(); ++i)
    {
        CameraT& cam = camera_data[camidx[i]];
        Point3D& pt  = point_data[ptidx[i]];
        zz[i] = cam.m[2][0] * pt.xyz[0] + cam.m[2][1] * pt.xyz[1] + cam.m[2][2] * pt.xyz[2] + cam.t[2];
    }
    size_t median_idx = ptidx.size() / 2;
    std::nth_element(zz.begin(), zz.begin() + median_idx, zz.end());
    float dist_threshold = zz[median_idx] * 0.001f;

    //keep removing 3D points. until all of them are infront of the cameras..
    vector<bool> pmask(point_data.size(), true);
    int points_removed = 0;
    for(size_t i  = 0; i < ptidx.size(); ++i)
    {
        int cid = camidx[i], pid = ptidx[i];
        if(!pmask[pid])continue;
        CameraT& cam = camera_data[cid];
        Point3D& pt  = point_data[pid];
        bool visible = (cam.m[2][0] * pt.xyz[0] + cam.m[2][1] * pt.xyz[1] + cam.m[2][2] * pt.xyz[2] + cam.t[2] > dist_threshold);
        pmask[pid] = visible; //this point should be removed
        if(!visible) points_removed++;
    }
    if(points_removed == 0) return false;
    vector<int>  cv(camera_data.size(), 0);
    //should any cameras be removed ?
    int min_observation = 20; //cameras should see at leat 20 points

    do
    {
        //count visible points for each camera
        std::fill(cv.begin(), cv.end(), 0);
        for(size_t i = 0; i < ptidx.size(); ++i)
        {
            int cid = camidx[i], pid = ptidx[i];
            if(pmask[pid])  cv[cid]++; 
        }

        //check if any more points should be removed
        vector<int>  pv(point_data.size(), 0);
        for(size_t i = 0; i < ptidx.size(); ++i)
        {
            int cid = camidx[i], pid = ptidx[i];
            if(!pmask[pid]) continue; //point already removed
            if(cv[cid] < min_observation) //this camera shall be removed.
            {
                ///
            }else
            {
                pv[pid]++;
            }
        }

        points_removed = 0;
        for(size_t i = 0; i < point_data.size(); ++i)
        {
            if(pmask[i] == false) continue;
            if(pv[i] >= 2) continue;
            pmask[i] = false;
            points_removed++;
        }
    }while(points_removed > 0);

    ////////////////////////////////////
    vector<bool> cmask(camera_data.size(), true);
    for(size_t i = 0; i < camera_data.size(); ++i) cmask[i] = cv[i] >= min_observation; 
    ////////////////////////////////////////////////////////

    vector<int> cidx(camera_data.size());
    vector<int> pidx(point_data.size());




    ///modified model.
    vector<CameraT> camera_data2;
    vector<Point3D> point_data2;
    vector<int> ptidx2;
    vector<int> camidx2;
    vector<Point2D> measurements2;
    vector<string> names2;
    vector<int> ptc2;


    //
    if(names.size() < camera_data.size()) names.resize(camera_data.size(),string("unknown"));
    if(ptc.size() < 3 * point_data.size()) ptc.resize(point_data.size() * 3, 0);

    //////////////////////////////
    int new_camera_count = 0, new_point_count = 0;
    for(size_t i = 0; i < camera_data.size(); ++i)
    {
        if(!cmask[i])continue;
        camera_data2.push_back(camera_data[i]);
        names2.push_back(names[i]);
        cidx[i] = new_camera_count++;
    }

    for(size_t i = 0; i < point_data.size(); ++i)
    {
        if(!pmask[i])continue;
        point_data2.push_back(point_data[i]);
        ptc.push_back(ptc[i]);
        pidx[i] = new_point_count++;
    }

    int new_observation_count = 0;
    for(size_t i = 0; i < ptidx.size(); ++i)
    {
        int pid = ptidx[i], cid = camidx[i];
        if(!pmask[pid] || ! cmask[cid]) continue;
        ptidx2.push_back(pidx[pid]);
        camidx2.push_back(cidx[cid]);
        measurements2.push_back(measurements[i]);
        new_observation_count++;
    }

    std::cout << "NOTE: removing " << (camera_data.size() - new_camera_count) << " cameras; "<< (point_data.size() - new_point_count) 
                << " 3D Points; " << (measurements.size() - new_observation_count) << " Observations;\n";

    camera_data2.swap(camera_data); names2.swap(names);
    point_data2.swap(point_data);   ptc2.swap(ptc);
    ptidx2.swap(ptidx);  camidx2.swap(camidx);
    measurements2.swap(measurements);

    return true;
}

void SaveModelFile(const char* outpath, vector<CameraT>& camera_data, vector<Point3D>& point_data,
              vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx, 
              vector<string>& names, vector<int>& ptc)
{
	if(outpath == NULL) return;
    if(strstr(outpath, ".nvm"))
            SaveNVM(outpath, camera_data, point_data, measurements, ptidx, camidx, names, ptc); 
	else if(strstr(outpath, ".out"))
			SaveBundlerOut(outpath, camera_data, point_data, measurements, ptidx, camidx, names, ptc);
       else      
            SaveBundlerModel(outpath,  camera_data, point_data, measurements, ptidx, camidx);

}
