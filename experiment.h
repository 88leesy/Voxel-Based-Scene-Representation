#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <time.h>
#include <iomanip>
#include <algorithm>

#define _USE_MATH_DEFINES //<cmath>���� M_PI ����Ϸ���...
#include <cmath> 
using namespace std;
#include "DataInterface.h"

#include "dbscan.h"
//#include "voxel_DB.h"

#include "PlaneRANSAC.h"
#include "kmeans.h"

#include <opencv2\opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2\cudafeatures2d.hpp>

#define NOMINMAX  //windows.h ��� ���Ͽ��� min�� max�� ��ó���� ��ũ�η� ���Ǹ� �ؼ� �߻��ϴ� ������ �ذ��ϱ� ����.

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
bool RemoveInvisiblePoints(vector<CameraT>& camera_data, vector<Point3D>& point_data,
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

	//micro ��ķ... �ְ���
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
	bool useExtrinsicGuess = true;
	//bool correspondence = cv::solvePnP(list_points3d, list_points2d, A_matrix, distCoeffs, rvec, tvec,
	//	useExtrinsicGuess, SOLVEPNP_ITERATIVE);
	bool correspondence = cv::solvePnPRansac(list_points3d, list_points2d, A_matrix, distCoeffs, rvec, tvec, useExtrinsicGuess);

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
	if (in.peek() == 'N')
	{
		in >> token; //file header
		if (strstr(token.c_str(), "R9T"))
		{
			rotation_parameter_num = 9;    //rotation as 3x3 matrix
			format_r9t = true;
		}
	}


	//kmeans();


	int ncam = 0, npoint = 0, nproj = 0;
	// read # of cameras
	in >> ncam;  if (ncam <= 1) return false;

	//read the camera parameters
	camera_data.resize(ncam); // allocate the camera data
	names.resize(ncam);
	float radial_distortion = 0;
	for (int i = 0; i < ncam; ++i)
	{
		double f, q[9], c[3], d[2];
		in >> token >> f;
		for (int j = 0; j < rotation_parameter_num; ++j) in >> q[j];
		in >> c[0] >> c[1] >> c[2] >> d[0] >> d[1];

		//q�� ���ʹϾ�, c�� translation


		camera_data[i].SetFocalLength(f);
		if (format_r9t)
		{
			camera_data[i].SetMatrixRotation(q);
			camera_data[i].SetTranslation(c);
		}
		else
		{
			//older format for compability -> ���ʹϾ�� ī�޶� �߽��� R�� T�� ��ȯ ������
			camera_data[i].SetQuaternionRotation(q);        //quaternion from the file
			camera_data[i].SetCameraCenterAfterRotation(c); //camera center from the file
		}
		camera_data[i].SetNormalizedMeasurementDistortion(d[0]);
		names[i] = token;
		radial_distortion = d[0];
	}

	//////////////////////////////////////
	in >> npoint;   if (npoint <= 0) return false;

	//read image projections and 3D points.
	//vector<Mat> featureImages;
	vector<vector<Point2d>> features_2d;
	vector<vector<Point3d>> features_3d;
	vector<Point3d> test_features_3d;
	//featureImages.resize(ncam);
	features_2d.resize(ncam);
	features_3d.resize(ncam);
	point_data.resize(npoint);
	for (int i = 0; i < npoint; ++i)
	{
		float pt[3]; int cc[3], npj;
		in >> pt[0] >> pt[1] >> pt[2]
			>> cc[0] >> cc[1] >> cc[2] >> npj;

		for (int j = 0; j < npj; ++j)
		{
			int cidx, fidx; float imx, imy;
			in >> cidx >> fidx >> imx >> imy;

			camidx.push_back(cidx);    //camera index
			ptidx.push_back(fidx);        //point index

			//add a measurment to the vector
			measurements.push_back(Point2D(imx, imy));
			nproj++;

			float pt_2d[2] = { 0 };
			camera_data[cidx].GetProjectionPoint(pt, pt_2d);

			features_2d[cidx].push_back(Point2d(imx, imy));
			features_3d[cidx].push_back(Point3d(pt[0], pt[1], pt[2]));

			//test_features_3d.push_back(Point3d(pt[0], pt[1], pt[2]));
		}

		//printf("i:%d\n", i);

		point_data[i].SetPoint(pt);
		ptc.insert(ptc.end(), cc, cc + 3);
	}

	printf("finish!\n");


	/*
	DBSCAN Clustering
	*/
#define MINIMUM_POINTS 80    // minimum number of cluster
	//#define EPSILON (0.75*0.75)  // distance for clustering, metre^2
#define EPSILON (0.08)  // distance for clustering, metre^2

	vector<DB_Point> points;
	int nLabel = 0;

	for (int i = 0; i < npoint; ++i)
	{
		DB_Point tmp_pt;
		tmp_pt.clusterID = UNCLASSIFIED;
		tmp_pt.x = point_data[i].xyz[0];
		tmp_pt.y = point_data[i].xyz[1];
		tmp_pt.z = point_data[i].xyz[2];
		points.push_back(tmp_pt);
	}

	//#define NUM_COLOR 300
	srand(0);
	float color[2000][3] = { 0 };
	for (int i = 0; i < 2000; i++)
	{
		color[i][0] = (rand() % 255);
		color[i][1] = (rand() % 255);
		color[i][2] = (rand() % 255);
	}

	float reSize_H = 256, reSize_W = 455;
	int cubeSize = 40;          //king-100(���߿� 120���� �غ���), old-30, shop-40, stmary-50(��� 70���� �غ���)

	DBSCAN ds(MINIMUM_POINTS, EPSILON, points, cubeSize);
	//ds.run();
	//ds.printResults(ds.m_points, ds.getTotalPointSize(), true);
	//ds.PlaneFitting(ds.m_points, ds.getTotalPointSize());
	//ds.voxelFitting(ds.m_points, ds.getTotalPointSize(), color, nLabel);

	//ds.voxelFitting2(ds.m_points, ds.getTotalPointSize(), color, nLabel);q
	ds.voxelFitting3(ds.m_points, ds.getTotalPointSize(), color, nLabel, cubeSize);  //��������.

	int num_points = ds.m_points.size();
	printf("\n num_points:%d\n", num_points);

	vector<DB_Point> tmp_points;
	for (int i = 0; i < num_points; ++i)
	{
		if (ds.m_points[i].clusterID != -1)
		{
			tmp_points.push_back(ds.m_points[i]);
		}
	}
	printf("tmp_points size:%d\n", tmp_points.size());
	printf("nLabel:%d\n", nLabel);

	/*
	label���� ����
	*/
	vector<Point3f> vec_centroid_points;
	vector<vector<DB_Point>> vec_label_points;
	vec_label_points.resize(nLabel);

	printf("-------------------------------------------\n");
	for (int i = 0; i < num_points; ++i)
	{
		int label = tmp_points[i].clusterID-1;
		vec_label_points[label].push_back(tmp_points[i]);
	}

	/*
	plane fitting���� inlier�� �̾Ƴ��� �����.
	*/
	vector<DB_Point> vec_inlierData;
	for (int i = 0; i < nLabel; ++i)
	{
		vector<DB_Point> tmp_inlierData;
		ds.PlaneFitting(vec_label_points[i], vec_label_points[i].size(), tmp_inlierData, color);
		for (int n = 0; n < tmp_inlierData.size(); ++n)
		{
			vec_inlierData.push_back(tmp_inlierData[n]);
		}

		// ���� ����
		vec_label_points[i].clear();
		vec_label_points[i].resize(tmp_inlierData.size());
		copy(tmp_inlierData.begin(), tmp_inlierData.end(), vec_label_points[i].begin());

	}
	printf("vec_inlierData:%d\n", vec_inlierData.size());
	ds.printResults(vec_inlierData, vec_inlierData.size(), true, color);
	


	printf("--------- centroid points extracting... -----------\n");
	int sumEachVoxelPtCnt = 0;
	for (int i = 0; i < nLabel; ++i)
	{
		float sumX = 0, sumY = 0, sumZ = 0;
		float avrX = 0, avrY = 0, avrZ = 0;
		for (int j = 0; j < vec_label_points[i].size(); ++j)
		{
			float x = vec_label_points[i][j].x;
			float y = vec_label_points[i][j].y;
			float z = vec_label_points[i][j].z;

			sumX += x; sumY += y; sumZ += z;
		}
		avrX = sumX / vec_label_points[i].size();
		avrY = sumY / vec_label_points[i].size();
		avrZ = sumZ / vec_label_points[i].size();

		Point3f centroidPt;
		int centId = -1;
		float minDist = 99999;
		for (int j = 0; j < vec_label_points[i].size(); ++j)
		{
			float x = vec_label_points[i][j].x;
			float y = vec_label_points[i][j].y;
			float z = vec_label_points[i][j].z;

			float dist = sqrt(pow(avrX - x, 2) + pow(avrY - y, 2) + pow(avrZ - z, 2));
			if (dist < minDist)
			{
				minDist = dist;
				centId = j;
			}
		}

		centroidPt.x = vec_label_points[i][centId].x;
		centroidPt.y = vec_label_points[i][centId].y;
		centroidPt.z = vec_label_points[i][centId].z;
		//centroidPt.x = avrX;
		//centroidPt.y = avrY;
		//centroidPt.z = avrZ;

		vec_centroid_points.push_back(centroidPt);

		sumEachVoxelPtCnt += vec_label_points[i].size();
		printf("centId:%d, minDist:%f, vec_label_points[i]:%d\n", centId, minDist, vec_label_points[i].size());
	}
	int avgEachVoxelPtCnt = sumEachVoxelPtCnt / nLabel;
	printf("sumEachVoxelPtCnt:%d, avgEachVoxelPtCnt:%d\n", sumEachVoxelPtCnt, avgEachVoxelPtCnt);
	printf("vec_centroid_points:%d\n", vec_centroid_points.size());

	/*
	Write 3d centroid points 
	*/
	string save_path = "F:/_voxelFeatureMap3/" + dataname + "_3d_nearest_centroid.txt";
	ofstream output(save_path);
	output << vec_centroid_points.size() << "\n";
	for (int i = 0; i < vec_centroid_points.size(); ++i)
	{
		float x = vec_centroid_points[i].x;
		float y = vec_centroid_points[i].y;
		float z = vec_centroid_points[i].z;

		output << x << " " << y << " " << z << "\n";
	}
	output.close();

	
	//printf("finish!\n");
	//exit(0);


	//�̹������� �ҷ�����
	for (int i = 0; i < ncam; ++i)
	{
		string img_path = "F:/RPNet_test/data/absolute_cambridge/" + dataname + names[i];
		//string img_path = "F:/" + dataname + names[i];  //Steet ��...

		cout << names[i] << endl;

		int nExt = img_path.rfind("jpg");
		int nName = img_path.rfind("/") + 1;
		string strModExt("png");
		string strReName;
		strReName = img_path.substr(0, nName);
		strReName += img_path.substr(nName, nExt - nName);
		strReName += strModExt;

		Mat img = imread(strReName);
		Mat down_img = img.clone();
		resize(down_img, down_img, Size(reSize_W, reSize_H));  //455, 256

		int c_w = img.cols / 2;
		int c_h = img.rows / 2;
		Mat label_img = Mat::zeros(down_img.rows, down_img.cols, CV_16U);
	
		int size = features_3d[i].size();


		/*
		clustering 3D Points�� ���� �̹����� ������ 3D Point�� ���� ������,
		�� �̹����� ������ �� 3D Point�� clustering 3D Points�� ���� �ο�.
		�׸��� �󺧺��� 2d Point�� �ٽ� ����. 
		-> �� �� Ŭ�����Ϳ� ����Ʈ ������ ������, Ư¡�� �׸�ŭ ���� ���̹Ƿ�, ���Ŀ� patch�μ� ���� ���� ��Ű�� ����.
		*/			
		vector<vector<DB_Point>> eachLabelPt;
		eachLabelPt.resize(vec_label_points.size());
		for (int j = 0; j < size; ++j)
		{
			for (int k = 0; k < vec_label_points.size(); ++k)
			{
				for (int l = 0; l < vec_label_points[k].size(); ++l)
				{
					if (features_3d[i][j].x == vec_label_points[k][l].x
						&& features_3d[i][j].y == vec_label_points[k][l].y
						&& features_3d[i][j].z == vec_label_points[k][l].z)
					{
						//int feature_x = features_2d[i][j].x + c_w + 0.5;
						//int feature_y = features_2d[i][j].y + c_h + 0.5;
						int feature_x = features_2d[i][j].x;
						int feature_y = features_2d[i][j].y;

						DB_Point tmp;
						int color_lb = vec_label_points[k][l].clusterID-1;
						tmp = vec_label_points[k][l];
						tmp.x_2d = feature_x;
						tmp.y_2d = feature_y;
						eachLabelPt[color_lb].push_back(tmp);
					}
				}

			}
		}

		int nlbCntInimg = 0;
		for (int j = 0; j < eachLabelPt.size(); ++j)
		{
			//if (eachLabelPt[j].size() < 5)
			//	eachLabelPt[j].clear();
			if (eachLabelPt[j].size() >= 5)
				++nlbCntInimg;
		}
		printf("nlbCntInimg:%d\n", nlbCntInimg);
		for (int j = 0; j < eachLabelPt.size(); ++j)
		{
			if (eachLabelPt[j].size() < 5)
				continue;
			vector<Point> contour, resized_contour;
			Scalar cvColor(color[j][0], color[j][1], color[j][2]);
			for (int k = 0; k < eachLabelPt[j].size(); ++k)
			{
				//int feature_x = eachLabelPt[j][k].x_2d;
				//int feature_y = eachLabelPt[j][k].y_2d;
				int feature_x = eachLabelPt[j][k].x_2d + c_w + 0.5;
				int feature_y = eachLabelPt[j][k].y_2d + c_h + 0.5;

				//circle(img, Point(feature_x, feature_y), 1, cvColor, -1);
				//contour.push_back(Point(feature_x, feature_y));

				feature_x = feature_x / (1920.0 / down_img.cols);
				feature_y = feature_y / (1080.0 / down_img.rows);

				circle(down_img, Point(feature_x, feature_y), 1, cvColor, -1);
				resized_contour.push_back(Point(feature_x, feature_y));

				//char txt[20];    // ũ�Ⱑ 20�� char�� �迭�� ����
				//sprintf(txt, "%d", j);
				//putText(img, txt, Point(feature_x, feature_y), CV_FONT_HERSHEY_SIMPLEX, 0.4, Scalar(b, g, r));

			}
			bool isClosed = true;

			//vector<Point> hull(contour.size());
			//convexHull(contour, hull);
			//const Point *pts = (const Point*)Mat(hull).data;
			//int npts = Mat(hull).rows;
			//polylines(img, &pts, &npts, 1, isClosed, cvColor, 1);
			////ushort val = eachLabelPt[j][0].clusterID;
			////fillPoly(label_img, &pts, &npts, 1, Scalar(val));

			vector<Point> hull(resized_contour.size());
			convexHull(resized_contour, hull);
			const Point *pts = (const Point*)Mat(hull).data;
			int npts = Mat(hull).rows;
			polylines(down_img, &pts, &npts, 1, isClosed, cvColor, 1);

			ushort val = eachLabelPt[j][0].clusterID;
			//printf("val:%d, j:%d\n", val, j);
			fillPoly(label_img, &pts, &npts, 1, Scalar(val));

		}

		//cv::Mat tmp2 = cv::imread("__asdf__.png", cv::IMREAD_ANYDEPTH); //IMREAD_ANYDEPTH
		//for (int y = 0; y < tmp2.rows; ++y)
		//{
		//	for (int x = 0; x < tmp2.cols; ++x)
		//	{
		//		if(tmp2.at<ushort>(y, x) > 255)
		//			printf("%hd\n", tmp2.at<ushort>(y,x)); // .at<type>(y, x)
		//	}
		//}

		//cv::Mat pic16bit;
		//label_img.convertTo(pic16bit, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
		//cv::namedWindow("asdfasdf");
		//cv::imshow("asdfasdf", pic16bit);
		//cv::waitKey(0);



		/*
			�� �̹����� �ش��ϴ� feature points Ȯ���ϱ�...
		*/
		Mat kp_label_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		for (int j = 0; j < size; ++j)
		{
			Scalar cvColor(255, 0, 0);
			int feature_x = features_2d[i][j].x + c_w + 0.5;
			int feature_y = features_2d[i][j].y + c_h + 0.5;

			feature_x = feature_x / (1920.0 / down_img.cols);
			feature_y = feature_y / (1080.0 / down_img.rows);

			//circle(kp_label_img, Point(feature_x, feature_y), 1, cvColor, -1);
			kp_label_img.data[feature_y*kp_label_img.cols + feature_x] = 255;
		}
		imshow("kp_label_img", kp_label_img);
		//waitKey(0);

		//string save_path = "F:/featuresMap_By_reprojetion_error_2/" + dataname + names[i];
		string save_path = "F:/_voxelFeatureMap3/" + dataname + names[i];
		string save_path2 = "F:/_voxelFeatureMap3/" + dataname + names[i];
		save_path.replace(save_path.find("jpg"), 3, "png");
		save_path2.replace(save_path2.find("frame"), 5, "kp_frame");
		save_path2.replace(save_path2.find("jpg"), 3, "png");
		cout << save_path << endl;
		////imshow("test rgb", img);
		imshow("test resize rgb", down_img);
		////imshow("test label", label_img);

		//imwrite(save_path, label_img);
		//imwrite(save_path2, kp_label_img);
		waitKey(0);
	}


	printf("DBSCAN finish!\n");



	/*
	KingsCollege, OldHospital, ShopFacade, StMarysChurch
	*/

	float error_T_sum = 0, error_R_sum = 0;
	for (int i = 0; i < ncam; ++i)
	{
		//string img_path = "F:/RPNet_test/data/absolute_cambridge/ShopFacade/sfm/" + names[i];
		string img_path = "F:/RPNet_test/data/absolute_cambridge/" + dataname + names[i];
		//string img_path = "F:/" + dataname + names[i];
		int nExt = img_path.rfind("jpg");
		int nName = img_path.rfind("/") + 1;
		string strModExt("png");
		string strReName;
		strReName = img_path.substr(0, nName);
		strReName += img_path.substr(nName, nExt - nName);
		strReName += strModExt;

		Mat img = imread(strReName);
		int c_w = img.cols / 2;
		int c_h = img.rows / 2;

		int size = features_2d[i].size();
		for (int j = 0; j < size; ++j)
		{
			int feature_x = features_2d[i][j].x + c_w + 0.5;
			int feature_y = features_2d[i][j].y + c_h + 0.5;
			//circle(img, Point(feature_x, feature_y), 4, Scalar(0, 0, 255), -1);
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


				//Ư¡���� �������� ����þȺ����� �̹��� ��ġ�� �����Ͽ�, �̹����� �����.
				//����þ� ����ũ�� ũ��� ���� 6*ǥ������-1�� ��
				float tmp = 3;
				if (fabs(reproj_err) < 0.1)		tmp = 10;
				else if (fabs(reproj_err) < 1.0) tmp = 7;
				else if (fabs(reproj_err) < 5) tmp = 4;

				//tmp = 1.0f / fabs(reproj_err);
				tmp = 1.0f / fabs(reproj_err) * 512;
				tmp = log2(tmp) + 5;
				//int nHalf = tmp;

				//float tmp2 = fabs( log2(fabs(reproj_err)) );
				float sigma = tmp;
				int nHalf = std::min(std::max((sigma * 6 - 1) / 2, 3.0f), 100.0f);
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

			} //size => features ����

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
			imshow("test", weighted_img);
			//imwrite(save_path, weighted_img);
			waitKey(0);
		}  //if correspondence


		//imshow(names[i], img);
		//imshow("ttt", img);
		//string save_path = "ShopFacade/" + names[i];
		//imwrite(save_path, img);
		//waitKey(0);
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
	if (names.size() < camera_data.size()) names.resize(camera_data.size(), string("unknown"));
	if (ptc.size() < 3 * point_data.size()) ptc.resize(point_data.size() * 3, 0);

	////////////////////////////////////
	for (size_t i = 0; i < camera_data.size(); ++i)
	{
		CameraT& cam = camera_data[i];
		out << names[i] << ' ' << cam.GetFocalLength() << ' ';
		for (int j = 0; j < 9; ++j) out << cam.m[0][j] << ' ';
		out << cam.t[0] << ' ' << cam.t[1] << ' ' << cam.t[2] << ' '
			<< cam.GetNormalizedMeasurementDistortion() << " 0\n";
	}

	out << point_data.size() << '\n';

	for (size_t i = 0, j = 0; i < point_data.size(); ++i)
	{
		Point3D& pt = point_data[i];
		int * pc = &ptc[i * 3];
		out << pt.xyz[0] << ' ' << pt.xyz[1] << ' ' << pt.xyz[2] << ' '
			<< pc[0] << ' ' << pc[1] << ' ' << pc[2] << ' ';

		size_t je = j;
		while (je < ptidx.size() && ptidx[je] == (int)i) je++;

		out << (je - j) << ' ';

		for (; j < je; ++j)    out << camidx[j] << ' ' << " 0 " << measurements[j].x << ' ' << measurements[j].y << ' ';

		out << '\n';
	}
}


bool LoadBundlerOut(const char* name, ifstream& in, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc)
{
	int rotation_parameter_num = 9;
	string token;
	while (in.peek() == '#') std::getline(in, token);

	char listpath[1024], filepath[1024];
	strcpy(listpath, name);
	char* ext = strstr(listpath, ".out");
	strcpy(ext, "-list.txt\0");

	///////////////////////////////////
	ifstream listin(listpath);
	if (!listin.is_open())
	{
		listin.close();       listin.clear();
		char * slash = strrchr(listpath, '/');
		if (slash == NULL) slash = strrchr(listpath, '\\');
		slash = slash ? slash + 1 : listpath;
		strcpy(slash, "image_list.txt");
		listin.open(listpath);
	}
	if (listin) std::cout << "Using image list: " << listpath << '\n';

	// read # of cameras
	int ncam = 0, npoint = 0, nproj = 0;
	in >> ncam >> npoint;
	if (ncam <= 1 || npoint <= 1) return false;
	std::cout << ncam << " cameras; " << npoint << " 3D points;\n";

	//read the camera parameters
	camera_data.resize(ncam); // allocate the camera data
	names.resize(ncam);

	bool det_checked = false;
	for (int i = 0; i < ncam; ++i)
	{
		float f, q[9], c[3], d[2];
		in >> f >> d[0] >> d[1];
		for (int j = 0; j < rotation_parameter_num; ++j) in >> q[j];
		in >> c[0] >> c[1] >> c[2];

		camera_data[i].SetFocalLength(f);
		camera_data[i].SetInvertedR9T(q, c);
		camera_data[i].SetProjectionDistortion(d[0]);

		if (listin >> filepath && f != 0)
		{
			char* slash = strrchr(filepath, '/');
			if (slash == NULL) slash = strchr(filepath, '\\');
			names[i] = (slash ? (slash + 1) : filepath);
			std::getline(listin, token);

			if (!det_checked)
			{
				float det = camera_data[i].GetRotationMatrixDeterminant();
				std::cout << "Check rotation matrix: " << det << '\n';
				det_checked = true;
			}
		}
		else
		{
			names[i] = "unknown";
		}
	}


	//read image projections and 3D points.
	point_data.resize(npoint);
	for (int i = 0; i < npoint; ++i)
	{
		float pt[3]; int cc[3], npj;
		in >> pt[0] >> pt[1] >> pt[2]
			>> cc[0] >> cc[1] >> cc[2] >> npj;
		for (int j = 0; j < npj; ++j)
		{
			int cidx, fidx; float imx, imy;
			in >> cidx >> fidx >> imx >> imy;

			camidx.push_back(cidx);    //camera index
			ptidx.push_back(i);        //point index

			//add a measurment to the vector
			measurements.push_back(Point2D(imx, -imy));
			nproj++;
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
	char* ext = strstr(listpath, ".out"); if (ext == NULL) return;
	strcpy(ext, "-list.txt\0");

	ofstream out(filename);
	out << "# Bundle file v0.3\n";
	out << std::setprecision(12); //need enough precision
	out << camera_data.size() << " " << point_data.size() << '\n';

	//save camera data
	for (size_t i = 0; i < camera_data.size(); ++i)
	{
		float q[9], c[3];
		CameraT& ci = camera_data[i];
		out << ci.GetFocalLength() << ' ' << ci.GetProjectionDistortion() << " 0\n";
		ci.GetInvertedR9T(q, c);
		for (int j = 0; j < 9; ++j) out << q[j] << (((j % 3) == 2) ? '\n' : ' ');
		out << c[0] << ' ' << c[1] << ' ' << c[2] << '\n';
	}
	///
	for (size_t i = 0, j = 0; i < point_data.size(); ++i)
	{
		int npj = 0, *ci = &ptc[i * 3]; Point3D& pt = point_data[i];
		while (j + npj < point_data.size() && ptidx[j + npj] == ptidx[j]) npj++;
		///////////////////////////
		out << pt.xyz[0] << ' ' << pt.xyz[1] << ' ' << pt.xyz[2] << '\n';
		out << ci[0] << ' ' << ci[1] << ' ' << ci[2] << '\n';
		out << npj << ' ';
		for (int k = 0; k < npj; ++k) out << camidx[j + k] << " 0 "
			<< measurements[j + k].x << ' ' << -measurements[j + k].y << '\n';
		out << '\n'; j += npj;
	}

	ofstream listout(listpath);
	for (size_t i = 0; i < names.size(); ++i) listout << names[i] << '\n';
}

template<class CameraT, class Point3D>
bool LoadBundlerModel(ifstream& in, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx)
{
	// read bundle data from a file
	size_t ncam = 0, npt = 0, nproj = 0;
	if (!(in >> ncam >> npt >> nproj)) return false;
	///////////////////////////////////////////////////////////////////////////////
	std::cout << ncam << " cameras; " << npt << " 3D points; " << nproj << " projections\n";


	camera_data.resize(ncam);
	point_data.resize(npt);
	measurements.resize(nproj);
	camidx.resize(nproj);
	ptidx.resize(nproj);

	for (size_t i = 0; i < nproj; ++i)
	{
		double x, y;    int cidx, pidx;
		in >> cidx >> pidx >> x >> y;
		if (((size_t)pidx) == npt && camidx.size() > i)
		{
			camidx.resize(i);
			ptidx.resize(i);
			measurements.resize(i);
			std::cout << "Truncate measurements to " << i << '\n';
		}
		else if (((size_t)pidx) >= npt)
		{
			continue;
		}
		else
		{
			camidx[i] = cidx;    ptidx[i] = pidx;
			measurements[i].SetPoint2D(x, -y);
		}
	}

	for (size_t i = 0; i < ncam; ++i)
	{
		double p[9];
		for (int j = 0; j < 9; ++j) in >> p[j];
		CameraT& cam = camera_data[i];
		cam.SetFocalLength(p[6]);
		cam.SetInvertedRT(p, p + 3);
		cam.SetProjectionDistortion(p[7]);
	}

	for (size_t i = 0; i < npt; ++i)
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
	out << camera_data.size() << ' ' << point_data.size() << ' ' << measurements.size() << '\n';
	for (size_t i = 0; i < measurements.size(); ++i)
	{
		out << camidx[i] << ' ' << ptidx[i] << ' ' << measurements[i].x << ' ' << -measurements[i].y << '\n';
	}

	for (size_t i = 0; i < camera_data.size(); ++i)
	{
		CameraT& cam = camera_data[i];
		double r[3], t[3]; cam.GetInvertedRT(r, t);
		out << r[0] << ' ' << r[1] << ' ' << r[2] << ' '
			<< t[0] << ' ' << t[1] << ' ' << t[2] << ' ' << cam.f
			<< ' ' << cam.GetProjectionDistortion() << " 0\n";
	}

	for (size_t i = 0; i < point_data.size(); ++i)
	{
		Point3D& pt = point_data[i];
		out << pt.xyz[0] << ' ' << pt.xyz[1] << ' ' << pt.xyz[2] << '\n';
	}
}

bool LoadModelFile(const char* name, string dataname, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc)
{
	if (name == NULL)return false;
	ifstream in(name);

	std::cout << "Loading cameras/points: " << name << "\n";
	if (!in.is_open()) return false;

	if (strstr(name, ".nvm"))return LoadNVM(in, dataname, camera_data, point_data, measurements, ptidx, camidx, names, ptc);
	else if (strstr(name, ".out")) return LoadBundlerOut(name, in, camera_data, point_data, measurements, ptidx, camidx, names, ptc);
	else return LoadBundlerModel(in, camera_data, point_data, measurements, ptidx, camidx);


}


float random_ratio(float percent)
{
	return (rand() % 101 - 50) * 0.02f * percent + 1.0f;
}

void AddNoise(vector<CameraT>& camera_data, vector<Point3D>& point_data, float percent)
{
	std::srand((unsigned int)time(NULL));
	for (size_t i = 0; i < camera_data.size(); ++i)
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

	for (size_t i = 0; i < point_data.size(); ++i)
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
	std::srand((unsigned int)time(NULL));
	//do not modify the visibility status..
	vector<float> zz0(ptidx.size());
	vector<CameraT> backup = camera_data;
	vector<float> vx(point_data.size()), vy(point_data.size()), vz(point_data.size());
	for (size_t i = 0; i < point_data.size(); ++i)
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

	for (size_t i = 0; i < ptidx.size(); ++i)
	{
		CameraT& cam = camera_data[camidx[i]];
		Point3D& pt = point_data[ptidx[i]];
		zz0[i] = cam.m[2][0] * pt.xyz[0] + cam.m[2][1] * pt.xyz[1] + cam.m[2][2] * pt.xyz[2] + cam.t[2];
	}

	vector<float> z2 = zz0;    median_idx = ptidx.size() / 2;
	std::nth_element(z2.begin(), z2.begin() + median_idx, z2.end());
	float mz = z2[median_idx]; // median depth
	float dist_noise_base = mz * 0.2f;

	/////////////////////////////////////////////////
	//modify points first..
	for (size_t i = 0; i < point_data.size(); ++i)
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
		if (invalid_count)  std::cout << "NOTE" << std::setw(2) << modify_iteration
			<< ": modify " << invalid_count << " camera to fix visibility\n";

		//////////////////////////////////////////////////////
		for (size_t i = 0; i < camera_data.size(); ++i)
		{
			if (!need_modification[i])continue;
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
		for (size_t i = 0; i < ptidx.size(); ++i)
		{
			int cid = camidx[i];
			if (need_modification[cid] == false) continue;
			if (invalidc[cid])continue;
			CameraT& cam = camera_data[cid];
			Point3D& pt = point_data[ptidx[i]];
			float z = cam.m[2][0] * pt.xyz[0] + cam.m[2][1] * pt.xyz[1] + cam.m[2][2] * pt.xyz[2] + cam.t[2];
			if (z * zz0[i] > 0)continue;
			if (zz0[i] == 0 && z > 0) continue;
			invalid_count++;
			invalidc[cid] = true;
		}

		need_modification = invalidc;
		modify_iteration++;

	} while (invalid_count && modify_iteration < 20);

}

void ExamineVisiblity(const char* input_filename)
{

	//////////////
	vector<CameraD> camera_data;
	vector<Point3B> point_data;
	vector<int> ptidx, camidx;
	vector<Point2D> measurements;
	ifstream in(input_filename);
	LoadBundlerModel(in, camera_data, point_data, measurements, ptidx, camidx);

	////////////////
	int count = 0; double d1 = 100, d2 = 100;
	std::cout << "checking visibility...\n";
	vector<double> zz(ptidx.size());
	for (size_t i = 0; i < ptidx.size(); ++i)
	{
		CameraD& cam = camera_data[camidx[i]];
		Point3B& pt = point_data[ptidx[i]];
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

		if (dz * fz <= 0 || fz == 0)
		{
			std::cout << "cam " << camidx[i] //<<// "; dx = " << dx << "; dy = " << dy 
				<< "; double: " << dz << "; float " << fz << "; float2 " << fz2 << "\n";
			//std::cout << cam.m[2][0] << " "<<cam.m[2][1]<< " " <<  cam.m[2][2] << " "<<cam.t[2] << "\n";
			//std::cout << camt.m[2][0] << " "<<camt.m[2][1]<< " " <<  camt.m[2][2] << " "<<camt.t[2] << "\n";
			//std::cout << cam.m[2][0] - camt.m[2][0] << " " <<cam.m[2][1] - camt.m[2][1]<< " " 
			//          << cam.m[2][2] - camt.m[2][2] << " " <<cam.t[2] - camt.t[2]<< "\n";
		}

		zz[i] = dz;
		d1 = std::min(fabs(dz), d1);
		d2 = std::min(fabs(fz), d2);
	}

	std::cout << count << " points moved to wrong side "
		<< d1 << ", " << d2 << "\n";
}

bool RemoveInvisiblePoints(vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<int>& ptidx, vector<int>& camidx,
	vector<Point2D>& measurements, vector<string>& names, vector<int>& ptc)
{
	vector<float> zz(ptidx.size());
	for (size_t i = 0; i < ptidx.size(); ++i)
	{
		CameraT& cam = camera_data[camidx[i]];
		Point3D& pt = point_data[ptidx[i]];
		zz[i] = cam.m[2][0] * pt.xyz[0] + cam.m[2][1] * pt.xyz[1] + cam.m[2][2] * pt.xyz[2] + cam.t[2];
	}
	size_t median_idx = ptidx.size() / 2;
	std::nth_element(zz.begin(), zz.begin() + median_idx, zz.end());
	float dist_threshold = zz[median_idx] * 0.001f;

	//keep removing 3D points. until all of them are infront of the cameras..
	vector<bool> pmask(point_data.size(), true);
	int points_removed = 0;
	for (size_t i = 0; i < ptidx.size(); ++i)
	{
		int cid = camidx[i], pid = ptidx[i];
		if (!pmask[pid])continue;
		CameraT& cam = camera_data[cid];
		Point3D& pt = point_data[pid];
		bool visible = (cam.m[2][0] * pt.xyz[0] + cam.m[2][1] * pt.xyz[1] + cam.m[2][2] * pt.xyz[2] + cam.t[2] > dist_threshold);
		pmask[pid] = visible; //this point should be removed
		if (!visible) points_removed++;
	}
	if (points_removed == 0) return false;
	vector<int>  cv(camera_data.size(), 0);
	//should any cameras be removed ?
	int min_observation = 20; //cameras should see at leat 20 points

	do
	{
		//count visible points for each camera
		std::fill(cv.begin(), cv.end(), 0);
		for (size_t i = 0; i < ptidx.size(); ++i)
		{
			int cid = camidx[i], pid = ptidx[i];
			if (pmask[pid])  cv[cid]++;
		}

		//check if any more points should be removed
		vector<int>  pv(point_data.size(), 0);
		for (size_t i = 0; i < ptidx.size(); ++i)
		{
			int cid = camidx[i], pid = ptidx[i];
			if (!pmask[pid]) continue; //point already removed
			if (cv[cid] < min_observation) //this camera shall be removed.
			{
				///
			}
			else
			{
				pv[pid]++;
			}
		}

		points_removed = 0;
		for (size_t i = 0; i < point_data.size(); ++i)
		{
			if (pmask[i] == false) continue;
			if (pv[i] >= 2) continue;
			pmask[i] = false;
			points_removed++;
		}
	} while (points_removed > 0);

	////////////////////////////////////
	vector<bool> cmask(camera_data.size(), true);
	for (size_t i = 0; i < camera_data.size(); ++i) cmask[i] = cv[i] >= min_observation;
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
	if (names.size() < camera_data.size()) names.resize(camera_data.size(), string("unknown"));
	if (ptc.size() < 3 * point_data.size()) ptc.resize(point_data.size() * 3, 0);

	//////////////////////////////
	int new_camera_count = 0, new_point_count = 0;
	for (size_t i = 0; i < camera_data.size(); ++i)
	{
		if (!cmask[i])continue;
		camera_data2.push_back(camera_data[i]);
		names2.push_back(names[i]);
		cidx[i] = new_camera_count++;
	}

	for (size_t i = 0; i < point_data.size(); ++i)
	{
		if (!pmask[i])continue;
		point_data2.push_back(point_data[i]);
		ptc.push_back(ptc[i]);
		pidx[i] = new_point_count++;
	}

	int new_observation_count = 0;
	for (size_t i = 0; i < ptidx.size(); ++i)
	{
		int pid = ptidx[i], cid = camidx[i];
		if (!pmask[pid] || !cmask[cid]) continue;
		ptidx2.push_back(pidx[pid]);
		camidx2.push_back(cidx[cid]);
		measurements2.push_back(measurements[i]);
		new_observation_count++;
	}

	std::cout << "NOTE: removing " << (camera_data.size() - new_camera_count) << " cameras; " << (point_data.size() - new_point_count)
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
	if (outpath == NULL) return;
	if (strstr(outpath, ".nvm"))
		SaveNVM(outpath, camera_data, point_data, measurements, ptidx, camidx, names, ptc);
	else if (strstr(outpath, ".out"))
		SaveBundlerOut(outpath, camera_data, point_data, measurements, ptidx, camidx, names, ptc);
	else
		SaveBundlerModel(outpath, camera_data, point_data, measurements, ptidx, camidx);

}