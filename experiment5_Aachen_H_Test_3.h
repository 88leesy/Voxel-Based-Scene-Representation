#pragma once


#include <math.h>
#include <time.h>
#include <iomanip>

#define _USE_MATH_DEFINES //<cmath>에서 M_PI 사용하려고...
#include <cmath> 
using namespace std;
#include "DataInterface.h"

#include "H_voxel_DB.h"
//#include "voxel_DB.h"
#//include "kmeans.h"
//#include "dbscan.h"

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

#define PI 3.14159265359

vector<string> split(string str, char delimiter) {
	vector<string> internal;
	stringstream ss(str);
	string temp;

	while (getline(ss, temp, delimiter)) {
		internal.push_back(temp);
	}

	return internal;
}

//File loader supports .nvm format and bundler format
bool LoadModelFile(const char* name, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc, int cubeSize);
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
	vector<Point2D>& measurements, vector<string>& names, vector<int>& ptc, vector<DB_Point_3D> & vec_db_pt_3d);

void GetQuaternionRotationByPnP(cv::Mat R_matrix, cv::Mat t_matrix, float q2[4]);
bool EstimatePoseByPnP(vector<Point2d> &features_2d, vector<Point3d> &features_3d, double focal_length, double coef,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix);

void plane_normVec(DB_Plane& plane);

bool EstimatePoseByPnP(vector<Point2d> &list_points2d, vector<Point3d> &list_points3d, double focal_length, double coef,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix)
{
	double f = focal_length;
	//const double params[] = { f,   // fx
	//						  f,  // fy
	//						  0,      // cx	
	//						  0 };    // cy

	//A_matrix.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
	//A_matrix.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
	//A_matrix.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
	//A_matrix.at<double>(1, 2) = params[3];
	//A_matrix.at<double>(2, 2) = 1;

	//micro 웹캠... 왜곡계수
	//double k1 = 0.022774;
	//double k2 = -0.041311;
	//double p1 = -0.0055;
	//double p2 = -0.0009367;
	double k1 = coef;
	double k2 = -0;
	double p1 = -0;
	double p2 = -0;
	double d[] = { k1, k2, p1, p2 };
	cv::Mat distCoeffs(4, 1, CV_64FC1, d);

	cv::Mat inliers;
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	bool useExtrinsicGuess = true;
	/*bool correspondence = cv::solvePnP(list_points3d, list_points2d, A_matrix, distCoeffs, rvec, tvec,
		useExtrinsicGuess, SOLVEPNP_ITERATIVE);*/
	bool correspondence = cv::solvePnPRansac(list_points3d, list_points2d, A_matrix, distCoeffs, rvec, tvec, false,
		5000, 10, 0.99, inliers, SOLVEPNP_P3P);

	if (correspondence)
	{
		//int index = inliers.at<int>(5);
		//printf("index:%d, inliers.size()_H:%d, inliers.size()_W:%d\n", index, inliers.size().height, inliers.size().width);

		int num_inliers = inliers.size().height;
		//int num_inliers2 = inliers.size().width;
		if (num_inliers < 5) // 최소 5개 보단 많아야 함.
			return false;

		vector<Point2d> refi_list_points2d; vector<Point3d> refi_list_points3d;
		for (int i = 0; i < num_inliers; ++i)
		{
			/*refi_list_points2d.push_back(list_points2d[inliers.at<int>(i)]);
			refi_list_points3d.push_back(list_points3d[inliers.at<int>(i)]);*/
			refi_list_points2d.emplace_back(list_points2d[inliers.at<int>(i)]);
			refi_list_points3d.emplace_back(list_points3d[inliers.at<int>(i)]);
		}

		bool correspondence = cv::solvePnP(refi_list_points3d, refi_list_points2d, A_matrix, distCoeffs, rvec, tvec,
			useExtrinsicGuess, SOLVEPNP_ITERATIVE);

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

//  1 : nvm 3d 좌표에서 사영된 2d points로 label image 만든 것으로, 2d-3d corresponding 관계 구한것.
//  2 : fitting plane 안에 속하는 3d points에서 min,max XYZ 구한 꼭지점을 사영한 2d points로 label image 만든 것으로, 2d-3d corresponding 관계 구한것.
#define Voxel_Labeling_Method 1


/////////////////////////////////////////////////////////////////////////////
bool LoadNVM(ifstream& in, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc, int cubeSize)
{

#define NUM_COLOR 10000
	srand(0);
	int(*color)[3] = new int[NUM_COLOR][3];
	for (int i = 0; i < NUM_COLOR; i++)
	{
		color[i][0] = (rand() % 255);
		color[i][1] = (rand() % 255);
		color[i][2] = (rand() % 255);
	}

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

		//q는 쿼터니언, c는 translation


		camera_data[i].SetFocalLength(f);
		if (format_r9t)
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
		camera_data[i].radial = d[0];
		names[i] = token;
		radial_distortion = d[0];


		//string absPath2;
		//absPath2 = "F:/_voxelFeatureMap_Aachen2/_single_Voxel_set/";
		//vector<string> _line_str = split(names[0], '/');
		//string save_path_rgb2 = absPath2 + _line_str[1] + ".png";
		//string _save_path = absPath2 + _line_str[1] + "_label_" + ".png";
		//string _save_path2 = absPath2 + _line_str[1] + "_kp_" + ".png";
		//string _save_path3 = absPath2 + _line_str[1] + "_3dpts" + ".yml";
		//string _save_path4 = absPath2 + _line_str[1] + "_contour" + ".png";
		//string _save_path5 = absPath2 + _line_str[1] + "_fillcontour" + ".png";
		//cout << _save_path << endl;
	}

	//////////////////////////////////////
	in >> npoint;   if (npoint <= 0) return false;

	//read image projections and 3D points.
	vector<H_Voxels> vec_db_pt_3d;
	vec_db_pt_3d.resize(1);
	vec_db_pt_3d[0].nLabel = -1;
	vec_db_pt_3d[0].nLevel = 0;
	vec_db_pt_3d[0].vec_pt_3d_db.clear();


	vector<vector<Point2d>> features_2d;
	vector<vector<Point3d>> features_3d;
	vector<Point3d> test_features_3d;
	//featureImages.resize(ncam);
	features_2d.resize(ncam);
	features_3d.resize(ncam);
	point_data.resize(npoint);
	for (int i = 0; i < npoint; ++i)
	{
		DB_Point_3D db_pt_3d;

		float pt[3]; int cc[3], npj;
		in >> pt[0] >> pt[1] >> pt[2]
			>> cc[0] >> cc[1] >> cc[2] >> npj;

		db_pt_3d.setXYZ(pt[0], pt[1], pt[2]);
		db_pt_3d.pt_3d_id = i;

		for (int j = 0; j < npj; ++j)
		{
			int cidx, fidx; float imx, imy;
			in >> cidx >> fidx >> imx >> imy;

			camidx.push_back(cidx);    //camera index
			ptidx.push_back(fidx);        //point index

			//add a measurment to the vector
			measurements.push_back(Point2D(imx, imy));  //emplace_back
			nproj++;

			//float pt_2d[2] = { 0 };
			//camera_data[cidx].GetProjectionPoint(pt, pt_2d);

			features_2d[cidx].push_back(Point2d(imx, imy));
			features_3d[cidx].push_back(Point3d(pt[0], pt[1], pt[2]));

			db_pt_3d.vec_2d_pt.push_back(Point_2D(imx, imy));
			db_pt_3d.pt_2d_ids.push_back(fidx);
			db_pt_3d.img_IDs.push_back(cidx);
		}

		//printf("i:%d\n", i);

		point_data[i].SetPoint(pt);
		ptc.insert(ptc.end(), cc, cc + 3);

		vec_db_pt_3d[0].vec_pt_3d_db.push_back(db_pt_3d);

	}

	printf("finish!\n");


	if (0)
	{
		int nSmall_err_cnt2 = 0;
		int nMid_err_cnt2 = 0;
		int nLarge_err_cnt2 = 0;

		for (int idx = 0; idx < ncam; ++idx)
		{
			string img_path = "F:/Aachen-Day-Night dataset/images/images_upright/" + names[idx];

			cout << names[idx] << endl;
			printf("features_2d[idx].size():%d\n", features_2d[idx].size());

			Mat img = imread(img_path);


			if (features_2d[idx].size() < 10)
			{
				for (int k = 0; k < features_2d[idx].size(); ++k)
				{
					int feature_x = features_2d[idx][k].x;
					int feature_y = features_2d[idx][k].y;

					circle(img, Point(feature_x, feature_y), 1, Scalar(0, 0, 255), -1);
				}
				imshow("asdf", img);
				waitKey(0);
			}
			if (features_2d[idx].size() < 4)
				continue;

			//vector<DB_Point> eachLabelPt;
			//vector<Point2d> tmp_features_2d;
			//vector<Point3d> tmp_features_3d;
			//int size = vec_db_pt_3d[0].vec_pt_3d_db.size();
			//for (int i = 0; i < size; ++i)
			//{
			//	int imgSize = vec_db_pt_3d[0].vec_pt_3d_db[i].img_IDs.size();

			//	for (int m = 0; m < imgSize; ++m)
			//	{
			//		if (vec_db_pt_3d[0].vec_pt_3d_db[i].img_IDs[m] == idx)  //vec_db_pt_3d[0].vec_pt_3d_db[i].img_IDs[m], vec_label_points[l].img_IDs[m]
			//		{
			//			float feature_x = vec_db_pt_3d[0].vec_pt_3d_db[i].vec_2d_pt[m].x;
			//			float feature_y = vec_db_pt_3d[0].vec_pt_3d_db[i].vec_2d_pt[m].y;

			//			DB_Point tmp_db;
			//			tmp_db.x = vec_db_pt_3d[0].vec_pt_3d_db[i].x;
			//			tmp_db.y = vec_db_pt_3d[0].vec_pt_3d_db[i].y;
			//			tmp_db.z = vec_db_pt_3d[0].vec_pt_3d_db[i].z;
			//			tmp_db.x_2d = feature_x;
			//			tmp_db.y_2d = feature_y;
			//			tmp_db.pt_3d_id = vec_db_pt_3d[0].vec_pt_3d_db[i].pt_3d_id;
			//			tmp_db.pt_2d_id = vec_db_pt_3d[0].vec_pt_3d_db[i].pt_2d_ids[m];

			//			eachLabelPt.push_back(tmp_db);
			//			tmp_features_2d.push_back(Point2d(feature_x, feature_y));
			//			tmp_features_3d.push_back(Point3d(tmp_db.x, tmp_db.y, tmp_db.z));
			//		}
			//	}
			//}

			cv::Mat R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
			cv::Mat t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix
			cv::Mat A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
			cv::Mat _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);
			cv::Mat EST_P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);
			const double params[] = { camera_data[idx].f,   // fx
									  camera_data[idx].f,  // fy
									  img.cols / 2,      // cx	
									  img.rows / 2 };    // cy
			//const double params[] = { camera_data[idx].f,   // fx
			//				  camera_data[idx].f,  // fy
			//				  0,      // cx	
			//				  0 };    // cy

			A_matrix.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
			A_matrix.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
			A_matrix.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
			A_matrix.at<double>(1, 2) = params[3];
			A_matrix.at<double>(2, 2) = 1;
			_P_matrix.at<double>(0, 0) = camera_data[idx].m[0][0];
			_P_matrix.at<double>(0, 1) = camera_data[idx].m[0][1];
			_P_matrix.at<double>(0, 2) = camera_data[idx].m[0][2];
			_P_matrix.at<double>(1, 0) = camera_data[idx].m[1][0];
			_P_matrix.at<double>(1, 1) = camera_data[idx].m[1][1];
			_P_matrix.at<double>(1, 2) = camera_data[idx].m[1][2];
			_P_matrix.at<double>(2, 0) = camera_data[idx].m[2][0];
			_P_matrix.at<double>(2, 1) = camera_data[idx].m[2][1];
			_P_matrix.at<double>(2, 2) = camera_data[idx].m[2][2];
			_P_matrix.at<double>(0, 3) = camera_data[idx].t[0];
			_P_matrix.at<double>(1, 3) = camera_data[idx].t[1];
			_P_matrix.at<double>(2, 3) = camera_data[idx].t[2];

			cv::Mat AP_mat = cv::Mat(4, 3, CV_64FC1);
			AP_mat = A_matrix * _P_matrix;
			//bool correspondence = EstimatePoseByPnP(tmp_features_2d, tmp_features_3d, camera_data[idx].f, camera_data[idx].radial, A_matrix, R_matrix, t_matrix);
			bool correspondence = EstimatePoseByPnP(features_2d[idx], features_3d[idx], camera_data[idx].f, camera_data[idx].radial, A_matrix, R_matrix, t_matrix);

			EST_P_matrix.at<double>(0, 0) = R_matrix.at<double>(0, 0);
			EST_P_matrix.at<double>(0, 1) = R_matrix.at<double>(0, 1);
			EST_P_matrix.at<double>(0, 2) = R_matrix.at<double>(0, 2);
			EST_P_matrix.at<double>(1, 0) = R_matrix.at<double>(1, 0);
			EST_P_matrix.at<double>(1, 1) = R_matrix.at<double>(1, 1);
			EST_P_matrix.at<double>(1, 2) = R_matrix.at<double>(1, 2);
			EST_P_matrix.at<double>(2, 0) = R_matrix.at<double>(2, 0);
			EST_P_matrix.at<double>(2, 1) = R_matrix.at<double>(2, 1);
			EST_P_matrix.at<double>(2, 2) = R_matrix.at<double>(2, 2);
			EST_P_matrix.at<double>(0, 3) = t_matrix.at<double>(0);
			EST_P_matrix.at<double>(1, 3) = t_matrix.at<double>(1);
			EST_P_matrix.at<double>(2, 3) = t_matrix.at<double>(2);


			double t1 = t_matrix.at<double>(0);
			double t2 = t_matrix.at<double>(1);
			double t3 = t_matrix.at<double>(2);

			float est_q[4];
			GetQuaternionRotationByPnP(R_matrix, t_matrix, est_q);

			float gt_q[4];
			camera_data[idx].GetQuaternionRotation(gt_q);

			float dot_prod = est_q[0] * gt_q[0] + est_q[1] * gt_q[1] + est_q[2] * gt_q[2] + est_q[3] * gt_q[3];
			if (dot_prod > 1) dot_prod = 1;
			if (dot_prod < -1) dot_prod = -1;
			float theta2 = 2 * acos(dot_prod) * 180 / M_PI;

			float error_x = sqrt(pow(t1 - camera_data[idx].t[0], 2) + pow(t2 - camera_data[idx].t[1], 2) + pow(t3 - camera_data[idx].t[2], 2));
			printf("gt - id:%d, Error XYZ (m):%f, theta:%f\n", idx, error_x, theta2);

			if (0.25 >= error_x && 2.0 >= theta2)
			{
				++nSmall_err_cnt2;
			}
			if (0.5 >= error_x && 5.0 >= theta2)
			{
				++nMid_err_cnt2;
			}
			if (5.0 >= error_x && 10.0 >= theta2)
			{
				++nLarge_err_cnt2;
			}


			//reprojection points

			//std::vector<cv::Point2f> projections;
			//cv::projectPoints(features_3d[idx], rotHyp, tHyp, A_matrix, cv::Mat(), projections);

			vector<float> vec_GT_error;
			vector<float> vec_EST_error;
			vector<float> vec_RT_pts_error;
			for (int ind = 0; ind < features_3d[idx].size(); ++ind)
				//for (int ind = 0; ind < tmp_features_3d.size(); ++ind)
			{
				//GT - R|t 곱한값
				cv::Mat Point3f_vec2 = cv::Mat(4, 1, CV_64FC1);
				Point3f_vec2.at<double>(0) = features_3d[idx][ind].x;
				Point3f_vec2.at<double>(1) = features_3d[idx][ind].y;
				Point3f_vec2.at<double>(2) = features_3d[idx][ind].z;
				//Point3f_vec2.at<double>(0) = tmp_features_3d[ind].x;  //똑같음
				//Point3f_vec2.at<double>(1) = tmp_features_3d[ind].y;
				//Point3f_vec2.at<double>(2) = tmp_features_3d[ind].z;
				Point3f_vec2.at<double>(3) = 1;
				cv::Mat Point2f_vec2 = cv::Mat(3, 1, CV_64FC1);
				Point2f_vec2 = AP_mat * Point3f_vec2;

				cv::Point2d Point2f;
				Point2f.x = Point2f_vec2.at<double>(0) / Point2f_vec2.at<double>(2);
				Point2f.y = Point2f_vec2.at<double>(1) / Point2f_vec2.at<double>(2);
				int xxx = Point2f.x + 0.5;
				int yyy = Point2f.y + 0.5;
				circle(img, Point(xxx, yyy), 1, Scalar(255, 0, 0), -1);

				//EST - R|t 곱한값
				cv::Mat Point2f_vec = cv::Mat(3, 1, CV_64FC1);
				Point2f_vec = A_matrix * EST_P_matrix * Point3f_vec2;

				cv::Point2d Point2f_est;
				Point2f_est.x = Point2f_vec.at<double>(0) / Point2f_vec.at<double>(2);
				Point2f_est.y = Point2f_vec.at<double>(1) / Point2f_vec.at<double>(2);
				int xx = Point2f_est.x + 0.5;
				int yy = Point2f_est.y + 0.5;
				circle(img, Point(xx, yy), 1, Scalar(0, 255, 0), -1);


				//GT
				int feature_x = features_2d[idx][ind].x;
				int feature_y = features_2d[idx][ind].y;
				//int feature_x = tmp_features_2d[ind].x;
				//int feature_y = tmp_features_2d[ind].y;
				circle(img, Point(feature_x, feature_y), 1, Scalar(0, 0, 255), -1);

				float GT_error = sqrt((xxx - feature_x)*(xxx - feature_x) + (yyy - feature_y)*(yyy - feature_y));
				float EST_error = sqrt((xx - feature_x)*(xx - feature_x) + (yy - feature_y)*(yy - feature_y));
				float RT_pts_error = sqrt((xx - xxx)*(xx - xxx) + (yy - yyy)*(yy - yyy));

				vec_GT_error.push_back(GT_error);
				vec_EST_error.push_back(EST_error);
				vec_RT_pts_error.push_back(RT_pts_error);

				//printf("GT_error:%f, EST_error:%f,     RT_pts_error:%f\n", GT_error, EST_error, RT_pts_error);
			}
			//printf("AVG - GT_error:%f, EST_error:%f,   RT_pts_error:%f\n", 
			//	GT_error_avg/ features_3d[idx].size(), EST_error_avg/ features_3d[idx].size(), RT_pts_error_avg/ features_3d[idx].size());

			double gt_sum = std::accumulate(vec_GT_error.begin(), vec_GT_error.end(), 0.0);
			double gt_mean = gt_sum / vec_GT_error.size();
			double gt_accum = 0.0;
			std::for_each(std::begin(vec_GT_error), std::end(vec_GT_error), [&](const double d) {
				gt_accum += (d - gt_mean) * (d - gt_mean);
			});
			double gt_stdev = sqrt(gt_accum / (vec_GT_error.size() - 1));

			double est_sum = std::accumulate(vec_EST_error.begin(), vec_EST_error.end(), 0.0);
			double est_mean = est_sum / vec_GT_error.size();
			double est_accum = 0.0;
			std::for_each(std::begin(vec_EST_error), std::end(vec_EST_error), [&](const double d) {
				est_accum += (d - est_mean) * (d - est_mean);
			});
			double est_stdev = sqrt(est_accum / (vec_EST_error.size() - 1));

			double rt_sum = std::accumulate(vec_RT_pts_error.begin(), vec_RT_pts_error.end(), 0.0);
			double rt_mean = rt_sum / vec_GT_error.size();
			double rt_accum = 0.0;
			std::for_each(std::begin(vec_RT_pts_error), std::end(vec_RT_pts_error), [&](const double d) {
				rt_accum += (d - rt_mean) * (d - rt_mean);
			});
			double rt_stdev = sqrt(rt_accum / (vec_RT_pts_error.size() - 1));

			printf("AVG - GT_error(blue):%f, EST_error(green):%f,   RT_pts_error:%f\n", gt_mean, est_mean, rt_mean);
			printf("std - GT_error:%f, EST_error:%f,   RT_pts_error:%f\n", gt_stdev, est_stdev, rt_stdev);


			imshow("asdf", img);
			waitKey(0);

		}

		printf("GT-nSmall_err_per:%f, nMid_err_per:%f, nLarge_err_per:%f\n",
			(float)nSmall_err_cnt2 / ncam, (float)nMid_err_cnt2 / ncam, (float)nLarge_err_cnt2 / ncam);

	}




	/*
	DBSCAN Clustering
	*/
#define MINIMUM_POINTS 80    // minimum number of cluster
	//#define EPSILON (0.75*0.75)  // distance for clustering, metre^2
#define EPSILON (0.08)  // distance for clustering, metre^2

	vector<DB_Point> points;
	int nLabel = 0;
	int nLevel = 2;

	vector<DB_Voxels> vec_voxels;


	H_VOXEL_DB ds(MINIMUM_POINTS, EPSILON, vec_db_pt_3d, cubeSize);

	//ds.voxelFitting_aachen(ds.m_points, ds.getTotalPointSize(), color, nLabel);  //만들어야함.
	ds.H_voxelFitting(ds.m_points, vec_voxels, ds.getTotalPointSize(), color, nLabel);  //만들어야함.
	//ds.voxelFitting_octree(ds.m_points, ds.getTotalPointSize(), color, nLabel);  //만들어야함.

	int num_points = 0;
	for (int n = 0; n < ds.m_points.size(); ++n)
	{
		for (int p = 0; p < ds.m_points[n].vec_H_voxels.size(); ++p)
		{
			num_points += ds.m_points[n].vec_H_voxels[p].vec_pt_3d_db.size();
		}
	}
	printf("\n refined num_points:%d\n", num_points);

	//To do : plane fitting 부분 H_Voxels에 맞게 수정하기.
	vector<Point3f> vec_centroid_points;
	printf("----------plane fitting preparing--------------\n");
	printf("- nLabel : %d -\n", nLabel);
	int nThresh_points = 4;

	/*
	plane fitting으로 inlier만 뽑아내서 만들기.
	*/
	//vector<H_Voxels> vec_inlierData;
	float **coefs = new float *[nLabel];
	for (int n = 0; n < nLabel; ++n)
	{
		coefs[n] = new float[4];
	}

	float sumFit_distErr = 0;

	// level 1의 fitting
	sumFit_distErr = 0;
	for (int i = 0; i < nLabel; ++i)
	{
		for (int j = 0; j < ds.m_points[i].vec_H_voxels.size(); ++j)
		{

			if (ds.m_points[i].vec_H_voxels[j].vec_pt_3d_db.size() < nThresh_points)
			{
				ds.m_points[i].vec_H_voxels[j].vec_pt_3d_db.clear();
				continue; 
			}

			printf("before ds.m_points[%d].vec_H_voxels[%d].vec_pt_3d_db size : %d\n", i, j, ds.m_points[i].vec_H_voxels[j].vec_pt_3d_db.size());


			vector<vector<float>> coef;
			vector<vector<DB_Point_3D>> tmp_inlierData;
			float meanErr = ds.PlaneFitting_Hierarchy_ver2(ds.m_points[i].vec_H_voxels[j].vec_pt_3d_db,
				ds.m_points[i].vec_H_voxels[j].vec_pt_3d_db.size(), tmp_inlierData, 0.5, color, coef);
			sumFit_distErr += meanErr;

			printf("coef size : %d\n", coef.size());

			// 벡터 복사
			//최하단 복셀에서, 평면의 개수만큼 나누어진 포인트들을... 한번 더 복셀별로 나눈다... 
			//tmp_inlierData.size()=>평면의 개수.., 그리고, 각 평면안에는 DB_Point_3D가 들어가있음
			for (int m = 0; m < tmp_inlierData.size(); ++m)
			{
				H_Voxels tmp_db_pt;
				tmp_db_pt.nLabel = tmp_inlierData[m][0].clusterID;
				tmp_db_pt.nLevel = 2;
				tmp_db_pt.vec_pt_3d_db = tmp_inlierData[m];
				ds.m_points[i].vec_H_voxels[j].vec_H_voxels.push_back(tmp_db_pt);
			}


			// 두 평면 사이의 각도로... 얼마나 비슷한지 확인해보기
			if (coef.size() != 0)
			{
				for (int m = 0; m < coef.size() - 1; ++m)
				{
					for (int n = m + 1; n < coef.size(); ++n)
					{
						double f_dotProduct = 0;
						double f_normalVectorAbs1 = 0;
						double f_normalVectorAbs2 = 0;
						double f_angle = 0;

						f_dotProduct = fabs((coef[m][0] * coef[n][0]) + (coef[m][1] * coef[n][1]) + (coef[m][2] * coef[n][2]));
						f_normalVectorAbs1 = fabs(sqrt((coef[m][0] * coef[m][0]) + (coef[m][1] * coef[m][1]) + (coef[m][2] * coef[m][2])));
						f_normalVectorAbs2 = fabs(sqrt((coef[n][0] * coef[n][0]) + (coef[n][1] * coef[n][1]) + (coef[n][2] * coef[n][2])));
						f_angle = (180 * acos(f_dotProduct / (f_normalVectorAbs1*f_normalVectorAbs2))) / M_PI;

						printf("m:%d, n:%d, f_angle:%f\n", m, n, f_angle);
					}
				}
			}


			int ds_m_points_vec_pt_3d_db_size = 0;
			for (int m = 0; m < ds.m_points[i].vec_H_voxels[j].vec_H_voxels.size(); ++m)
			{
				ds_m_points_vec_pt_3d_db_size += ds.m_points[i].vec_H_voxels[j].vec_H_voxels[m].vec_pt_3d_db.size();

				ds.m_points[i].vec_H_voxels[j].vec_H_voxels[m].coef[0] = coef[m][0];
				ds.m_points[i].vec_H_voxels[j].vec_H_voxels[m].coef[1] = coef[m][1];
				ds.m_points[i].vec_H_voxels[j].vec_H_voxels[m].coef[2] = coef[m][2];
				ds.m_points[i].vec_H_voxels[j].vec_H_voxels[m].coef[3] = coef[m][3];
				ds.m_points[i].vec_H_voxels[j].vec_H_voxels[m].nLabel = m + 1;

				for (int n = 0; n < ds.m_points[i].vec_H_voxels[j].vec_H_voxels[m].vec_pt_3d_db.size(); ++n)
				{
					ds.m_points[i].vec_H_voxels[j].vec_H_voxels[m].vec_pt_3d_db[n].clusterID = m + 1;
				}
			}
			//printf("after ds.m_points[i].vec_H_voxels[j].vec_pt_3d_db size : %d\n\n", ds.m_points[i].vec_H_voxels[j].vec_pt_3d_db.size());
			printf("after ds.m_points[i].vec_H_voxels[j].vec_pt_3d_db size : %d\n\n", ds_m_points_vec_pt_3d_db_size);
		}
	}
	printf("level1 - avgFit_distErr:%f\n", sumFit_distErr / nLabel);
	ds.H_planeFitResults2(ds.m_points, ds.m_points.size(), true, color);//Display

	num_points = 0;
	for (int n = 0; n < ds.m_points.size(); ++n)
	{
		for (int p = 0; p < ds.m_points[n].vec_H_voxels.size(); ++p)
		{
			num_points += ds.m_points[n].vec_H_voxels[p].vec_pt_3d_db.size();
		}
	}
	printf("\n  after fitting num_points:%d\n", num_points);

	//------------------------------------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------------------------------------
	//2021.06.29 : 피팅 까지 완료 했고, 이제... 딥러닝에 이용할 데이터 만드는 부분인데... 복잡하네...
	printf("\n--------- centroid points extracting... -----------\n");
	int sumEachVoxelPtCnt = 0;
	for (int i = 0; i < nLabel; ++i)
	{
		for (int j = 0; j < ds.m_points[i].vec_H_voxels.size(); ++j)
		{
			for (int k = 0; k < ds.m_points[i].vec_H_voxels[j].vec_H_voxels.size(); ++k)
			{
				vector<DB_Point_3D> &vec_label_points = ds.m_points[i].vec_H_voxels[j].vec_H_voxels[k].vec_pt_3d_db;

				Point3f centroidPt;
				if (vec_label_points.size() < nThresh_points)
				{
					centroidPt.x = -1000;
					centroidPt.y = -1000;
					centroidPt.z = -1000;
					vec_centroid_points.push_back(centroidPt);
					continue;
				}

				float sumX = 0, sumY = 0, sumZ = 0;
				float avrX = 0, avrY = 0, avrZ = 0;
				for (int k = 0; k < vec_label_points.size(); ++k)
				{
					float x = vec_label_points[k].x;
					float y = vec_label_points[k].y;
					float z = vec_label_points[k].z;

					sumX += x; sumY += y; sumZ += z;
				}
				avrX = sumX / vec_label_points.size();
				avrY = sumY / vec_label_points.size();
				avrZ = sumZ / vec_label_points.size();

				int centId = -1;
				float minDist = 99999;
				for (int k = 0; k < vec_label_points.size(); ++k)
				{
					float x = vec_label_points[k].x;
					float y = vec_label_points[k].y;
					float z = vec_label_points[k].z;

					float dist = sqrt(pow(avrX - x, 2) + pow(avrY - y, 2) + pow(avrZ - z, 2));
					if (dist < minDist)
					{
						minDist = dist;
						centId = k;
					}
				}

				centroidPt.x = vec_label_points[centId].x;
				centroidPt.y = vec_label_points[centId].y;
				centroidPt.z = vec_label_points[centId].z;

				vec_centroid_points.push_back(centroidPt);

				sumEachVoxelPtCnt += vec_label_points.size();
				printf("centId:%d, minDist:%f, #of vec_label_points:%d\n", centId, minDist, vec_label_points.size());
			}

		}

	}
	int avgEachVoxelPtCnt = sumEachVoxelPtCnt / vec_centroid_points.size();
	printf("sumEachVoxelPtCnt:%d, avgEachVoxelPtCnt:%d\n", sumEachVoxelPtCnt, avgEachVoxelPtCnt);
	printf("vec_centroid_points:%d\n", vec_centroid_points.size());



	string absPath;
	if (Voxel_Labeling_Method == 1)
		absPath = "F:/_voxelFeatureMap_Aachen_H/_single_Voxel_set/";
	else if (Voxel_Labeling_Method == 2)
		absPath = "F:/_voxelFeatureMap_Aachen_H2/_single_Voxel_set/";


	/*
	Write 3d centroid points
	*/
	string save_path = absPath + "_3d_nearest_centroid.txt";
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

	std::ofstream oStream_dbData(absPath + "db_data.txt", ios::out | ios::binary); //ios::app => 이어쓰기


	//이미지별로 불러오기
	vector<float> vec_error_t;
	vector<float> vec_error_r;
	int nSmall_err_cnt = 0;
	int nMid_err_cnt = 0;
	int nLarge_err_cnt = 0;
	for (int i = 0; i < ncam; ++i)
	{
		string img_path = "F:/Aachen-Day-Night dataset/images/images_upright/" + names[i];

		cout << names[i] << endl;

		Mat img = imread(img_path);
		Mat down_img = img.clone();

		int reSize_W = img.cols / 4;
		int reSize_H = img.rows / 4;

		resize(down_img, down_img, Size(reSize_W, reSize_H));
		Mat down_img2 = down_img.clone();

		Mat label_img = Mat::zeros(down_img.rows, down_img.cols, CV_16U);   //여기 어떻게 할지... 워낙 라벨 범위가 크다...
		Mat kp_label_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);

		Mat voxel_fillContour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		Mat ref_voxel_fillContour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);

		Mat voxel_Contour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		Mat ref_voxel_Contour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);

		int c_w = img.cols / 2;   //여기 생각좀 하자.
		int c_h = img.rows / 2;

		int donw_c_w = down_img.cols / 2;
		int donw_c_h = down_img.rows / 2;

		float fx = camera_data[i].f;
		float fy = camera_data[i].f;

		cout << "img.cols:" << img.cols << ", img.rows:" << img.rows << ", f:" << fx << ", CX:" << c_w << ", CY:" << c_h << endl;
		int size = features_3d[i].size();


		DB_Image_Aachen DB_img;

		float q[4]; float t[3];
		camera_data[i].GetQuaternionRotation(q);
		camera_data[i].GetCameraCenter(t);
		//printf("q:%f %f %f %f, t:%f %f %f\n", q[0], q[1], q[2], q[3], t[0], t[1], t[2]);

		DB_img.quat[0] = q[0]; DB_img.quat[1] = q[1]; DB_img.quat[2] = q[2]; DB_img.quat[3] = q[3];
		DB_img.camCent[0] = t[0]; DB_img.camCent[1] = t[1]; DB_img.camCent[2] = t[2];
		DB_img.r = camera_data[i].radial;
		DB_img.img_ID = i;

		string reName = names[i];
		reName.replace(reName.find("jpg"), 3, "png");
		DB_img.img_path = reName;

		float focal_len = camera_data[i].GetFocalLength();
		double scale = (float)img.rows / reSize_H;
		focal_len = focal_len / scale;
		DB_img.focal_len = focal_len;
		DB_img.Cx = donw_c_w;
		DB_img.Cy = donw_c_h;
		DB_img.r = camera_data[i].radial;


		cv::Mat A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
		cv::Mat _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);
		const double params[] = { focal_len,   // fx
								  focal_len,  // fy
								  donw_c_w,      // cx	
								  donw_c_h };    // cy

		A_matrix.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
		A_matrix.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
		A_matrix.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
		A_matrix.at<double>(1, 2) = params[3];
		A_matrix.at<double>(2, 2) = 1;
		_P_matrix.at<double>(0, 0) = camera_data[i].m[0][0];
		_P_matrix.at<double>(0, 1) = camera_data[i].m[0][1];
		_P_matrix.at<double>(0, 2) = camera_data[i].m[0][2];
		_P_matrix.at<double>(1, 0) = camera_data[i].m[1][0];
		_P_matrix.at<double>(1, 1) = camera_data[i].m[1][1];
		_P_matrix.at<double>(1, 2) = camera_data[i].m[1][2];
		_P_matrix.at<double>(2, 0) = camera_data[i].m[2][0];
		_P_matrix.at<double>(2, 1) = camera_data[i].m[2][1];
		_P_matrix.at<double>(2, 2) = camera_data[i].m[2][2];
		_P_matrix.at<double>(0, 3) = camera_data[i].t[0];
		_P_matrix.at<double>(1, 3) = camera_data[i].t[1];
		_P_matrix.at<double>(2, 3) = camera_data[i].t[2];

		cv::Mat AP_mat = cv::Mat(4, 3, CV_64FC1);
		AP_mat = A_matrix * _P_matrix;
		cv::Mat AP_mat_inv = AP_mat.inv(cv::DECOMP_SVD);

		//vector<vector<DB_Point>> vec_tmpLabel_2d_3d_pts;
		//vector<vector<DB_Point>> vec_label_2d_3d_pts;
		//vec_tmpLabel_2d_3d_pts.resize(down_img.cols*down_img.rows);
		//vec_label_2d_3d_pts.resize(down_img.cols*down_img.rows);

		Mat voxel_label_img = label_img.clone();
		Mat ref_voxel_label_img = label_img.clone();

		//2d에서 3d로 바꾸는 실험 - 카메라 자세에 의한 방향벡터(직선)의 공간상의 한점 Pl구하기
		Mat voxel_3dPt_img = Mat::zeros(down_img.rows, down_img.cols, CV_32FC3); //3dpt를 이미지로 저장하자.
		Mat voxel_3dPt_dist_img = Mat::zeros(down_img.rows, down_img.cols, CV_32FC1); //3dpt를 이미지로 저장하자.
		vector<Point2d> vec_features_2d;
		vector<Point3d> vec_features_3d;

		vector<Point2d> gt_vec_features_2d = features_2d[i];
		vector<Point3d> gt_vec_features_3d = features_3d[i];

		//To do... =>lv1_coefs 이거랑,
		/*
		clustering 3D Points와 기존 이미지별 들어오는 3D Point와 같은 곳에서,
		각 이미지별 들어오는 각 3D Point에 clustering 3D Points의 라벨을 부여.
		그리고 라벨별로 2d Point를 다시 묶음.
		-> 그 라벨 클러스터에 포인트 개수가 적으면, 특징이 그만큼 적은 것이므로, 이후에 patch로서 볼때 제외 시키기 위함.
		*/
		vector<vector<vector<vector<float>>>> lv1_coefs;
		for (int n = 0; n < nLabel; ++n)
		{
			lv1_coefs.push_back(vector<vector<vector<float>>>());
			for (int k = 0; k < ds.m_points[n].vec_H_voxels.size(); ++k)
			{
				lv1_coefs[n].push_back(vector<vector<float>>());
				for (int o = 0; o < ds.m_points[n].vec_H_voxels[k].vec_H_voxels.size(); ++o)
				{
					//vector<DB_Point_3D> &vec_label_points = ds.m_points[n].vec_H_voxels[k].vec_H_voxels[o].vec_pt_3d_db;

					lv1_coefs[n][k].push_back(vector<float>());
					lv1_coefs[n][k][o].push_back(ds.m_points[n].vec_H_voxels[k].vec_H_voxels[o].coef[0]);
					lv1_coefs[n][k][o].push_back(ds.m_points[n].vec_H_voxels[k].vec_H_voxels[o].coef[1]);
					lv1_coefs[n][k][o].push_back(ds.m_points[n].vec_H_voxels[k].vec_H_voxels[o].coef[2]);
					lv1_coefs[n][k][o].push_back(ds.m_points[n].vec_H_voxels[k].vec_H_voxels[o].coef[3]);
				}
			}
		}


		for (int n = 0; n < nLabel; ++n)
		{
			//printf("nLabel :%d - n:%d ---- vec_H_voxels.size:%d\n", nLabel, n, ds.m_points[n].vec_H_voxels.size());

			vector<vector<vector<DB_Point>>> eachLabelPt;
			int ss = ds.m_points[n].vec_H_voxels.size();
			eachLabelPt.resize(ss);   //level0의 각 복셀의 아래단계인 level1의 복셀 개수(라벨)

			vector<vector<DB_Point>> vec_tmpLabel_2d_3d_pts;
			vector<vector<DB_Point>> vec_label_2d_3d_pts;
			vec_tmpLabel_2d_3d_pts.resize(down_img.cols*down_img.rows);
			vec_label_2d_3d_pts.resize(down_img.cols*down_img.rows);

			//printf("n:%d\n", n);

			//계층적 복셀에...전부다 each로 만들어서...? 흠.... 생각해라.
			//To DO!!!! 2021.07.22 - eachLabelPt 이렇게 하는게 맞나... 다시 한번 살펴 보자.
			for (int k = 0; k < ds.m_points[n].vec_H_voxels.size(); ++k)
			{
				//여기 n 문제 있음./
				eachLabelPt[k].resize(ds.m_points[n].vec_H_voxels[k].vec_H_voxels.size());  //level1의 각 복셀에서 피팅된 평면의 개수(라벨) - level2

				//printf("ds.m_points[%d].vec_H_voxels[k].vec_H_voxels.size():%d\n",
				//	n, ds.m_points[n].vec_H_voxels[k].vec_H_voxels.size());

				for (int o = 0; o < ds.m_points[n].vec_H_voxels[k].vec_H_voxels.size(); ++o)  //평면의 개수...
				{
					vector<DB_Point_3D> &vec_label_points = ds.m_points[n].vec_H_voxels[k].vec_H_voxels[o].vec_pt_3d_db; //각 평면안에 들어오는 포이트 수.

					// level 2단계...
					if (vec_label_points.size() < nThresh_points)  // 복셀안의 포인트 수가 4개 미만이면, 진행 X
					{
						continue;
					}

					//printf("vec_label_points :%d\n", vec_label_points.size());
					for (int l = 0; l < vec_label_points.size(); ++l)
					{
						int imgs_size = vec_label_points[l].img_IDs.size();

						//printf("vec_label_points imgs_size:%d\n", imgs_size);

						for (int m = 0; m < imgs_size; ++m)
						{

							//printf("vec_label_points[l].img_IDs[%d]:%d\n", m, vec_label_points[l].img_IDs[m]);

							if (vec_label_points[l].img_IDs[m] == i)
							{
								float feature_x = vec_label_points[l].vec_2d_pt[m].x;
								float feature_y = vec_label_points[l].vec_2d_pt[m].y;

								feature_x = feature_x / ((float)img.cols / reSize_W);
								feature_y = feature_y / ((float)img.rows / reSize_H);

								DB_Point tmp_db;
								int color_lb = vec_label_points[l].clusterID - 1;
								tmp_db.clusterID = color_lb + 1;  //배경을 0 라벨로 주기위해... 
								tmp_db.x = vec_label_points[l].x;
								tmp_db.y = vec_label_points[l].y;
								tmp_db.z = vec_label_points[l].z;
								tmp_db.x_2d = feature_x;
								tmp_db.y_2d = feature_y;
								tmp_db.pt_3d_id = vec_label_points[l].pt_3d_id;
								tmp_db.pt_2d_id = vec_label_points[l].pt_2d_ids[m];

								//printf("color_lb :%d\n", color_lb);

								eachLabelPt[k][color_lb].push_back(tmp_db);


							}

						}
					}
				} // ds.m_points[n].vec_H_voxels[k].vec_H_voxels.size();

			}  //ds.m_points[n].vec_H_voxels.size();


			// '원본' voxel  data... keypoints랑 그에 의한 convexhull labeling...
			for (int j = 0; j < eachLabelPt.size(); ++j)
			{
				for (int k = 0; k < eachLabelPt[j].size(); ++k)
				{
					if (eachLabelPt[j][k].size() < nThresh_points)
						continue;

					DB_img.voxel_db_pt.push_back(eachLabelPt[j][k]);

					vector<Point> contour, resized_contour;
					Scalar cvColor(color[j][0], color[j][1], color[j][2]);
					//printf("eachLabelPt[%d].size() : %d\n", j, eachLabelPt[j].size());
					for (int l = 0; l < eachLabelPt[j][k].size(); ++l)
					{
						int feature_x = eachLabelPt[j][k][l].x_2d + 0.5;
						int feature_y = eachLabelPt[j][k][l].y_2d + 0.5;

						circle(down_img, Point(feature_x, feature_y), 1, cvColor, -1);
						resized_contour.push_back(Point(feature_x, feature_y));

						kp_label_img.data[feature_y*kp_label_img.cols + feature_x] = 255;

					}
					bool isClosed = true;

					vector<Point> hull(resized_contour.size());
					convexHull(resized_contour, hull);
					const Point *pts = (const Point*)Mat(hull).data;
					int npts = Mat(hull).rows;
					polylines(down_img, &pts, &npts, 1, isClosed, cvColor, 1);

					ushort val = eachLabelPt[j][k][0].clusterID;
					fillPoly(label_img, &pts, &npts, 1, Scalar(val));
				}
			}



		}  // for lv0 - label

		////레이블링 된 것이 아무 것도 없다면, 그 이미지는 그냥 스킵.
		//int TotalNumberOfPixels = voxel_label_img.rows*voxel_label_img.cols;
		//int zeroPixels = TotalNumberOfPixels - countNonZero(voxel_label_img);
		//printf("voxelLabel CNT zerospixels, TotalNumOfPixels! : %d, %d\n", zeroPixels, TotalNumberOfPixels);
		//if (zeroPixels == TotalNumberOfPixels)
		//{
		//	printf("None voxelLabel!\n");
		//	continue;
		//}


		//////2d에서 3d로 바꾸는 실험 - 카메라 자세에 의한 방향벡터(직선)의 공간상의 한점 Pl구하기
		//cv::Mat R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
		//cv::Mat t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix


		//bool correspondence = EstimatePoseByPnP(vec_features_2d, vec_features_3d,
		//	DB_img.focal_len, DB_img.r, A_matrix, R_matrix, t_matrix);

		//float t1 = t_matrix.at<double>(0);
		//float t2 = t_matrix.at<double>(1);
		//float t3 = t_matrix.at<double>(2);

		//GetQuaternionRotationByPnP(R_matrix, t_matrix, q);

		//float q2[4];
		//camera_data[i].GetQuaternionRotation(q2);

		//float dot_prod = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3];
		//if (dot_prod > 1) dot_prod = 1;
		//if (dot_prod < -1) dot_prod = -1;
		//float err_theta = 2 * acos(dot_prod) * 180 / M_PI;

		//float error_x = sqrt(pow(t1 - camera_data[i].t[0], 2) + pow(t2 - camera_data[i].t[1], 2) + pow(t3 - camera_data[i].t[2], 2));
		//printf("id:%d, Error XYZ (m):%f, theta:%f\n", i, error_x, err_theta);
		//vec_error_t.push_back(error_x);
		//vec_error_r.push_back(err_theta);

		//if (0.25 >= error_x && 2.0 >= err_theta)
		//{
		//	++nSmall_err_cnt;
		//}
		//if (0.5 >= error_x && 5.0 >= err_theta)
		//{
		//	++nMid_err_cnt;
		//}
		//if (5.0 >= error_x && 10.0 >= err_theta)
		//{
		//	++nLarge_err_cnt;
		//}


		//Ground truth
		for (int k = 0; k < gt_vec_features_2d.size(); ++k)
		{
			int feature_x = gt_vec_features_2d[k].x / scale + 0.5;
			int feature_y = gt_vec_features_2d[k].y / scale + 0.5;

			circle(down_img2, Point(feature_x, feature_y), 1, Scalar(0, 0, 255), -1);

			gt_vec_features_2d[k].x = feature_x;
			gt_vec_features_2d[k].y = feature_y;
		}

		////A_matrix.at<double>(0, 0) = params[0] * scale;       //      [ fx   0  cx ]
		////A_matrix.at<double>(1, 1) = params[1] * scale;       //      [  0  fy  cy ]
		////A_matrix.at<double>(0, 2) = params[2] * scale;       //      [  0   0   1 ]
		////A_matrix.at<double>(1, 2) = params[3] * scale;
		////A_matrix.at<double>(2, 2) = 1;
		//correspondence = EstimatePoseByPnP(gt_vec_features_2d, gt_vec_features_3d, DB_img.focal_len, A_matrix, R_matrix, t_matrix);  //원 사이즈가 에러가 더 작은듯?

		//t1 = t_matrix.at<double>(0);
		//t2 = t_matrix.at<double>(1);
		//t3 = t_matrix.at<double>(2);

		//float gt_q[4];
		//GetQuaternionRotationByPnP(R_matrix, t_matrix, gt_q);


		//dot_prod = gt_q[0] * q2[0] + gt_q[1] * q2[1] + gt_q[2] * q2[2] + gt_q[3] * q2[3];
		//if (dot_prod > 1) dot_prod = 1;
		//if (dot_prod < -1) dot_prod = -1;
		//float err_theta2 = 2 * acos(dot_prod) * 180 / M_PI;

		//float error_x2 = sqrt(pow(t1 - camera_data[i].t[0], 2) + pow(t2 - camera_data[i].t[1], 2) + pow(t3 - camera_data[i].t[2], 2));
		//printf("gt - id:%d, Error XYZ (m):%f, theta:%f\n", i, error_x2, err_theta2);


		//if (err_theta > 10 || error_x > 10)
		if (1)
		{
			imshow("down_img2", down_img2);
			imshow("kp_label_img", kp_label_img);
			imshow("down_img", down_img);
			cv::Mat pic16bit;
			label_img.convertTo(pic16bit, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
			cv::namedWindow("label_img");
			cv::imshow("label_img", pic16bit);

			cv::Mat pic16bit2;
			voxel_label_img.convertTo(pic16bit2, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
			cv::namedWindow("voxel_label_img");
			cv::imshow("voxel_label_img", pic16bit2);

			//cv::Mat pic16bit3;
			//ref_voxel_label_img.convertTo(pic16bit3, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
			//cv::namedWindow("ref_voxel_label_img");
			//cv::imshow("ref_voxel_label_img", pic16bit3);

			imshow("voxel_Contour_img", voxel_Contour_img);
			//imshow("ref_voxel_Contour_img", ref_voxel_Contour_img);
			cv::waitKey(0);
		}
		// ========================================  - level 1 - =================================================		


		/*
			그 이미지에 해당하는 feature points 확인하기...
		*/
		//Mat gt_kp_label_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		//for (int j = 0; j < size; ++j)
		//{
		//	Scalar cvColor(255, 0, 0);
		//	float feature_x = features_2d[i][j].x + c_w + 0.5;
		//	float feature_y = features_2d[i][j].y + c_h + 0.5;

		//	feature_x = feature_x / (1920.0 / down_img.cols);
		//	feature_y = feature_y / (1080.0 / down_img.rows);

		//	//circle(kp_label_img, Point(feature_x, feature_y), 1, cvColor, -1);
		//	gt_kp_label_img.data[(int)feature_y*gt_kp_label_img.cols + (int)feature_x] = 255;
		//}
		//imshow("GT kp_label_img", gt_kp_label_img);
		//waitKey(0);



		//vector<string> line_str = split(names[i], '.');
		//vector<string> img_path_split = split(line_str[0], '_');
		//string save_path_rgb = absPath + img_path_split[0] + ".png";
		//string save_path = absPath + img_path_split[0] + "_label" + ".png";
		//string save_path2 = absPath + img_path_split[0] + "_kp" + ".png";
		//string save_path3 = absPath + img_path_split[0] + "_3dpts" + ".yml";
		//string save_path4 = absPath + img_path_split[0] + "_contour" + ".png";
		//string save_path5 = absPath + img_path_split[0] + "_fillcontour" + ".png";
		//cout << save_path << endl;

		//if (Voxel_Labeling_Method == 1)
		//{
		//	imwrite(save_path_rgb, down_img2);
		//	imwrite(save_path, voxel_label_img);
		//	imwrite(save_path2, kp_label_img);
		//	imwrite(save_path4, voxel_Contour_img);
		//	imwrite(save_path5, voxel_fillContour_img);
		//}
		//else if (Voxel_Labeling_Method == 2)
		//{
		//	imwrite(save_path_rgb, down_img2);
		//	imwrite(save_path, ref_voxel_label_img);
		//	imwrite(save_path2, kp_label_img);
		//	imwrite(save_path4, ref_voxel_Contour_img);
		//	imwrite(save_path5, ref_voxel_fillContour_img);
		//}
		//else
		//{
		//	imwrite(save_path_rgb, down_img2);
		//	imwrite(save_path, label_img);
		//	imwrite(save_path2, kp_label_img);
		//}


		//FileStorage fs_w(save_path3, FileStorage::WRITE);
		//fs_w << "voxel_3dPt_img" << voxel_3dPt_img;  //choose any key here, just be consistant with the one below
		//fs_w.release();

		////text파일로 저장하는 방법 - pt_2d_id는 저장 안함. -> 이미지 사이즈가 클 수록... 저장할 포인트가 많아지기 때문에.. 문제당...
		//oStream_dbData << DB_img.img_ID << " " << DB_img.img_path << " " << DB_img.quat[0] << " " << DB_img.quat[1]
		//	<< " " << DB_img.quat[2] << " " << DB_img.quat[3] << " " << DB_img.camCent[0] << " " << DB_img.camCent[1]
		//	<< " " << DB_img.camCent[2] << " " << DB_img.focal_len << " ";

		//int ref_pt_size_sum = 0;
		//for (int lb_id = 0; lb_id < DB_img.voxel_db_pt.size(); ++lb_id)
		//{
		//	int lb_pt_sz = DB_img.voxel_db_pt[lb_id].size();
		//	ref_pt_size_sum += lb_pt_sz;
		//}
		//oStream_dbData << ref_pt_size_sum << endl;

		//for (int lb_id = 0; lb_id < DB_img.voxel_db_pt.size(); ++lb_id)
		//{
		//	int lb_pt_sz = DB_img.voxel_db_pt[lb_id].size();
		//	for (int pt_id = 0; pt_id < lb_pt_sz; ++pt_id)
		//	{
		//		oStream_dbData << DB_img.voxel_db_pt[lb_id][pt_id].x_2d << " " << DB_img.voxel_db_pt[lb_id][pt_id].y_2d
		//			<< " " << DB_img.voxel_db_pt[lb_id][pt_id].x << " " << DB_img.voxel_db_pt[lb_id][pt_id].y << " " << DB_img.voxel_db_pt[lb_id][pt_id].z
		//			<< " " << DB_img.voxel_db_pt[lb_id][pt_id].clusterID << " " << DB_img.voxel_db_pt[lb_id][pt_id].pt_3d_id << endl;
		//	}

		//}

	}

	oStream_dbData.close();

	double sum = std::accumulate(vec_error_t.begin(), vec_error_t.end(), 0.0);
	double mean = sum / vec_error_t.size();

	double sum_q = std::accumulate(vec_error_r.begin(), vec_error_r.end(), 0.0);
	double mean_q = sum_q / vec_error_r.size();

	printf("vec_error_t mean:%f, mean_q:%f \n", mean, mean_q);


	printf("nSmall_err_per:%f, nMid_err_per:%f, nLarge_err_per:%f\n",
		(float)nSmall_err_cnt / vec_error_t.size(), (float)nMid_err_cnt / vec_error_t.size(), (float)nLarge_err_cnt / vec_error_t.size());


	for (int x = 0; x < NUM_COLOR; x++)
	{
		delete[] color[x];
	}
	delete[] color;

	for (int x = 0; x < nLabel; x++)
	{
		delete[] coefs[x];
	}
	delete[] coefs;

	printf("finish!\n");

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



bool LoadModelFile(const char* name, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc, int cubeSize)
{
	if (name == NULL)return false;
	ifstream in(name);

	std::cout << "Loading cameras/points: " << name << "\n";
	if (!in.is_open()) return false;

	if (strstr(name, ".nvm"))return LoadNVM(in, camera_data, point_data, measurements, ptidx, camidx, names, ptc, cubeSize);
}


void plane_normVec(DB_Plane& plane)
{
	double x1 = plane.x[3] - plane.x[0];
	double y1 = plane.y[3] - plane.y[0];
	double z1 = plane.z[3] - plane.z[0];

	double x2 = plane.x[1] - plane.x[0];
	double y2 = plane.y[1] - plane.y[0];
	double z2 = plane.z[1] - plane.z[0];

	//surface noramal vector
	double x3 = y1 * z2 - z1 * y2;
	double y3 = z1 * x2 - x1 * z2;
	double z3 = x1 * y2 - y1 * x2;
	//plane.surfaceNomalX = x3;
	//plane.surfaceNomalY = y3;
	//plane.surfaceNomalZ = z3;

	double l = sqrt(x3*x3 + y3 * y3 + z3 * z3);
	plane.surfaceNomalX = x3 / l;
	plane.surfaceNomalY = y3 / l;
	plane.surfaceNomalZ = z3 / l;

	//nomalized vector
	plane.equation.a = x3 / l;
	plane.equation.b = y3 / l;
	plane.equation.c = z3 / l;
	plane.equation.d = -(plane.equation.a*plane.x[0] + plane.equation.b*plane.y[0] + plane.equation.c*plane.z[0]);
}