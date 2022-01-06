
#include <opencv2\opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2\cudafeatures2d.hpp>

#define NOMINMAX  //windows.h 헤더 파일에서 min과 max를 전처리기 매크로로 정의를 해서 발생하는 문제를 해결하기 위함.
#define M_PI       3.14159265358979323846   // pi

//#if _DEBUG
//	#pragma comment(lib, "opencv_world343d.lib")
//#endif
//	#pragma comment(lib, "opencv_world343.lib")
#if _DEBUG
#pragma comment(lib, "opencv_world345d.lib")
#endif
#pragma comment(lib, "opencv_world345.lib")
using namespace cv;

#include "SY_Labeling.h"
#include "DataInterface.h"

#include <string.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include <io.h>

using namespace std;

vector<string> split(string str, char delimiter) {
	vector<string> internal;
	stringstream ss(str);
	string temp;

	while (getline(ss, temp, delimiter)) {
		internal.push_back(temp);
	}

	return internal;
}

bool EstimatePoseByPnP(vector<Point2d> features_2d, vector<Point3d> features_3d, double focal_length,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix);

void GetQuaternionRotationByPnP(cv::Mat R_matrix, cv::Mat t_matrix, float q[4]);


int main(void)
{

	//////////////////////////////////////////////////////
	//int nImageCnt = 103;   //kings = 343, OldHospital=182, shopfacade=103, StMarysChurch=530, Street=2923

	//vector<string> data_names{ "ShopFacade_cube20/", "OldHospital_cube15/", "KingsCollege_cube50/",  "StMarysChurch_cube50/" };
	//vector<string> data_names{ "ShopFacade_cube40/", "OldHospital_cube30/", "KingsCollege_cube100/",  "StMarysChurch_cube100/" };
	vector<string> data_names{ "ShopFacade_cube80/" };
	for (int id = 0; id < data_names.size(); ++id)   // id=1 부터 시작하는건, shop 40 완성 했으니깐.
	{
		string data_name = data_names[id];


		const char *input_filename = "";

		if (id == 0)
		{
			input_filename = "[ShopFacade] reconstruction.nvm";            //first argument must be filename
		}
		else if (id == 1)
		{
			input_filename = "[OldHospital] reconstruction.nvm";
		}
		else if (id == 2)
		{
			input_filename = "[KingsCollege] reconstruction.nvm";
		}
		else if (id == 3)
		{
			input_filename = "[StMarysChurch] reconstruction.nvm";
		}


		//string txt_path = "F:/_voxelFeatureMap11_single/" + data_name + "_3d_nearest_centroid.txt";
		//ifstream input(txt_path);  //"_3d_centroid.txt"	
		//ifstream input2("ShopFacade_pose.txt");   //_pose,  KingsCollege_pose, OldHospital_pose, ShopFacade_pose, StMarysChurch_pose, ShopFacade
		//vector<Point3d> vec_cent_pt3d;
		//int lb_size = 0;
		//if (input.is_open())
		//{
		//	float val = 0;

		//	input >> lb_size;
		//	for (int i = 0; i < lb_size; ++i)
		//	{
		//		Point3d pt;
		//		input >> val;
		//		pt.x = val;
		//		input >> val;
		//		pt.y = val;
		//		input >> val;
		//		pt.z = val;
		//		vec_cent_pt3d.push_back(pt);
		//	}
		//}
		//input.close();

		if (input_filename == NULL)return false;
		ifstream in(input_filename);

		std::cout << "Loading cameras/points: " << input_filename << "\n";
		if (!in.is_open()) return false;

		float error_T_sum = 0, error_R_sum = 0;
		vector<float> vec_error_t;
		vector<float> vec_error_r;
		double d_processAvgTime = 0;
		int nImageCnt = 0;
		if (strstr(input_filename, ".nvm"))
		{
			int rotation_parameter_num = 4;
			bool format_r9t = false;
			string token;
			vector<string> names;
			vector<double> vec_focalLen;

			vector<CameraT> camera_data;

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
			nImageCnt = ncam;
			names.resize(ncam);
			vec_focalLen.resize(ncam); // allocate the focal length
			camera_data.resize(ncam); // allocate the camera data

			float radial_distortion = 0;
			for (int i = 0; i < ncam; ++i)
			{
				double f, q[9], c[3], d[2];
				in >> token >> f;
				for (int j = 0; j < rotation_parameter_num; ++j) in >> q[j];
				in >> c[0] >> c[1] >> c[2] >> d[0] >> d[1];

				names[i] = token;
				radial_distortion = d[0];
				vec_focalLen[i] = f;

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
			}

			//////////////////////////////////////
			in >> npoint;   if (npoint <= 0) return false;

			//read image projections and 3D points.
			for (int i = 0; i < npoint; ++i)
			{

				float pt[3]; int cc[3], npj;
				in >> pt[0] >> pt[1] >> pt[2]
					>> cc[0] >> cc[1] >> cc[2] >> npj;

				for (int j = 0; j < npj; ++j)
				{
					int cidx, fidx; float imx, imy;
					in >> cidx >> fidx >> imx >> imy;

					nproj++;
				}
			}

			string absPath;
			absPath = "F:/_voxelFeatureMap11_single/";

			for (int i = 0; i < ncam; ++i)
			{
				string img_path = "F:/RPNet_test/data/absolute_cambridge/" + data_name + names[i];
				cout << "camId: " << i << ",  " << names[i] << endl;

				vector<string> line_str = split(names[i], '.');
				vector<string> img_path_split = split(line_str[0], '_');
				string lb_path = absPath + data_name + img_path_split[0] + "_label_" + ".png";
				string contour_path = absPath + data_name + img_path_split[0] + "_contour" + ".png";
				string fillcontour_path = absPath + data_name + img_path_split[0] + "_fillcontour" + ".png";
				string pts_3d_map_path = absPath + data_name + img_path_split[0] + "_3dpts" + ".yml";

				string weightMap_path = absPath + data_name + img_path_split[0] + "_weightmap_" + ".png";

				Mat lb_img = imread(lb_path, IMREAD_ANYDEPTH);
				Mat contour_img = imread(contour_path, 0);
				//imshow("lb_img", lb_img);
				//waitKey(0);

				Mat label_weight_img = Mat::zeros(lb_img.rows, lb_img.cols, CV_8U);

				ushort *lb_data_ptr = lb_img.ptr<ushort>(0);
				uchar *lb_weight_data_ptr = label_weight_img.ptr<uchar>(0);

				for (int yy = 0; yy < lb_img.rows; ++yy)
				{
					for (int xx = 0; xx < lb_img.cols; ++xx)
					{
						ushort val = lb_data_ptr[yy*lb_img.cols + xx];
						if (val == 0)
							continue;
						lb_weight_data_ptr[yy*lb_img.cols + xx] = 255;
					}
				}

				Mat dist;
				distanceTransform(label_weight_img, dist, DIST_L2, 3);
				// Normalize the distance image for range = {0.0, 1.0}
				// so we can visualize and threshold it
				normalize(dist, dist, 0, 1.0, NORM_MINMAX);

				//float *dist_data_ptr = dist.ptr<float>(0);
				//for (int yy = 0; yy < dist.rows; ++yy)
				//{
				//	for (int xx = 0; xx < dist.cols; ++xx)
				//	{
				//		//double val = dist_data_ptr[yy*dist.cols + xx];
				//		float val = dist.at<float>(yy, xx);
				//		float ret = exp(2 * (1 - val));
				//		if (val <= 0.00001)
				//			ret = 2;
				//		else if (val > 0.00001)
				//			int dfu = 0;
				//		
				//		lb_weight_data_ptr[yy*dist.cols + xx] = int(ret);
				//	}
				//}

				float *dist_data_ptr = dist.ptr<float>(0);
				for (int yy = 0; yy < dist.rows; ++yy)
				{
					for (int xx = 0; xx < dist.cols; ++xx)
					{
						float val = dist_data_ptr[yy*dist.cols + xx];
						//float val = dist.at<float>(yy, xx);
						float ret = (1 - val) * 10 + 1;
						int ret2 = 1;
						if (ret < 10.2)
						{
							ret = 1;
							ret2 = 1;
						}
						else
						{
							ret2 = 10;
						}
						if (val <= 0.00001)
						{
							ret = 5;
							ret2 = 5;
						}
							
												
						dist_data_ptr[yy*dist.cols + xx] = ret;
						lb_weight_data_ptr[yy*dist.cols + xx] = (int)ret;
					}
				}


				for (int yy = 0; yy < contour_img.rows; ++yy)
				{
					for (int xx = 0; xx < contour_img.cols; ++xx)
					{
						uchar val = contour_img.data[yy*contour_img.cols + xx];
						if (val == 0) continue;

						lb_weight_data_ptr[yy*contour_img.cols + xx] = 10;
					}
				}


				//imshow("Distance Transform Image", dist);
				//imshow("label_weight_img", label_weight_img);
				//waitKey(0);
				imwrite(weightMap_path, label_weight_img);

				int debug = 0;




				//pnp TEst - 속도 확인하기.

				/** The calibration matrix */
				cv::Mat A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
				cv::Mat R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
				cv::Mat t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix

				/** The computed projection matrix */
				cv::Mat _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);

				float reSize_H = 256, reSize_W = 456;
				double scale = 1080.0 / 256.0;
				//double focal_length = 1672.0 / scale;  //oldhospital=1660, shopfacade=1672

				//double scale = lb_img.rows / reSize_H;
				double focal_len = vec_focalLen[i] / scale;  //oldhospital=1660, shopfacade=1672

				int donw_c_w = lb_img.cols / 2;
				int donw_c_h = lb_img.rows / 2;

				vector<cv::Point2d> list_points2d;
				vector<cv::Point3d> list_points3d;

				FileStorage fs(pts_3d_map_path, FileStorage::READ);
				Mat pts_3d_map;
				fs["voxel_3dPt_img"] >> pts_3d_map;
				fs.release();

				for (int y = 0; y < pts_3d_map.rows; ++y)
				{
					for (int x = 0; x < pts_3d_map.cols; ++x)
					{
						float x_3d = pts_3d_map.at<Vec3f>(y, x)[0];
						float y_3d = pts_3d_map.at<Vec3f>(y, x)[1];
						float z_3d = pts_3d_map.at<Vec3f>(y, x)[2];
						if (x_3d == 0 && y_3d == 0 && z_3d == 0)
							continue;

						list_points2d.push_back(Point2d(x- donw_c_w, y - donw_c_h));
						list_points3d.push_back(Point3d(x_3d, y_3d, z_3d));
					}
				}

				if (i == 318)
				{
					imshow("contour_img", contour_img);
					waitKey(0);
					int dbufdf = 0;
				}

				double time1 = cv::getTickCount();

				bool correspondence = EstimatePoseByPnP(list_points2d, list_points3d, focal_len, A_matrix, R_matrix, t_matrix);


				double time2 = cv::getTickCount();
				double tmatch = 1000.0*(time2 - time1) / cv::getTickFrequency();
				d_processAvgTime += tmatch;
				cout << " pose estimation processing Time (ms): " << tmatch << endl;

				float t1 = t_matrix.at<double>(0);
				float t2 = t_matrix.at<double>(1);
				float t3 = t_matrix.at<double>(2);

				float q[4] = { 0 };
				GetQuaternionRotationByPnP(R_matrix, t_matrix, q);


				float q2[4]; float _t1[3];
				camera_data[i].GetQuaternionRotation(q2);
				//camera_data[i].GetCameraCenter(_t1);

				//float theta = acos(0) * 180/ M_PI;
				float dot_prod = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3];
				if (dot_prod > 1) dot_prod = 1;
				if (dot_prod < -1) dot_prod = -1;
				float theta2 = acos(dot_prod) * 180 / M_PI;

				float err1 = t1 - camera_data[i].t[0]; float err2 = t2 - camera_data[i].t[1]; float err3 = t3 - camera_data[i].t[2];
				float error_x = sqrt(pow(t1 - camera_data[i].t[0], 2) + pow(t2 - camera_data[i].t[1], 2) + pow(t3 - camera_data[i].t[2], 2));

				error_T_sum += error_x;
				error_R_sum += theta2;

				vec_error_t.push_back(error_x);
				vec_error_r.push_back(theta2);
				printf("id:%d, Error XYZ (m):%f, theta:%f\n", id, error_x, theta2);

			}

			std::vector <float> ::iterator b = vec_error_t.begin();
			std::vector<float>::iterator e = vec_error_t.end();

			std::vector<float>::iterator med = b;
			std::advance(med, vec_error_t.size() / 2);

			// This makes the 2nd position hold the median.
			std::nth_element(b, med, e);

			cout << "med t:" << med[0];

			b = vec_error_r.begin();
			e = vec_error_r.end();
			med = b;
			std::advance(med, vec_error_r.size() / 2);
			std::nth_element(b, med, e);

			cout << ", med r:" << med[0] << endl;

			printf("error_T_avr : %f, error_R_avr : %f\n", error_T_sum / nImageCnt, error_R_sum / nImageCnt);

			d_processAvgTime = d_processAvgTime / nImageCnt;
			cout << " AVG pose estimation processing Time (ms): " << d_processAvgTime << endl;
		}
	}
}

	



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