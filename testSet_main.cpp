
#include <string.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include <io.h>

using namespace std;

#include "DataInterface.h"
#include "experiment.h"
#include "SY_Labeling.h"


/*
- KingsCollege_cube50 : class=87개 (배경까지 포함해서...)
- KingsCollege_cube100 : class=141개 (배경까지 포함해서...)
- KingsCollege_cube120 : class=195개 (배경까지 포함해서...)

- OldHospital_cube20 : class=183개 (배경까지 포함해서...)
- OldHospital_cube30 : class=311개 (배경까지 포함해서...)

- ShopFacade_cube10 : class=44개 (배경까지 포함해서...)
- ShopFacade_cube20 : class=96개 (배경까지 포함해서...)
- ShopFacade_cube30 : class=158개 (배경까지 포함해서...)
- ShopFacade_cube40 : class=242개 (배경까지 포함해서...)
- ShopFacade_cube50 : class=305개 (배경까지 포함해서...)

- StMarysChurch_cube30 : class=212개 (배경까지 포함해서...)
- StMarysChurch_cube50 : class=447개 (배경까지 포함해서...)
- StMarysChurch_cube70 : class=705개 (배경까지 포함해서...)
*/
int main(void)
{
	/*
KingsCollege, OldHospital, ShopFacade, StMarysChurch, Street
*/

//////////////////////////////////////////////////////
	//const char *input_filename = "[Street] reconstruction.nvm";            //first argument must be filename
	string data_name = "ShopFacade_cube40_0.01_fusion/";   //ShopFacade_90per, Street_cube50_0.01, ShopFacade_cube40_0.01_fusion
	int nImageCnt = 182;   //kings = 343, OldHospital=182, shopfacade=103, StMarysChurch=530, Street=2923

	string str = "ShopFacade_cube40/";
	string txt_path = "F:/_voxelFeatureMap/" + str + "_3d_nearest_centroid.txt";
	ifstream input(txt_path);  //"_3d_centroid.txt"	
	ifstream input2("ShopFacade_pose.txt");   //_pose,  KingsCollege_pose, OldHospital_pose, ShopFacade_pose, StMarysChurch_pose, ShopFacade

	vector<Point3d> vec_cent_pt3d;
	int lb_size = 0;
	if (input.is_open())
	{
		float val = 0;
		
		input >> lb_size;
		for (int i = 0; i < lb_size; ++i)
		{
			Point3d pt;
			input >> val;
			pt.x = val;
			input >> val;
			pt.y = val;
			input >> val;
			pt.z = val;
			vec_cent_pt3d.push_back(pt);
		}
		//for (int i = 0; i < lb_size; ++i)
		//{
		//	cout << vec_cent_pt3d[i].x << " " << vec_cent_pt3d[i].y << " " << vec_cent_pt3d[i].z << endl;
		//}
	}
	input.close();

	//3d points 읽는 시간 0.3ms

	float error_T_sum = 0, error_R_sum = 0;


	vector<vector<float>> vec_pose;
	vec_pose.resize(nImageCnt);
	if (input2.is_open())
	{
		float val = 0;
		for (int i = 0; i < nImageCnt; ++i)
		{
			float pose[7];
			for (int j = 0; j < 7; ++j)
			{
				input2 >> val;
				pose[j] = val;
				vec_pose[i].push_back(val);
			}
		}
		for (int i = 0; i < nImageCnt; ++i)
		{
			for (int j = 0; j < 7; ++j)
			{
				cout << vec_pose[i][j] << " ";
			}
			cout << endl;
		}
	}
	input2.close();


	vector<string> vec_ori_imgPath;
	vector<string> vec_imgPath;
	string path = "F:/_voxelFeatureMap/" + str  + "test_set2.txt";
	ifstream input3(path);
	if (input3.is_open())
	{
		float val = 0;
		for (int i = 0; i < nImageCnt; ++i)
		{
			string str;
			input3 >> str;

			int nExt = str.rfind("png");
			int nName = str.rfind("/") -4;
			//string strModExt("png");
			//string strReName;
			//strReName = "F:/_voxelFeatureMap/ShopFacade/";
			//strReName += str.substr(nName, nExt - nName);
			//strReName += strModExt;
			//vec_imgPath.push_back(strReName);
			vec_ori_imgPath.push_back(str);

			for (int j = 0; j < 7; ++j)
			{
				input3 >> val;
			}
		}
	}
	input3.close();

	double d_processAvgTime = 0;
	vector<float> vec_error_t;
	vector<float> vec_error_r;
	int points4lower = 0;
	for (int id = 0; id < nImageCnt; ++id)
	{
		//string strReName = vec_imgPath[id];
		string strReName = "F:/_segmentResults/" + data_name + to_string(id) + ".png";
		Mat img = imread(strReName, cv::IMREAD_ANYDEPTH); //uint16 읽기 위함
		Mat ori_img = imread(vec_ori_imgPath[id]); //uint16 읽기 위함

		resize(ori_img, ori_img, Size(img.cols, img.rows));


		double time1 = cv::getTickCount();


		//-----------------------------------------------------------------------------------------------------------------------
		//같은 라벨인데, 붙어있지 않고 떨어진 적은수의 라벨들을 제거해주자...-> 중심 구할 때 에러의 요소임.
		ushort *ptr_img_data = img.ptr<ushort>(0);
		vector<bool> vec_is_lb(false);
		vec_is_lb.resize(lb_size + 1);  //2D는 배경라벨(0번)도 있으므로, 배경 라벨까지 더해줘야함..
		for (int y = 0; y < img.rows; ++y)
		{
			for (int x = 0; x < img.cols; ++x)
			{
				int lb = ptr_img_data[y*img.cols + x];
				if (lb != 0)
				{
					vec_is_lb[lb] = true;
				}
			}
		}
		vector<int> curr_lb;  //현재 이미지에 있는 라벨만 따로 저장.
		for (int i = 0; i < vec_is_lb.size(); ++i)
		{
			if (vec_is_lb[i] == true)
				curr_lb.push_back(i);
		}

		Mat refinedLBImg = Mat::zeros(Size(img.cols, img.rows), CV_16U);
		//ushort *refinedLbData = refinedLBImg.ptr<ushort>(0);
		for (int i = 0; i < curr_lb.size(); ++i)
		{
			Mat labeledTmpImg = Mat::zeros(Size(img.cols, img.rows), CV_16U);
			ushort *lbData = labeledTmpImg.ptr<ushort>(0);
			for (int y = 0; y < img.rows; ++y)
			{
				for (int x = 0; x < img.cols; ++x)
				{
					int lb = ptr_img_data[y*img.cols + x];
					if (curr_lb[i] == lb)
						lbData[y*img.cols + x] = lb;
				}
			}

			Mat labeledDstImg = Mat::zeros(Size(img.cols, img.rows), CV_16U);
			ushort *dstData = labeledDstImg.ptr<ushort>(0);
			SY_Labeling sy_lb(img.cols, img.rows);
			int nNumofLabels = sy_lb.labeling(lbData, dstData);

			int maxLabelId = -1;
			int maxCnt = 0;
			for (int n = 0; n < sy_lb.m_vecPixelCnt.size(); ++n)
			{
				int cnt = sy_lb.m_vecPixelCnt[n];
				if (maxCnt < cnt)
				{
					maxCnt = cnt;
					maxLabelId = n + 1;
				}
			}


			for (int y = 0; y < img.rows; ++y)
			{
				for (int x = 0; x < img.cols; ++x)
				{
					int lb = dstData[y*img.cols + x];
					if (maxLabelId != lb)  //label이 1부터 시작함...
						lbData[y*img.cols + x] = 0;
				}
			}
			
			bitwise_or(refinedLBImg, labeledTmpImg, refinedLBImg);
		}
		//-----------------------------------------------------------------------------------------------------------------------


		int c_w = img.cols / 2;
		int c_h = img.rows / 2;

		int w = img.cols;
		int h = img.rows;

		ushort *ptr_data = refinedLBImg.ptr<ushort>(0);
		//ushort *ptr_data = img.ptr<ushort>(0);

		vector<vector<Point2d>> pt_2d;
		pt_2d.resize(lb_size+1);            //2D는 배경라벨(0번)도 있으므로, 배경 라벨까지 더해줘야함..
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				int val = ptr_data[y*w + x];
				if (val != 0)
				{
					pt_2d[val].push_back(Point2d(x, y));
					//printf("x:%d, y:%d, val:%d\n", x, y, val);
				}

			}
		}

		vector<Point2d> vec_cent_pt2d;
		vec_cent_pt2d.resize(lb_size + 1);   //2D는 배경라벨(0번)도 있으므로, 배경 라벨까지 더해줘야함..
		for (int i = 0; i < pt_2d.size(); ++i)
		{
			int sz = pt_2d[i].size();
			if (sz < 10) continue;           //이거 좀 체크하자.

			float sum_x = 0, sum_y = 0;
			for (int j = 0; j < sz; ++j)
			{
				sum_x += pt_2d[i][j].x;
				sum_y += pt_2d[i][j].y;
			}
			float avg_x = sum_x / sz;
			float avg_y = sum_y / sz;
			vec_cent_pt2d[i] = Point2d(avg_x - c_w + 0.5, avg_y - c_h + 0.5);
			//printf("avg_x:%f, avg_y:%f\n", avg_x, avg_y);
		}

		vector<Point2d> list_points2d;
		vector<Point3d> list_points3d;
		for (int i = 0; i < (lb_size+1)-1; ++i)
		{
			if (vec_cent_pt2d[i+1].x == 0 && vec_cent_pt2d[i+1].y == 0) continue;

			list_points2d.push_back(vec_cent_pt2d[i+1]);
			list_points3d.push_back(vec_cent_pt3d[i]);
		}

		//StMarysChurch cube 30 에 대한 실험...
		//if (id == 10)
		//{
		//	list_points2d.erase(list_points2d.begin() + 10);
		//	list_points3d.erase(list_points3d.begin() + 10);
		//	list_points2d.erase(list_points2d.begin() + 9);
		//	list_points3d.erase(list_points3d.begin() + 9);
		//	list_points2d.erase(list_points2d.begin() + 6);
		//	list_points3d.erase(list_points3d.begin() + 6);

		//	list_points2d.erase(list_points2d.begin() + 3);
		//	list_points3d.erase(list_points3d.begin() + 3);
		//	list_points2d.erase(list_points2d.begin() + 2);
		//	list_points3d.erase(list_points3d.begin() + 2);
		//}

		////StMarysChurch cube 50 에 대한 실험...
		//if (id == 7)
		//{
		//	list_points2d.erase(list_points2d.begin() + 21);
		//	list_points3d.erase(list_points3d.begin() + 21);

		//	list_points2d.erase(list_points2d.begin() + 19);
		//	list_points3d.erase(list_points3d.begin() + 19);

		//	list_points2d.erase(list_points2d.begin() + 18);
		//	list_points3d.erase(list_points3d.begin() + 18);

		//	list_points2d.erase(list_points2d.begin() + 15);
		//	list_points3d.erase(list_points3d.begin() + 15);

		//	list_points2d.erase(list_points2d.begin() + 9);
		//	list_points3d.erase(list_points3d.begin() + 9);

		//	list_points2d.erase(list_points2d.begin() + 4);
		//	list_points3d.erase(list_points3d.begin() + 4);
		//}



		if (list_points2d.size() < 4)
		{
			++points4lower;
			printf("%d\n", id);
			continue;
		}




		//TEST -> Ground Truth의 R|t  => q2[4]와 t[3] 구하기 error차이 구하려고.
		/*
		GT R|t  => q2[4]와 t[3]
		*/
		float _c1 = vec_pose[id][0];
		float _c2 = vec_pose[id][1];
		float _c3 = vec_pose[id][2];

		float q2[4];
		q2[0] = vec_pose[id][3];
		q2[1] = vec_pose[id][4];
		q2[2] = vec_pose[id][5];
		q2[3] = vec_pose[id][6];

		float m[3][3];
		double qq = sqrt(q2[0] * q2[0] + q2[1] * q2[1] + q2[2] * q2[2] + q2[3] * q2[3]);
		double qw, qx, qy, qz;
		if (qq > 0)
		{
			qw = q2[0] / qq;
			qx = q2[1] / qq;
			qy = q2[2] / qq;
			qz = q2[3] / qq;
		}
		else
		{
			qw = 1;
			qx = qy = qz = 0;
		}
		m[0][0] = float_t(qw*qw + qx * qx - qz * qz - qy * qy);
		m[0][1] = float_t(2 * qx*qy - 2 * qz*qw);
		m[0][2] = float_t(2 * qy*qw + 2 * qz*qx);
		m[1][0] = float_t(2 * qx*qy + 2 * qw*qz);
		m[1][1] = float_t(qy*qy + qw * qw - qz * qz - qx * qx);
		m[1][2] = float_t(2 * qz*qy - 2 * qx*qw);
		m[2][0] = float_t(2 * qx*qz - 2 * qy*qw);
		m[2][1] = float_t(2 * qy*qz + 2 * qw*qx);
		m[2][2] = float_t(qz*qz + qw * qw - qy * qy - qx * qx);

		//t = - R * C
		float t[3];
		for (int j = 0; j < 3; ++j)
			t[j] = -float_t(m[j][0] * _c1 + m[j][1] * _c2 + m[j][2] * _c3);
		float _t1 = t[0];
		float _t2 = t[1];
		float _t3 = t[2];


		//-----------------------------------------------------------------------------------------------------------------------
		//TEST -> Ground Truth의 R|t를 이용한 projection 좌표를 할당하여, solve pnp와 비교해 보기...
		//list_points2d.clear();
		//for (int n = 0; n < list_points3d.size(); ++n)
		//{
		//	/*
		//	GT R|t를 이용한 projection 좌표
		//	*/
		//	float x = m[0][0] * list_points3d[n].x
		//		+ m[0][1] * list_points3d[n].y
		//		+ m[0][2] * list_points3d[n].z;

		//	float y = m[1][0] * list_points3d[n].x
		//		+ m[1][1] * list_points3d[n].y
		//		+ m[1][2] * list_points3d[n].z;

		//	float z = m[2][0] * list_points3d[n].x
		//		+ m[2][1] * list_points3d[n].y
		//		+ m[2][2] * list_points3d[n].z;

		//	x = x + _t1;
		//	y = y + _t2;
		//	z = z + _t3;

		//	double scale = 1080.0 / 256.0;
		//	double focal_length = 1672.0 / scale;
		//	//double focal_length = 1672.0;

		//	x = focal_length * x;
		//	y = focal_length * y;

		//	float pnp_feature_x = x / z;
		//	float pnp_feature_y = y / z;

		//	list_points2d.push_back(Point2d(pnp_feature_x, pnp_feature_y));
		//}
		//-----------------------------------------------------------------------------------------------------------------------


		/** The calibration matrix */
		cv::Mat A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
		cv::Mat R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
		cv::Mat t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix

		/** The computed projection matrix */
		cv::Mat _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);

		double scale = 1080.0 / 256.0;
		double focal_length = 1672.0 / scale;  //oldhospital=1660, shopfacade=1672
		//double focal_length = 1672.0;
		bool correspondence = EstimatePoseByPnP(list_points2d, list_points3d, focal_length, A_matrix, R_matrix, t_matrix);


		double time2 = cv::getTickCount();
		double tmatch = 1000.0*(time2 - time1) / cv::getTickFrequency();
		//d_processAvgTime += tmatch;
		//cout << " pose estimation processing Time (ms): " << tmatch << endl;


		float t1 = t_matrix.at<double>(0);
		float t2 = t_matrix.at<double>(1);
		float t3 = t_matrix.at<double>(2);

		float q[4] = { 0 };
		GetQuaternionRotationByPnP(R_matrix, t_matrix, q);

		//float theta = acos(0) * 180/ M_PI;
		float dot_prod = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3];
		if (dot_prod > 1) dot_prod = 1;
		if (dot_prod < -1) dot_prod = -1;
		float theta2 = acos(dot_prod) * 180 / M_PI;

		float err1 = t1 - _t1; float err2 = t2 - _t2; float err3 = t3 - _t3;
		float error_x = sqrt(pow(t1 - _t1, 2) + pow(t2 - _t2, 2) + pow(t3 - _t3, 2));

		error_T_sum += error_x;
		error_R_sum += theta2;

		vec_error_t.push_back(error_x);
		vec_error_r.push_back(theta2);
		printf("id:%d, Error XYZ (m):%f, theta:%f\n", id, error_x, theta2);


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

		srand(0);
		float color[800][3] = { 0 };
		for (int i = 0; i < 800; i++)
		{
			color[i][0] = (rand() % 255);
			color[i][1] = (rand() % 255);
			color[i][2] = (rand() % 255);
		}
		
		for (int i = 0; i < pt_2d.size(); ++i)
		{
			vector<Point> contour;
			for (int k = 0; k < pt_2d[i].size(); ++k)
			{
				int feature_x = pt_2d[i][k].x;
				int feature_y = pt_2d[i][k].y;

				int r = color[i][0];
				int g = color[i][1];
				int b = color[i][2];
				//circle(ori_img, Point(feature_x, feature_y), 4, Scalar(b, g, r), -1);
				contour.push_back(Point(feature_x, feature_y));
			}

			if (contour.size() == 0) continue;

			Scalar cvColor(color[i][0], color[i][1], color[i][2]);
			bool isClosed = true;

			vector<Point> hull(contour.size());
			convexHull(contour, hull);
			const Point *pts = (const Point*)Mat(hull).data;
			int npts = Mat(hull).rows;
			polylines(ori_img, &pts, &npts, 1, isClosed, cvColor, 1);
			//polylines(label_img, &pts, &npts, 1, isClosed, Scalar(255), 1);
			fillPoly(ori_img, &pts, &npts, 1, cvColor);
		}


		for (int n = 0; n < list_points3d.size(); ++n)
		{
			float Point3f_x = list_points3d[n].x;
			float Point3f_y = list_points3d[n].y;
			float Point3f_z = list_points3d[n].z;

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

			Point2d avgPt = Point2d(list_points2d[n].x + c_w + 0.5, list_points2d[n].y + c_h + 0.5); 
			//printf("%f. %f\n", avgPt.x, avgPt.y);

			circle(ori_img, Point(pnp_feature_x, pnp_feature_y), 2, Scalar(255, 0, 0), -1);
			circle(ori_img, Point(avgPt.x, avgPt.y), 2, Scalar(0, 0, 255), -1);
		}

		//imshow("test", ori_img);
		//waitKey(0);

	} // nImageCnt

	d_processAvgTime = d_processAvgTime / nImageCnt;;

	//cout << " pose estimation processing Time (ms): " << d_processAvgTime << endl;




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
	printf("points4lower : %d \n", points4lower);

	

	return 0;
}
