#pragma once

#include <math.h>
#include <time.h>
#include <iomanip>
#include <unordered_map>

#define _USE_MATH_DEFINES //<cmath>에서 M_PI 사용하려고...
#include <cmath> 
using namespace std;
#include "DataInterface.h"
#include "H_voxel_DB.h"
//#include "voxel_DB.h"

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
bool LoadModelFile(string& image_filePath, string& camera_filePath, string& points3D_filePath, int cubeSize);



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
bool EstimatePoseByPnP(vector<Point2d> features_2d, vector<Point3d> features_3d, double focal_length,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix);

void plane_normVec(DB_Plane& plane);

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

//  1 : nvm 3d 좌표에서 사영된 2d points로 label image 만든 것으로, 2d-3d corresponding 관계 구한것.
//  2 : fitting plane 안에 속하는 3d points에서 min,max XYZ 구한 꼭지점을 사영한 2d points로 label image 만든 것으로, 2d-3d corresponding 관계 구한것.
#define Voxel_Labeling_Method 1


bool LoadModelFile(string& image_filePath, string& camera_filePath, string& points3D_filePath, int cubeSize)
{

	vector<CameraT> camera_data;
	vector<string> names;
	vector<int> vec_img_ids;

	//read image projections and 3D points.
	vector<H_Voxels> vec_db_pt_3d;
	vec_db_pt_3d.resize(1);
	vec_db_pt_3d[0].nLabel = -1;
	vec_db_pt_3d[0].nLevel = 0;
	vec_db_pt_3d[0].vec_pt_3d_db.clear();

	int ncam = 0;

	map<int, Image_DB> map_img_dbs;
	vector< Camera_DB > vec_camera_dbs;
	map<int, Point3D_DB > map_point3d_hloc_dbs;
	//unordered_map<int, DB_Point_3D> map_point3d_dbs;
	map<int, DB_Point_3D> map_point3d_dbs;

	std::ifstream readFile(image_filePath);
	if (readFile.is_open())    //파일이 열렸는지 확인
	{
		string str;
		while (!readFile.eof())    //파일 끝까지 읽었는지 확인
		{
			getline(readFile, str);
			vector<string> result = split(str, ' ');

			if (result.size() <= 0 || result[0] == "#")
				continue;

			Image_DB img_db_tmp;

			int image_id = stoi(result[0]);
			vector<float> qvec;
			vector<float> tvec;
			for (int i = 0; i < 4; ++i)
				qvec.push_back(stof(result[i + 1]));
			for (int i = 0; i < 3; ++i)
				tvec.push_back(stof(result[i + 5]));
			int camera_id = stoi(result[8]);
			string image_name = result[9];

			string sub_str;
			getline(readFile, sub_str);
			vector<string> result2 = split(sub_str, ' ');

			vector<Point_2D> xys;
			vector<int> point3D_ids;
			for (int i = 0; i < result2.size() / 3; i++)
			{
				Point_2D pt;
				pt.x = stof(result2[i * 3]);
				pt.y = stof(result2[i * 3 + 1]);
				xys.push_back(pt);

				point3D_ids.push_back(stoi(result2[i * 3 + 2]));
			}

			img_db_tmp.image_id = image_id;
			img_db_tmp.qvec = qvec;
			img_db_tmp.tvec = tvec;
			img_db_tmp.image_name = image_name;
			img_db_tmp.camera_id = camera_id;
			img_db_tmp.xys = xys;
			img_db_tmp.point3D_ids = point3D_ids;
			map_img_dbs[image_id] = img_db_tmp;


			CameraT camera_tmp;
			float q[4];
			q[0] = qvec[0]; q[1] = qvec[1]; q[2] = qvec[2]; q[3] = qvec[3];
			camera_tmp.SetQuaternionRotation(q);
			camera_tmp.t[0] = tvec[0]; camera_tmp.t[1] = tvec[1]; camera_tmp.t[2] = tvec[2];
			camera_data.push_back(camera_tmp);

			names.push_back(image_name);
			vec_img_ids.push_back(image_id);

			if (image_id == 2427 || image_id == 1889 || image_id == 3626 || image_id == 53 || image_id == 3090 || image_id == 44 || image_id == 43)
				printf("image_id:%d\n", image_id);

			int debug = 0;
		}
	}
	readFile.close();

	readFile.open(camera_filePath);
	if (readFile.is_open())    //파일이 열렸는지 확인
	{
		string str;
		while (!readFile.eof())    //파일 끝까지 읽었는지 확인
		{
			getline(readFile, str);
			vector<string> result = split(str, ' ');

			if (result.size() <= 0 || result[0] == "#")
				continue;

			int camera_id = stoi(result[0]);
			string model = result[1];

			int width = stoi(result[2]);
			int height = stoi(result[3]);

			vector<float> vec_params;
			for (int i = 0; i < 4; i++)
			{
				vec_params.push_back(stof(result[i + 4]));
			}

			Camera_DB camera_db;

			camera_db.camera_id = camera_id;
			camera_db.model_name = model;
			camera_db.width = width;
			camera_db.height = height;
			camera_db.focal_length = vec_params[0];
			camera_db.cx = vec_params[1];
			camera_db.cy = vec_params[2];
			camera_db.distortion = vec_params[3];
			vec_camera_dbs.push_back(camera_db);

			camera_data[ncam].SetFocalLength(vec_params[0]);
			camera_data[ncam].radial = vec_params[3];
			++ncam;
			int debug = 0;
		}
	}
	readFile.close();

	readFile.open(points3D_filePath);
	if (readFile.is_open())    //파일이 열렸는지 확인
	{
		string str;
		while (!readFile.eof())    //파일 끝까지 읽었는지 확인
		{
			getline(readFile, str);
			vector<string> result = split(str, ' ');

			if (result.size() <= 0 || result[0] == "#")
				continue;

			int point3D_id = stoi(result[0]);
			vector<float> vec_xyz;
			for (int i = 0; i < 3; i++)
			{
				vec_xyz.push_back(stof(result[i + 1]));
			}

			vector<float> vec_rgb;
			for (int i = 0; i < 3; i++)
			{
				vec_rgb.push_back(stof(result[i + 4]));
			}

			float error = stof(result[7]);

			vector<int> vec_image_ids;
			vector<int> vec_point2D_ids;
			int size = (result.size() - 8) / 2;
			for (int i = 0; i < size; i++)
			{
				vec_image_ids.push_back(stoi(result[i * 2 + 8]));
				vec_point2D_ids.push_back(stoi(result[i * 2 + 1 + 8]));
			}

			Point3D_DB point3d_db;
			point3d_db.point3D_id = point3D_id;
			point3d_db.xyz = Point_3D(vec_xyz[0], vec_xyz[1], vec_xyz[2]);
			point3d_db.rgb = Point_3D(vec_rgb[0], vec_rgb[1], vec_rgb[2]);
			point3d_db.image_ids = vec_image_ids;
			point3d_db.point2d_ids = vec_point2D_ids;
			map_point3d_hloc_dbs[point3D_id] = point3d_db;


			DB_Point_3D db_pt_3d;
			db_pt_3d.setXYZ(vec_xyz[0], vec_xyz[1], vec_xyz[2]);
			db_pt_3d.pt_3d_id = point3D_id;
			db_pt_3d.pt_2d_ids = vec_point2D_ids;
			db_pt_3d.img_IDs = vec_image_ids;
			//vec_db_pt_3d[0].vec_pt_3d_db.push_back(db_pt_3d);

			map_point3d_dbs[point3D_id] = db_pt_3d;
			map_point3d_dbs[point3D_id].vec_2d_pt.resize(map_point3d_dbs[point3D_id].img_IDs.size());
			int debug = 0;
		}
	}
	readFile.close();

	printf("refine(adding) 2dpts for 3dpt ... \n");


	map<int, vector<Point2d>> map_features_2d;
	map<int, vector<Point3d>> map_features_3d;
	for (pair<int, Image_DB> atom : map_img_dbs)
	{
		//if (atom.second.image_id != 1889) continue;

		vector<Point2d> vec_features_2d;
		vector<Point3d> vec_features_3d;
		for (int j = 0; j < atom.second.point3D_ids.size(); ++j)
		{
			int pointd3D_id = atom.second.point3D_ids[j];
			if (pointd3D_id == -1) continue;

			Point_2D pt = atom.second.xys[j];
			for (int k = 0; k < map_point3d_dbs[pointd3D_id].img_IDs.size(); ++k)
			{
				if (map_point3d_dbs[pointd3D_id].img_IDs[k] == atom.second.image_id)
					map_point3d_dbs[pointd3D_id].vec_2d_pt[k] = pt;
			}
			//map_point3d_dbs[pointd3D_id].vec_2d_pt.push_back(pt);
			//printf("%f, %f, %f, %f\n", pt.x, pt.y, map_point3d_dbs[pointd3D_id].vec_2d_pt[0].x, map_point3d_dbs[pointd3D_id].vec_2d_pt[0].y);

			float x_2d = atom.second.xys[j].x;
			float y_2d = atom.second.xys[j].y;
			//printf("%f, %f\n", x_2d, y_2d);

			if (atom.second.image_id == 256)  //1889, 3154, 3125, 3122, 3118, 2286, 1167, 3099, 1746, 1173, 3095, 276, 2121
			{
				printf("%f, %f, %d\n", pt.x, pt.y, pointd3D_id);
			}

			float x_3d = map_point3d_hloc_dbs[pointd3D_id].xyz.x;
			float y_3d = map_point3d_hloc_dbs[pointd3D_id].xyz.y;
			float z_3d = map_point3d_hloc_dbs[pointd3D_id].xyz.z;

			vec_features_2d.push_back(Point2d(x_2d, y_2d));
			vec_features_3d.push_back(Point3d(x_3d, y_3d, z_3d));

			map_features_2d[atom.second.image_id].push_back(Point2d(x_2d, y_2d));
			map_features_3d[atom.second.image_id].push_back(Point3d(x_3d, y_3d, z_3d));
		}

	}

	//2021/.08.28~  To do 여기서의 2d point할당 된것을 이용할 때 맞지가 않는다...
	for (pair<int, DB_Point_3D> atom : map_point3d_dbs)
	{
		vec_db_pt_3d[0].vec_pt_3d_db.push_back(atom.second);
		int sz = atom.second.img_IDs.size();
		for (int i = 0; i < sz; ++i)
		{
			if (atom.second.img_IDs[i] != 256) continue;
			printf("x:%f, y:%f, point3d_id:%d\n", atom.second.vec_2d_pt[i].x, atom.second.vec_2d_pt[i].y, atom.second.pt_3d_id);
		}
	}


#define NUM_COLOR 5000
	srand(0);
	int(*color)[3] = new int[NUM_COLOR][3];
	for (int i = 0; i < NUM_COLOR; i++)
	{
		color[i][0] = (rand() % 255);
		color[i][1] = (rand() % 255);
		color[i][2] = (rand() % 255);
	}

	map<int, vector<Point2d>> map_features_2d_2;
	map<int, vector<Point3d>> map_features_3d_2;

	for (int i = 0; i < vec_db_pt_3d[0].vec_pt_3d_db.size(); ++i)
	{
		DB_Point_3D db_pt_3ds = vec_db_pt_3d[0].vec_pt_3d_db[i];
		for (int j = 0; j < db_pt_3ds.img_IDs.size(); ++j)
		{
			int img_id = db_pt_3ds.img_IDs[j];
			//printf("img_id:%d\n", img_id);     //이미지 사이즈는 4328개인데, img id가 그걸 넘는 경우가 있음... 하...

			float x_2d = db_pt_3ds.vec_2d_pt[j].x;
			float y_2d = db_pt_3ds.vec_2d_pt[j].y;

			float x_3d = db_pt_3ds.x;
			float y_3d = db_pt_3ds.y;
			float z_3d = db_pt_3ds.z;

			map_features_2d_2[img_id].push_back(Point2d(x_2d, y_2d));
			map_features_3d_2[img_id].push_back(Point3d(x_3d, y_3d, z_3d));
		}
	}


	printf("finish!\n");

	/*
	DBSCAN Clustering
	*/
#define MINIMUM_POINTS 80    // minimum number of cluster
	//#define EPSILON (0.75*0.75)  // distance for clustering, metre^2
#define EPSILON (0.08)  // distance for clustering, metre^2

	//vector<DB_POINT> points;
	int nLabel = 0;

	vector<DB_Voxels> vec_voxels;
	H_VOXEL_DB ds(MINIMUM_POINTS, EPSILON, vec_db_pt_3d, cubeSize);
	ds.H_voxelFitting_allCnt_2(ds.m_points, vec_voxels, ds.getTotalPointSize(), color, nLabel);  //만들어야함.
	//VOXEL_DB ds(MINIMUM_POINTS, EPSILON, vec_db_pt_3d, cubeSize);
	//ds.voxelFitting5(ds.m_points, vec_voxels, ds.getTotalPointSize(), color, nLabel);  //만들어야함.

	int num_points = ds.m_points.size();
	printf("\n num_points:%d\n", num_points);

	vector<DB_Point_3D> tmp_points;
	for (int i = 0; i < num_points; ++i)
	{
		for (int p = 0; p < ds.m_points[i].vec_pt_3d_db.size(); ++p)
		{
			if (ds.m_points[i].vec_pt_3d_db[p].clusterID != -1)
			{
				tmp_points.push_back(ds.m_points[i].vec_pt_3d_db[p]);
			}
		}

	}
	printf("tmp_points size:%d\n", tmp_points.size());
	printf("nLabel:%d\n", nLabel);

	/*
	label별로 묶기
	*/
	vector<Point3f> vec_centroid_points;
	vector<vector<DB_Point_3D>> vec_label_points;
	vec_label_points.resize(nLabel);

	printf("-------------------------------------------\n");
	for (int i = 0; i < tmp_points.size(); ++i)
	{
		int label = tmp_points[i].clusterID - 1;
		vec_label_points[label].push_back(tmp_points[i]);
	}

	int nThresh_points = 4;

	
	printf("--------- centroid points extracting... -----------\n");
	int sumEachVoxelPtCnt = 0;
	for (int i = 0; i < nLabel; ++i)
	{
		Point3f centroidPt;
		if (vec_label_points[i].size() < nThresh_points)
		{
			//vec_label_points[i].clear();

			centroidPt.x = -1000;
			centroidPt.y = -1000;
			centroidPt.z = -1000;
			vec_centroid_points.push_back(centroidPt);
			continue;
		}

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

		//		Point3f centroidPt;
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

		vec_centroid_points.push_back(centroidPt);

		sumEachVoxelPtCnt += vec_label_points[i].size();
		//printf("centId:%d, minDist:%f, vec_label_points[i]:%d\n", centId, minDist, vec_label_points[i].size());
	}
	int avgEachVoxelPtCnt = sumEachVoxelPtCnt / nLabel;
	printf("sumEachVoxelPtCnt:%d, avgEachVoxelPtCnt:%d\n", sumEachVoxelPtCnt, avgEachVoxelPtCnt);
	printf("vec_centroid_points:%d\n", vec_centroid_points.size());

	string absPath;
	if (Voxel_Labeling_Method == 1)
		absPath = "F:/_voxelFeatureMap_Aachen_H/_single_Voxel_set/";
	else if (Voxel_Labeling_Method == 2)
		absPath = "F:/_voxelFeatureMap_Aachen_H2/_single_Voxel_set/";


	//graph-txt로 저장하기...
	string vox_save_path = absPath + "voxel_graph_2.txt";
	ofstream vox_output(vox_save_path);
	int all_vox_size = vec_centroid_points.size();
	vector<int> *vox_graph = new vector<int>[all_vox_size + 1];
	for (int i = 0; i < nLabel; ++i)
	{

		int node_size = ds.m_points[i].graph_vox.node.size();
		for (int k = 0; k < node_size; ++k)
		{
			int ref_voxID = ds.m_points[i].vec_pt_3d_db[0].clusterID;
			int node_voxID = ds.m_points[i].graph_vox.node[k];

			vox_graph[ref_voxID].push_back(node_voxID);
		}

		//for (int j = 0; j < ds.m_points[i].vec_H_voxels.size(); ++j)
		//{
		//	int node_size = ds.m_points[i].vec_H_voxels[j].graph_vox.node.size();
		//	for (int k = 0; k < node_size; ++k)
		//	{
		//		int ref_voxID = ds.m_points[i].vec_H_voxels[j].vec_pt_3d_db[0].clusterID;
		//		int node_voxID = ds.m_points[i].vec_H_voxels[j].graph_vox.node[k];

		//		vox_graph[ref_voxID].push_back(node_voxID);
		//	}
		//}
	}

	//첫번째는 복셀이 없는 부분(배경)이므로, 제외시키고... 저장
	vox_output << all_vox_size << "\n";
	for (int i = 1; i < all_vox_size + 1; ++i)
	{
		//printf("vox_graph[%d]-size:%d \n", i, vox_graph[i].size());
		for (int j = 0; j < vox_graph[i].size(); ++j)
		{
			//printf("%d ", vox_graph[i][j]);
			vox_output << vox_graph[i][j] << " ";
		}
		//printf("\n");
		vox_output << "\n";
	}

	vox_output.close();
	delete[] vox_graph;


	/*
	Write 3d centroid points
	*/
	//string save_path = "F:/_voxelFeatureMap_Aachen/_single_Voxel_set/_3d_nearest_centroid.txt";
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


	//printf("finish!\n");
	//exit(0);


	std::ofstream oStream_dbData(absPath + "db_data.txt", ios::out | ios::binary); //ios::app => 이어쓰기


	//이미지별로 불러오기
	vector<float> vec_error_t;
	vector<float> vec_error_r;
	//id 894
	for (int i = 0; i < ncam; ++i)
	{
		//string img_path = "F:/Aachen-Day-Night dataset/images/images_upright/" + names[i];
		//cout << names[i] << endl;

		int img_id = vec_img_ids[i];
		string img_path = "F:/Aachen-Day-Night dataset/images/images_upright/" + names[i];
		cout << names[i] << ", imageID: " << vec_img_ids[i] << endl;

		Mat img = imread(img_path);
		Mat down_img = img.clone();

		int reSize_W = img.cols;
		int reSize_H = img.rows;

		int c_w = img.cols / 2;   //여기 생각좀 하자.
		int c_h = img.rows / 2;

		float fx = camera_data[i].f;
		float fy = camera_data[i].f;

		cout << "img.cols:" << img.cols << ", img.rows:" << img.rows << ", f:" << fx << ", CX:" << c_w << ", CY:" << c_h << endl;
		//int size = features_3d[i].size();

		/*
		clustering 3D Points와 기존 이미지별 들어오는 3D Point와 같은 곳에서,
		각 이미지별 들어오는 각 3D Point에 clustering 3D Points의 라벨을 부여.
		그리고 라벨별로 2d Point를 다시 묶음.
		-> 그 라벨 클러스터에 포인트 개수가 적으면, 특징이 그만큼 적은 것이므로, 이후에 patch로서 볼때 제외 시키기 위함.
		*/
		vector<vector<DB_POINT>> eachLabelPt;
		int ss = vec_label_points.size();
		eachLabelPt.resize(ss);
		for (int k = 0; k < vec_label_points.size(); ++k)
		{
			if (vec_label_points[k].size() < nThresh_points)
			{
				continue;
			}
			for (int l = 0; l < vec_label_points[k].size(); ++l)
			{
				int imgs_size = vec_label_points[k][l].img_IDs.size();
				for (int m = 0; m < imgs_size; ++m)
				{
					//if (vec_label_points[k][l].img_IDs[m] == i)
					if (vec_label_points[k][l].img_IDs[m] == vec_img_ids[i])
					{
						float feature_x = vec_label_points[k][l].vec_2d_pt[m].x;
						float feature_y = vec_label_points[k][l].vec_2d_pt[m].y;

						feature_x = feature_x / ((float)img.cols / reSize_W);
						feature_y = feature_y / ((float)img.rows / reSize_H);


						//printf("%f, %f\n ", feature_x, feature_y);

						DB_POINT tmp_db;
						int color_lb = vec_label_points[k][l].clusterID - 1;
						tmp_db.clusterID = color_lb + 1;  //배경을 0 라벨로 주기위해... 
						tmp_db.x = vec_label_points[k][l].x;
						tmp_db.y = vec_label_points[k][l].y;
						tmp_db.z = vec_label_points[k][l].z;
						tmp_db.x_2d = feature_x;
						tmp_db.y_2d = feature_y;
						tmp_db.pt_3d_id = vec_label_points[k][l].pt_3d_id;
						tmp_db.pt_2d_id = vec_label_points[k][l].pt_2d_ids[m];

						eachLabelPt[color_lb].push_back(tmp_db);
					}

				}

			}

		}

		DB_Image_Aachen DB_img;

		float q[4]; float t[3];
		camera_data[i].GetQuaternionRotation(q);
		camera_data[i].GetCameraCenter(t);

		DB_img.quat[0] = q[0]; DB_img.quat[1] = q[1]; DB_img.quat[2] = q[2]; DB_img.quat[3] = q[3];
		DB_img.camCent[0] = t[0]; DB_img.camCent[1] = t[1]; DB_img.camCent[2] = t[2];
		DB_img.img_ID = img_id;

		string reName = names[i];
		reName.replace(reName.find("jpg"), 3, "png");
		DB_img.img_path = reName;

		float focal_len = camera_data[i].GetFocalLength();
		double scale = (float)img.rows / reSize_H;
		focal_len = focal_len / scale;
		DB_img.focal_len = focal_len;
		DB_img.Cx = c_w;
		DB_img.Cy = c_h;
		DB_img.r = camera_data[i].radial;


		//원본 voxel  data... keypoints랑 그에 의한 convexhull labeling...
		for (int j = 0; j < eachLabelPt.size(); ++j)
		{
			if (eachLabelPt[j].size() < 4)
				continue;

			DB_img.voxel_db_pt.push_back(eachLabelPt[j]);

		}

		//if (vec_features_2d.size() < 4)
		//{
		//	printf("\n i:%d, image_id:%d\n\n", i, img_id);
		//	continue;
		//}

		//text파일로 저장하는 방법 - pt_2d_id는 저장 안함. -> 이미지 사이즈가 클 수록... 저장할 포인트가 많아지기 때문에.. 문제당...
		oStream_dbData << DB_img.img_ID << " " << DB_img.img_path << " " << DB_img.quat[0] << " " << DB_img.quat[1]
			<< " " << DB_img.quat[2] << " " << DB_img.quat[3] << " " << DB_img.camCent[0] << " " << DB_img.camCent[1]
			<< " " << DB_img.camCent[2] << " " << DB_img.focal_len << " ";

		int ref_pt_size_sum = 0;
		for (int lb_id = 0; lb_id < DB_img.voxel_db_pt.size(); ++lb_id)
		{
			int lb_pt_sz = DB_img.voxel_db_pt[lb_id].size();
			ref_pt_size_sum += lb_pt_sz;
		}
		oStream_dbData << ref_pt_size_sum << endl;

		for (int lb_id = 0; lb_id < DB_img.voxel_db_pt.size(); ++lb_id)
		{
			int lb_pt_sz = DB_img.voxel_db_pt[lb_id].size();
			for (int pt_id = 0; pt_id < lb_pt_sz; ++pt_id)
			{
				oStream_dbData << DB_img.voxel_db_pt[lb_id][pt_id].x_2d << " " << DB_img.voxel_db_pt[lb_id][pt_id].y_2d
					<< " " << DB_img.voxel_db_pt[lb_id][pt_id].x << " " << DB_img.voxel_db_pt[lb_id][pt_id].y << " " << DB_img.voxel_db_pt[lb_id][pt_id].z
					<< " " << DB_img.voxel_db_pt[lb_id][pt_id].clusterID << " " << DB_img.voxel_db_pt[lb_id][pt_id].pt_3d_id
					<< " " << 0
					<< endl;
			}

		}

	}
	oStream_dbData.close();

	double sum = std::accumulate(vec_error_t.begin(), vec_error_t.end(), 0.0);
	double mean = sum / vec_error_t.size();

	double sum_q = std::accumulate(vec_error_r.begin(), vec_error_r.end(), 0.0);
	double mean_q = sum_q / vec_error_r.size();

	printf("vec_error_t mean:%f, mean_q:%f \n", mean, mean_q);

	for (int x = 0; x < NUM_COLOR; x++)
	{
		delete[] color[x];
	}
	delete[] color;


	printf("DBSCAN finish!\n");

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