#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#define _USE_MATH_DEFINES //<cmath>에서 M_PI 사용하려고...
#include <cmath> 
using namespace std;


#include <pcl/point_types.h> //`pcl::PointXYZ`을 포함한 포인트 type 구조체 정의 
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/impl/io.hpp>
#include <pcl/filters/extract_indices.h>

#if _DEBUG
#pragma comment(lib, "pcl_common_debug.lib")
#endif
#pragma comment(lib, "pcl_common_release.lib")
//#pragma comment(lib, "pcl_features_release.lib")
#pragma comment(lib, "pcl_io_release.lib")
#pragma comment(lib, "pcl_io_ply_release.lib")
//#pragma comment(lib, "pcl_kdtree_release.lib")
//#pragma comment(lib, "pcl_ml_release.lib")
//#pragma comment(lib, "pcl_octree_release.lib")
//#pragma comment(lib, "pcl_outofcore_release.lib")
//#pragma comment(lib, "pcl_people_release.lib")
//#pragma comment(lib, "pcl_recognition_release.lib")
//#pragma comment(lib, "pcl_registration_release.lib")
#pragma comment(lib, "pcl_sample_consensus_release.lib")
//#pragma comment(lib, "pcl_search_release.lib")
#pragma comment(lib, "pcl_segmentation_release.lib")
//#pragma comment(lib, "pcl_stereo_release.lib")
//#pragma comment(lib, "pcl_surface_release.lib")
//#pragma comment(lib, "pcl_tracking_release.lib")
#pragma comment(lib, "pcl_visualization_release.lib")


#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3



typedef struct Point_2D_
{
	float x;
	float y;
public:
	Point_2D_(float _x = 0, float _y = 0)
		:x(_x), y(_y) {}
	void setXY(float _x, float _y) { x = _x, y = _y; }
	Point_2D_ getXY() { return Point_2D_(x, y); }
}Point_2D;


typedef struct Point_3D_
{
	float x, y, z;
public:
	Point_3D_(float _x = 0, float _y = 0, float _z = 0)
		:x(_x), y(_y), z(_z) {}
	void setXY(float _x, float _y, float _z = 0) { x = _x, y = _y, z = _z; }
	Point_3D_ getXY() { return Point_3D_(x, y, z); }
}Point_3D;

typedef struct Point_3D_DB
{
	//float x_2d, y_2d;
	float x, y, z;  // X, Y, Z position
	int clusterID;  // clustered ID
	int pt_3d_id;
	vector<Point_2D> vec_2d_pt;
	vector<int> pt_2d_ids;
	vector<int> img_IDs;

	//Point_3D_DB ***cube_points;

public:
	Point_3D_DB(float _x = 0, float _y = 0, float _z = 0, int _label = -1)
		:x(_x), y(_y), z(_z), clusterID(_label) {}

	void setXYZ(float _x, float _y, float _z) { x = _x, y = _y, z = _z; }
	Point_3D_DB getXYZ() { return Point_3D_DB(x, y, z); }
}DB_Point_3D;


typedef struct DB_POINT
{
	float x_2d, y_2d;
	float x, y, z;  // X, Y, Z position
	int clusterID;  // clustered ID
	int pt_2d_id;
	int pt_3d_id;
	//int img_ID;
	//vector<int> pt_3d_ids;
	//string img_path;

public:
	DB_POINT(float _x = 0, float _y = 0, float _z = 0, int _label = -1)
		:x(_x), y(_y), z(_z), clusterID(_label) {}
	DB_POINT getXYZ() { return DB_POINT(x, y, z); }

}DB_Point;


typedef struct _Image
{
	int img_ID;
	string img_path;
	float quat[4];
	float camCent[3];
	float focal_len;

	vector<vector<DB_Point>> voxel_db_pt;    //experiment2_ver2

}DB_Image;

typedef struct _Image_Aachen
{
	int img_ID;
	string img_path;
	float quat[4];
	float camCent[3];
	float focal_len;
	float Cx;
	float Cy;
	float r;  //distortion

	vector<vector<DB_Point>> voxel_db_pt;    //experiment2_ver2

}DB_Image_Aachen;

typedef struct _Image2
{
	int img_ID;
	string img_path;
	float quat[4];
	float camCent[3];
	float focal_lenX;
	float focal_lenY;
	float Cx;
	float Cy;
	float r;  //distortion

	vector<vector<DB_Point>> voxel_db_pt;    //experiment2_ver2

}DB_Image2;


class DBSCAN {
public:
	DBSCAN(unsigned int minPts, float eps, vector<DB_Point_3D> points, int cubeSize) {
		m_minPoints = minPts;
		m_epsilon = eps;
		m_points = points;
		m_pointSize = points.size();
		m_cubeSize = cubeSize;
	}
	~DBSCAN() {}

	int run();
	vector<int> calculateCluster(DB_Point_3D point);
	int expandCluster(DB_Point_3D point, int clusterID);
	inline double calculateDistance(DB_Point_3D pointCore, DB_Point_3D pointTarget);

	int getTotalPointSize() { return m_pointSize; }
	int getMinimumClusterSize() { return m_minPoints; }
	int getEpsilonSize() { return m_epsilon; }
	void printResults(vector<Point_3D_DB>& points, int num_points, bool bAllpoints, float color[][3]);
	void PlaneFitting(vector<DB_Point>& points, int num_points, vector<DB_Point>& inlier_points, float color[][3]);
	void voxelFitting(vector<DB_Point>& points, int num_points, float color[][3], int &nLabel);
	void voxelFitting2(vector<DB_Point>& points, int num_points, float color[][3], int &nLabel);

	//절대 스케일에서 크기를 정하고.... 시작하자. (loop를 돌면서 줄일필요가 없음.)
	void voxelFitting3(vector<DB_Point>& points, int num_points, float color[][3], int &nLabel, int cubeSize);

	vector<DB_Point_3D> m_points;
	int m_numCluster;
private:
	
	double point2planedistnace(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients) {
		double f1 = fabs(coefficients->values[0] * pt.x + coefficients->values[1] * pt.y + coefficients->values[2] * pt.z + coefficients->values[3]);
		double f2 = sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) + pow(coefficients->values[2], 2));
		return f1 / f2;
	}

	unsigned int m_pointSize;
	unsigned int m_minPoints;
	float m_epsilon;
	int m_cubeSize;
};
