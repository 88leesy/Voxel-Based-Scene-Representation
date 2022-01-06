#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>



#define _USE_MATH_DEFINES //<cmath>에서 M_PI 사용하려고...
#include <cmath> 
using namespace std;

//#include <pcl/console/parse.h>
//#include <pcl/filters/extract_indices.h>
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

#include <pcl/octree/octree_search.h>  
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#if _DEBUG
#pragma comment(lib, "pcl_common_debug.lib")
#endif
#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_io_release.lib")
#pragma comment(lib, "pcl_io_ply_release.lib")
#pragma comment(lib, "pcl_sample_consensus_release.lib")
#pragma comment(lib, "pcl_segmentation_release.lib")
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
	void setXY(float _x, float _y, float _z = 0) { x = _x, y = _y, z=_z; }
	Point_3D_ getXY() { return Point_3D_(x, y, z); }
}Point_3D;

typedef struct DB_Plane
{
	float x[4], y[4], z[4];  // X, Y, Z position
	float surfaceNomalX, surfaceNomalY, surfaceNomalZ;

	struct _equation {
		float a, b, c, d;
		public:
			_equation(float _a, float _b, float _c, float _d) :a(_a), b(_b), c(_c), d(_d) {}
	}equation;

public:
	DB_Plane(float a = 0, float b = 0, float c = 0, float d = 0, double _surfaceNomalX = 0, double _surfaceNomalY = 0, double _surfaceNomalZ = 0)
		:equation(a, b, c, d), surfaceNomalX(_surfaceNomalX), surfaceNomalY(_surfaceNomalY), surfaceNomalZ(_surfaceNomalZ) {}

}DB_Plane;

typedef struct DB_Voxels
{
	DB_Plane planes[6];
	int clusterID;
}DB_Voxels;


typedef struct Point_3D_DB
{
	//float x_2d, y_2d;
	float x, y, z;  // X, Y, Z position
	int clusterID;  // clustered ID
	int pt_3d_id;
	vector<Point_2D> vec_2d_pt;
	vector<int> pt_2d_ids;
	vector<int> img_IDs;


public:
	Point_3D_DB(float _x = 0, float _y = 0, float _z = 0, int _label = -1)
		:x(_x), y(_y), z(_z), clusterID(_label) {}

	void setXYZ(float _x, float _y, float _z) { x=_x, y=_y, z=_z; }
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



//
//typedef struct HIERARCHICAL_Point_3D_DB
//{
//	//float x_2d, y_2d;
//	float x, y, z;  // X, Y, Z position
//	int clusterID;  // clustered ID
//	int pt_3d_id;
//	vector<Point_2D> vec_2d_pt;
//	vector<int> img_IDs;
//
//	vector<HIERARCHICAL_Point_3D_DB> ***cube_points;
//
//public:
//	HIERARCHICAL_Point_3D_DB(float _x = 0, float _y = 0, float _z = 0, int _label = -1)
//		:x(_x), y(_y), z(_z), clusterID(_label) {}
//
//	void setXYZ(float _x, float _y, float _z) { x = _x, y = _y, z = _z; }
//	HIERARCHICAL_Point_3D_DB getXYZ() { return HIERARCHICAL_Point_3D_DB(x, y, z); }
//}H_DB_Point_3D;


typedef struct _Image
{
	int img_ID;
	string img_path;
	float quat[4];
	float camCent[3];
	float focal_len;
	float distort;
	
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




template<typename T>
std::ostream& write_typed_data(std::ostream& stream, const T& value) {
	return stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
}
template<typename T>
std::istream  & read_typed_data(std::istream& stream, T& value) {
	return stream.read(reinterpret_cast<char*>(&value), sizeof(T));
}


//namespace db_point {
//
//
//	//template<typename T>
//	//void writeVector(ofstream &out, const vector<T> &vec);
//
//	//template<typename T>
//	//vector<T> readVector(ifstream& in);
//
//	template<typename T>
//	void writeVector(ostream &out, const vector<T> &vec)
//	{
//		out << vec.size();
//
//		for (typename vector<T>::const_iterator i = vec.begin(); i != vec.end(); ++i)
//		{
//			out << *i;
//		}
//	}
//
//	template<typename T>
//	vector<T> readVector(istream &in)
//	{
//		size_t size;
//		in >> size;
//
//		vector<T> vec;
//		vec.reserve(size);
//
//		for (int i = 0; i < size; ++i)
//		{
//			T tmp;
//			in >> tmp;
//			vec.push_back(tmp);
//		}
//
//		return vec;
//	}
//
//	ostream &operator << (ostream &out, const DB_Point &i)
//	{
//		out << i.x_2d << i.y_2d;  // FIXME Read/write strings properly.
//		out << i.x << i.y << i.z;
//		out << i.clusterID;
//		out << i.pt_2d_id;
//		out << i.pt_3d_id;
//
//		return out;
//	}
//
//	istream &operator >> (istream &in, DB_Point &i)
//	{
//		// Keep in same order as operator<<(ostream &, const DB_Point &)!
//		in >> i.x_2d >> i.y_2d;  // FIXME Read/write strings properly.
//		in >> i.x >> i.y >> i.z;
//		in >> i.clusterID;
//		in >> i.pt_2d_id;
//		in >> i.pt_3d_id;
//		return in;
//	}
//}





class VOXEL_DB {
public:
	VOXEL_DB(unsigned int minPts, float eps, vector<DB_Point_3D> points, int cubeSize) {
		m_minPoints = minPts;
		m_epsilon = eps;
		m_points = points;
		m_pointSize = points.size();
		m_cubeSize = cubeSize;   // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10
	}
	~VOXEL_DB() {}

	int getTotalPointSize() { return m_pointSize; }
	int getMinimumClusterSize() { return m_minPoints; }

	void printResults(vector<DB_Point_3D>& points, int num_points, bool bAllpoints, int color[][3]);
	float PlaneFitting(vector<DB_Point_3D>& points, int num_points, vector<DB_Point_3D>& inlier_points, int color[][3]);
	float PlaneFitting_ver2(vector<DB_Point_3D>& points, int num_points, vector<DB_Point_3D>& inlier_points, int color[][3], float coef[4]);
	float PlaneFitting_ver3(vector<DB_Point_3D>& points, int num_points, vector<DB_Point_3D>& inlier_points, int color[][3], float coef[4], DB_Point_3D& minDistPt);
	float PlaneFitting_multi(vector<DB_Point_3D>& points, int num_points, vector<DB_Point_3D>& inlier_points, int color[][3], float coef[4]);

	//절대 스케일에서 크기를 정하고.... 시작하자. (loop를 돌면서 줄일필요가 없음.)
	void voxelFitting3(vector<DB_Point_3D>& points, int num_points, int color[][3], int &nLabel);
	void voxelFitting4(vector<DB_Point_3D>& points, int num_points, int color[][3], int &nLabel);
	void voxelFitting5(vector<DB_Point_3D>& points, vector<DB_Voxels>& voxels, int num_points, int color[][3], int &nLabel);  // vector<DB_Point_3D>& cube_corners
	void voxelFitting_octree(vector<DB_Point_3D>& points, int num_points, int color[][3], int &nLabel);
	void voxelFitting_aachen(vector<DB_Point_3D>& points, vector<DB_Voxels>& voxels, int num_points, int color[][3], int &nLabel);

	vector<DB_Point_3D> m_points;
	int m_numCluster;


	float calcDistance(DB_Plane phi, float x, float y, float z) {
		return abs(phi.equation.a * x + phi.equation.b * y + phi.equation.c * z + phi.equation.d) / sqrt(phi.equation.a * phi.equation.a + phi.equation.b * phi.equation.b + phi.equation.c * phi.equation.c);
	}

private:

	double point2planedistnace(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients) {
		double f1 = fabs(coefficients->values[0] * pt.x + coefficients->values[1] * pt.y + coefficients->values[2] * pt.z + coefficients->values[3]);
		double f2 = sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) + pow(coefficients->values[2], 2));
		return f1 / f2;
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

	unsigned int m_pointSize;
	unsigned int m_minPoints;
	float m_epsilon;
	int m_cubeSize;
};
