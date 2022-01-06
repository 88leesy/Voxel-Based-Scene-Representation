#pragma once

#include "dbscan.h"

// Plane 타입 정의
struct Plane
{
	double surfaceNomalX, surfaceNomalY, surfaceNomalZ;

	struct _equation {
		float a, b, c, d;
	public:
		_equation(float _a, float _b, float _c, float _d) :a(_a), b(_b), c(_c), d(_d) {}
	}equation;
public:
	Plane(float a = 0, float b = 0, float c = 0, float d = 0, double _surfaceNomalX = 0, double _surfaceNomalY = 0, double _surfaceNomalZ = 0)
		:equation(a, b, c, d), surfaceNomalX(_surfaceNomalX), surfaceNomalY(_surfaceNomalY), surfaceNomalZ(_surfaceNomalZ) {}
};

//struct XYZ
//{
//	float x_3d, y_3d, z_3d;
//	int x_2d, y_2d;
//	int label;
//	int idx;
//
//public:
//	XYZ(float _x = 0, float _y = 0, float _z = 0, int _x_2d = 0, int _y_2d = 0, int _label = -1)
//		:x_3d(_x), y_3d(_y), z_3d(_z), x_2d(_x_2d), y_2d(_y_2d), label(_label) {}
//
//	Point3f getXYZ() { return Point3f(x_3d, y_3d, z_3d); }
//};

// Plane Segmentation
class PlaneRANSAC
{
	float m_distanceThreshold;
	float m_remainPointsRatio;
	int m_minInlier;
	int m_maxPlanes;
	int m_maxIteration;
private:
	void calcPlaneCoefficient(DB_Point p1, DB_Point p2, DB_Point p3, Plane &result);
	float calcDistance(Plane &phi, DB_Point p);
public:
	PlaneRANSAC(float _distanceThreshold, float _remainPointsRatio, int _minInlier = 100, int _maxPlanes = 10, int _maxIteration = 100)
		: m_distanceThreshold(_distanceThreshold), m_remainPointsRatio(_remainPointsRatio), m_minInlier(_minInlier), m_maxPlanes(_maxPlanes), m_maxIteration(_maxIteration) {}
	/*void run(vector<DB_Point>& vec_originPointCloudData, vector<DB_Point>& vec_superpixelXYZ,
		vector<Plane> &bestInlierPlane, int &planeLabel);*/
	void run(vector<DB_Point>& vec_originPointCloudData, vector<Plane> &bestInlierPlane, int &planeLabel);
};