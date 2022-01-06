#include "dbscan.h"

int DBSCAN::run()
{
	 
	int clusterID = 1;
	vector<Point_3D_DB>::iterator iter;
	for (iter = m_points.begin(); iter != m_points.end(); ++iter)
	{
		if (iter->clusterID == UNCLASSIFIED)
		{
			if (expandCluster(*iter, clusterID) != FAILURE)
			{
				clusterID += 1;
			}
		}
	}

	m_numCluster = clusterID - 1;
	printf("m_numCluster:%d\n", clusterID-1);
	return 0;
}

int DBSCAN::expandCluster(DB_Point_3D point, int clusterID)
{
	vector<int> clusterSeeds = calculateCluster(point);

	if (clusterSeeds.size() < m_minPoints)
	{
		point.clusterID = NOISE;
		return FAILURE;
	}
	else
	{
		//int index = 0, indexCorePoint = 0;
		//vector<int>::iterator iterSeeds;
		//for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
		//{
		//	m_points.at(*iterSeeds).clusterID = clusterID;
		//	if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z)
		//	{
		//		indexCorePoint = index;
		//	}
		//	++index;
		//}
		//clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

		//for (vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
		//{
		//	vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

		//	if (clusterNeighors.size() >= m_minPoints)
		//	{
		//		vector<int>::iterator iterNeighors;
		//		for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
		//		{
		//			if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE)
		//			{
		//				if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED)
		//				{
		//					clusterSeeds.push_back(*iterNeighors);
		//					n = clusterSeeds.size();
		//				}
		//				m_points.at(*iterNeighors).clusterID = clusterID;
		//			}
		//		}
		//	}
		//}

		int index = 0, indexCorePoint = 0;
		vector<int>::iterator iterSeeds;
		for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
		{
			m_points[(*iterSeeds)].clusterID = clusterID;
			if (m_points[(*iterSeeds)].x == point.x && m_points[(*iterSeeds)].y == point.y && m_points[(*iterSeeds)].z == point.z)
			{
				indexCorePoint = index;
			}
			++index;
		}
		clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

		for (vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
		{
			vector<int> clusterNeighors = calculateCluster(m_points[(clusterSeeds[i])]);

			if (clusterNeighors.size() >= m_minPoints)
			{
				vector<int>::iterator iterNeighors;
				for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
				{
					if (m_points[(*iterNeighors)].clusterID == UNCLASSIFIED || m_points[(*iterNeighors)].clusterID == NOISE)
					{
						if (m_points[(*iterNeighors)].clusterID == UNCLASSIFIED)
						{
							clusterSeeds.push_back(*iterNeighors);
							n = clusterSeeds.size();
						}
						m_points[(*iterNeighors)].clusterID = clusterID;
					}
				}
			}
		}

		return SUCCESS;
	}
}

vector<int> DBSCAN::calculateCluster(DB_Point_3D point)
{
	int index = 0;
	vector<DB_Point_3D>::iterator iter;
	vector<int> clusterIndex;
	for (iter = m_points.begin(); iter != m_points.end(); ++iter)
	{
		if (calculateDistance(point, *iter) <= m_epsilon)
		{
			clusterIndex.push_back(index);
		}
		index++;
	}
	return clusterIndex;
}

inline double DBSCAN::calculateDistance(DB_Point_3D pointCore, DB_Point_3D pointTarget)
{
	return pow(pointCore.x - pointTarget.x, 2) + pow(pointCore.y - pointTarget.y, 2) + pow(pointCore.z - pointTarget.z, 2);
}



void DBSCAN::printResults(vector<Point_3D_DB>& points, int num_points, bool bAllpoints, float color[][3])
{
	//#define NUM_COLOR 300
	//float color[NUM_COLOR][3] = { 0 };
	//for (int i = 0; i < NUM_COLOR; i++)
	//{
	//	//color[i][0] = (float)(rand() % (NUM_COLOR+1)) / NUM_COLOR;
	//	//color[i][1] = (float)(rand() % (NUM_COLOR+1)) / NUM_COLOR;
	//	//color[i][2] = (float)(rand() % (NUM_COLOR+1)) / NUM_COLOR;
	//	color[i][0] = (rand() % 255);
	//	color[i][1] = (rand() % 255);
	//	color[i][2] = (rand() % 255);
	//}


	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZ> cloud; //생성할 PointCloud structure구조체(x,y,z) 정의 
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (bAllpoints)
	{
		cloud.is_dense = false;
		cloud.points.resize(points.size());
		int color_lb = 0;
		for (int i = 0; i < points.size(); ++i)
		{
			color_lb = points[i].clusterID-1;
			cloud.points[i].x = points[i].x;
			cloud.points[i].y = points[i].y;
			cloud.points[i].z = points[i].z;
			cloud.points[i].r = color[color_lb][0];
			cloud.points[i].g = color[color_lb][1];
			cloud.points[i].b = color[color_lb][2];
		}

		*ptr_cloud = cloud;

		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		viewer.showCloud(ptr_cloud);
		while (!viewer.wasStopped())
		{
		}
	}
	else
	{
		vector<Point_3D_DB> tmp_points;
		for (int i = 0; i < num_points; ++i)
		{
			if (points[i].clusterID != -1)
			{
				tmp_points.push_back(points[i]);
			}
		}

		// 포인트클라우드의 파라미터 설정 : width, height, is_dense
		//cloud.width = 500;
		//cloud.height = 500;
		cloud.is_dense = false;
		cloud.points.resize(tmp_points.size());

		for (int i = 0; i < tmp_points.size(); ++i)
		{
			cloud.points[i].x = tmp_points[i].x;
			cloud.points[i].y = tmp_points[i].y;
			cloud.points[i].z = tmp_points[i].z;
			int color_lb = tmp_points[i].clusterID - 1;
			cloud.points[i].r = color[color_lb][0];
			cloud.points[i].g = color[color_lb][1];
			cloud.points[i].b = color[color_lb][2];
		}

		*ptr_cloud = cloud;

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setOptimizeCoefficients(true);  // Optional
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE); //PLANE 모델 사용
		seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용 
		seg.setDistanceThreshold(0.01); //determines how close a point must be to the model in order to be considered an inlier

		seg.setInputCloud(ptr_cloud);
		seg.segment(*inliers, *coefficients);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);

		// copies all inliers of the model computed to another PointCloud
		pcl::copyPointCloud(*ptr_cloud, *inliers, *ptr_cloud_2);

		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		viewer.showCloud(ptr_cloud_2);
		while (!viewer.wasStopped())
		{
		}
	}

	
	

}


void DBSCAN::PlaneFitting(vector<DB_Point>& points, int num_points, vector<DB_Point>& inlier_points, float color[][3])
{
	//#define NUM_COLOR 100
	//float color[NUM_COLOR][3] = { 0 };
	//for (int i = 0; i < NUM_COLOR; i++)
	//{
	//	color[i][0] = (rand() % 255);
	//	color[i][1] = (rand() % 255);
	//	color[i][2] = (rand() % 255);
	//}


	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 포인트클라우드의 파라미터 설정 : width, height, is_dense
	//cloud.width = 500;
	//cloud.height = 500;
	cloud.is_dense = true;
	cloud.points.resize(points.size());


	vector<DB_Point> tmp_points;
	for (int i = 0; i < num_points; ++i)
	{
		if (points[i].clusterID != -1)
		{
			tmp_points.push_back(points[i]);
		}
	}
	for (int i = 0; i < tmp_points.size(); ++i)
	{
		cloud.points[i].x = tmp_points[i].x;
		cloud.points[i].y = tmp_points[i].y;
		cloud.points[i].z = tmp_points[i].z;
	}

	//int color_lb = 0;
	//for (int i = 0; i < points.size(); ++i)
	//{
	//	cloud.points[i].x = points[i].x;
	//	cloud.points[i].y = points[i].y;
	//	cloud.points[i].z = points[i].z;
	//}

	*ptr_cloud = cloud;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	seg.setOptimizeCoefficients(true);  // Optional
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE); //PLANE 모델 사용   //SACMODEL_PERPENDICULAR_PLANE, SACMODEL_PLANE, SACMODEL_NORMAL_PLANE
	seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용  , SAC_PROSAC
	seg.setDistanceThreshold(0.08); //determines how close a point must be to the model in order to be considered an inlier

	// Create pointcloud to publish inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
	int original_size(ptr_cloud->size());
	int n_planes(0);
	int maxIterCntPlane = 1;
	while ((ptr_cloud->size() > original_size*0.9) && (n_planes < maxIterCntPlane)) {

		seg.setInputCloud(ptr_cloud);
		seg.segment(*inliers, *coefficients);

		// Check result
		if (inliers->indices.size() == 0)
			break;

		// Iterate inliers
		double mean_error(0);
		double max_error(0);
		double min_error(100000);
		std::vector<double> err;
		
		inlier_points.clear();
		for (int i = 0; i < inliers->indices.size(); i++) {

			// Get Point
			pcl::PointXYZ pt = ptr_cloud->points[inliers->indices[i]];

			// Compute distance
			double d = point2planedistnace(pt, coefficients) * 1000;// mm
			err.push_back(d);

			// Update statistics
			mean_error += d;
			if (d > max_error) max_error = d;
			if (d < min_error) min_error = d;

			inlier_points.push_back(tmp_points[inliers->indices[i]]);
		}
		mean_error /= inliers->indices.size();

		tmp_points.clear();
		tmp_points.resize(inlier_points.size());
		copy(inlier_points.begin(), inlier_points.end(), tmp_points.begin());
		

		double sigma(0);
		for (int i = 0; i < inliers->indices.size(); i++) {

			sigma += pow(err[i] - mean_error, 2);

			// Get Point
			pcl::PointXYZ pt = ptr_cloud->points[inliers->indices[i]];

			// Copy point to noew cloud
			pcl::PointXYZRGB pt_color;
			pt_color.x = pt.x;
			pt_color.y = pt.y;
			pt_color.z = pt.z;
			pt_color.r = color[n_planes][0];
			pt_color.g = color[n_planes][1];
			pt_color.b = color[n_planes][2];

			cloud_pub->points.push_back(pt_color);
		}
		sigma = sqrt(sigma / inliers->indices.size());

		// Extract inliers
		extract.setInputCloud(ptr_cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		pcl::PointCloud<pcl::PointXYZ> cloudF;
		extract.filter(cloudF);
		ptr_cloud->swap(cloudF);

		// Nest iteration
		n_planes++;
	}

	//printf("n_planes:%d\n", n_planes);

	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//viewer.showCloud(cloud_pub);
	//while (!viewer.wasStopped())
	//{
	//}


}


void DBSCAN::voxelFitting(vector<DB_Point>& points, int num_points, float color[][3], int &nLabel)
{
	float maxX = -100, maxY = -100, maxZ = -100;
	float minX = 100, minY = 100, minZ = 100;
	float sumX = 0, sumY = 0, sumZ = 0;
	for (int i = 0; i < points.size(); ++i)
	{
		if (maxX < points[i].x) maxX = points[i].x;
		if (maxY < points[i].y) maxY = points[i].y;
		if (maxZ < points[i].z) maxZ = points[i].z;

		if (minX > points[i].x) minX = points[i].x;
		if (minY > points[i].y) minY = points[i].y;
		if (minZ > points[i].z) minZ = points[i].z;

		sumX += points[i].x;
		sumY += points[i].y;
		sumZ += points[i].z;

		//printf("x:%f, y:%f, z:%f\n", points[i].x, points[i].y, points[i].z);
	}
	float avrX = sumX / points.size();
	float avrY = sumY / points.size();
	float avrZ = sumZ / points.size();

	float stdX = 0, stdY = 0, stdZ = 0;
	for (int i = 0; i < points.size(); ++i)
	{
		stdX += pow(points[i].x - avrX, 2);
		stdY += pow(points[i].y - avrY, 2);
		stdZ += pow(points[i].z - avrZ, 2);
	}
	float varianceX = stdX / points.size();
	float varianceY = stdY / points.size();
	float varianceZ = stdZ / points.size();

	stdX = sqrt(varianceX);
	stdY = sqrt(varianceY);
	stdZ = sqrt(varianceZ);

	printf("avrX:%f, avrY:%f, avrZ:%f\n", avrX, avrY, avrZ);
	printf("varianceX:%f, varianceY:%f, varianceZ:%f\n", varianceX, varianceY, varianceZ);
	printf("stdX:%f, stdY:%f, stdZ:%f\n", stdX, stdY, stdZ);
	printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ );

	vector<DB_Point> tmpPt;
	for (int i = 0; i < points.size(); ++i)
	{
		if ((avrX - stdX) < points[i].x && points[i].x < (avrX + stdX)
			&& (avrY - stdY) < points[i].y && points[i].y < (avrY + stdY)
			&& (avrZ - stdZ) < points[i].z && points[i].z < (avrZ + stdZ))
			tmpPt.push_back(points[i]);
	}

	// 벡터 복사
	points.clear();
	points.resize(tmpPt.size());
	copy(tmpPt.begin(), tmpPt.end(), points.begin());





	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int cubeSize = 10;
	vector<DB_Point> ***cube_points = new vector<DB_Point> **[cubeSize];
	for (int x = 0; x < cubeSize; ++x)
	{
		cube_points[x] = new vector<DB_Point> *[cubeSize];
		for (int y = 0; y < cubeSize; ++y)
		{
			cube_points[x][y] = new vector<DB_Point>[cubeSize];
		}
	}

	for (int x = 1; x <= cubeSize; ++x)
	{
		float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / cubeSize);
		float maxRangeX = minX + (maxX - minX) * ((float)(x) / cubeSize);

		for (int y = 1; y <= cubeSize; ++y)
		{
			float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / cubeSize);
			float maxRangeY = minY + (maxY - minY) * ((float)(y) / cubeSize);
			for (int z = 1; z <= cubeSize; ++z)
			{
				float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / cubeSize);
				float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / cubeSize);

				float color[3] = { 0 };
				for (int i = 0; i < 3; i++)
				{
					color[0] = (rand() % 255);
					color[1] = (rand() % 255);
					color[2] = (rand() % 255);
				}
				for (int i = 0; i < points.size(); ++i)
				{
					if ( (minRangeX < points[i].x && points[i].x < maxRangeX)
						&& (minRangeY < points[i].y && points[i].y < maxRangeY)
						&& (minRangeZ < points[i].z && points[i].z < maxRangeZ) )
					{
						cube_points[x-1][y-1][z-1].push_back(points[i]);

						// Copy point to noew cloud
						pcl::PointXYZRGB pt_color;
						pt_color.x = points[i].x;
						pt_color.y = points[i].y;
						pt_color.z = points[i].z;
						pt_color.r = color[0];
						pt_color.g = color[1];
						pt_color.b = color[2];

						ptr_cloud->points.push_back(pt_color);
					}
				}
				
				// Copy point to noew cloud
				pcl::PointXYZRGB pt_color;
				pt_color.x = minRangeX;
				pt_color.y = minRangeY;
				pt_color.z = minRangeZ;
				pt_color.r = 255;
				pt_color.g = 0;
				pt_color.b = 0;

				ptr_cloud->points.push_back(pt_color);
			}
		}
	}

	for (int x = 0; x < cubeSize; x++)
	{
		for (int y = 0; y < cubeSize; y++)
		{
			delete[] cube_points[x][y];
		}
	}
	for (int x = 0; x < cubeSize; x++)
	{
		delete[] cube_points[x];
	}
	delete[] cube_points;

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(ptr_cloud);
	while (!viewer.wasStopped())
	{
	}

	printf("first processing finish!\n");
}

void DBSCAN::voxelFitting2(vector<DB_Point>& points, int num_points, float color[][3], int &nLabel)
{
	int nPrevCnt = 0;
	int nCurrCnt = 0;
	int nLabelCntTresh = 20;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<DB_Point> refinedPoints;

	//srand(0);
	//float color[300][3] = { 0 };
	//for (int i = 0; i < 300; i++)
	//{
	//	color[i][0] = (rand() % 255);
	//	color[i][1] = (rand() % 255);
	//	color[i][2] = (rand() % 255);
	//}


	refinedPoints.clear();
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());

	while (1)
	{
		float maxX = -100, maxY = -100, maxZ = -100;
		float minX = 100, minY = 100, minZ = 100;
		float sumX = 0, sumY = 0, sumZ = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			if (maxX < refinedPoints[i].x) maxX = refinedPoints[i].x;
			if (maxY < refinedPoints[i].y) maxY = refinedPoints[i].y;
			if (maxZ < refinedPoints[i].z) maxZ = refinedPoints[i].z;

			if (minX > refinedPoints[i].x) minX = refinedPoints[i].x;
			if (minY > refinedPoints[i].y) minY = refinedPoints[i].y;
			if (minZ > refinedPoints[i].z) minZ = refinedPoints[i].z;

			sumX += refinedPoints[i].x;
			sumY += refinedPoints[i].y;
			sumZ += refinedPoints[i].z;
		}
		float avrX = sumX / refinedPoints.size();
		float avrY = sumY / refinedPoints.size();
		float avrZ = sumZ / refinedPoints.size();

		float stdX = 0, stdY = 0, stdZ = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			stdX += pow(refinedPoints[i].x - avrX, 2);
			stdY += pow(refinedPoints[i].y - avrY, 2);
			stdZ += pow(refinedPoints[i].z - avrZ, 2);
		}
		float varianceX = stdX / refinedPoints.size();
		float varianceY = stdY / refinedPoints.size();
		float varianceZ = stdZ / refinedPoints.size();

		stdX = sqrt(varianceX);
		stdY = sqrt(varianceY);
		stdZ = sqrt(varianceZ);

		printf("avrX:%f, avrY:%f, avrZ:%f\n", avrX, avrY, avrZ);
		printf("varianceX:%f, varianceY:%f, varianceZ:%f\n", varianceX, varianceY, varianceZ);
		printf("stdX:%f, stdY:%f, stdZ:%f\n", stdX, stdY, stdZ);
		printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

		float cubeW = maxX - minX;
		float cubeH = maxY - minY;
		float cubeD = maxZ - minZ;
		float maxLen = max(max(cubeW, cubeH), maxZ);

		ptr_cloud->clear();
		ptr_cloud->points.clear();

		int cubeSize = 15;
		//vector<DB_Point> cube_points[10][10][10];
		//int width = cubeSize * cubeW / maxLen;   //x
		//int height = cubeSize * cubeH / maxLen;  //y
		//int depth = cubeSize * cubeD / maxLen;   //z
		int width = cubeSize;   //x
		int height = cubeSize;  //y
		int depth = cubeSize;   //z
		printf("width:%d, height:%d, depth:%d\n", width, height, depth);

		vector<DB_Point> ***cube_points = new vector<DB_Point> **[width];
		for (int x = 0; x < width; ++x)
		{
			cube_points[x] = new vector<DB_Point> *[height];
			for (int y = 0; y < height; ++y)
			{
				cube_points[x][y] = new vector<DB_Point> [depth];
			}
		}

		for (int x = 1; x <= width; ++x)
		{
			float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			for (int y = 1; y <= height; ++y)
			{
				float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
				float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
				for (int z = 1; z <= depth; ++z)
				{
					float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
					float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

					/* 
					여기 조건문 확인좀 해보자...!
					*/
					for (int i = 0; i < refinedPoints.size(); ++i)
					{
						if (   (minRangeX <= refinedPoints[i].x && refinedPoints[i].x <= maxRangeX)
							&& (minRangeY <= refinedPoints[i].y && refinedPoints[i].y <= maxRangeY)
							&& (minRangeZ <= refinedPoints[i].z && refinedPoints[i].z <= maxRangeZ) )
						{
							cube_points[x - 1][y - 1][z - 1].push_back(refinedPoints[i]);

						}
					}

					// Copy point to noew cloud
					pcl::PointXYZRGB pt_color;
					pt_color.x = minRangeX;
					pt_color.y = minRangeY;
					pt_color.z = minRangeZ;
					pt_color.r = 255;
					pt_color.g = 0;
					pt_color.b = 0;

					ptr_cloud->points.push_back(pt_color);
				}
			}
		}

		//int threshold = refinedPoints.size()/(width*height*depth);
		int threshold = 50;

		printf("threshold:%d,\n", threshold);
		nCurrCnt = 0;
		refinedPoints.clear();
		for (int x = 0; x < width; ++x)
		{
			for (int y = 0; y < height; ++y)
			{
				for (int z = 0; z < depth; ++z)
				{
					int size = cube_points[x][y][z].size();
					if (size >= threshold)
					{
						++nCurrCnt;

						//float color[3] = { 0 };
						//for (int i = 0; i < 3; i++)
						//{
						//	color[i] = (rand() % 255);
						//}

						//printf("x:%d, y:%d, z:%d\n", x, y, z);
						for (int i = 0; i < cube_points[x][y][z].size(); ++i)
						{
							cube_points[x][y][z][i].clusterID = nCurrCnt;
							refinedPoints.push_back(cube_points[x][y][z][i]);

							// Copy point to noew cloud
							pcl::PointXYZRGB pt_color;
							pt_color.x = cube_points[x][y][z][i].x;
							pt_color.y = cube_points[x][y][z][i].y;
							pt_color.z = cube_points[x][y][z][i].z;
							//pt_color.r = color[0];
							//pt_color.g = color[1];
							//pt_color.b = color[2];
							pt_color.r = color[nCurrCnt-1][0];
							pt_color.g = color[nCurrCnt-1][1];
							pt_color.b = color[nCurrCnt-1][2];


							ptr_cloud->points.push_back(pt_color);
						}
					}
					//else
					//{

					//	float color[3] = { 255, 255, 255  };

					//	//printf("x:%d, y:%d, z:%d\n", x, y, z);
					//	for (int i = 0; i < cube_points[x][y][z].size(); ++i)
					//	{
					//		//refinedPoints.push_back(cube_points[x][y][z][i]);

					//		// Copy point to noew cloud
					//		pcl::PointXYZRGB pt_color;
					//		pt_color.x = cube_points[x][y][z][i].x;
					//		pt_color.y = cube_points[x][y][z][i].y;
					//		pt_color.z = cube_points[x][y][z][i].z;
					//		pt_color.r = color[0];
					//		pt_color.g = color[1];
					//		pt_color.b = color[2];

					//		ptr_cloud->points.push_back(pt_color);
					//	}
					//}


				}
			}
		}

		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				delete[] cube_points[x][y];
			}
		}
		for (int x = 0; x < width; x++)
		{
			delete[] cube_points[x];
		}
		delete[] cube_points;


		printf("nPrevCnt:%d, nCurrCnt:%d\n", nPrevCnt, nCurrCnt);
		printf("refinedPoints:%d\n\n", refinedPoints.size());

		if (nPrevCnt == nCurrCnt && nCurrCnt>nLabelCntTresh)
		{
			//for (int i = 0; i < refinedPoints.size(); ++i)
			//{
			//	for (int j = 0; j < points.size(); ++j)
			//	{
			//		if (refinedPoints[i].x == points[j].x
			//			&& refinedPoints[i].y == points[j].y
			//			&& refinedPoints[i].z == points[j].z)
			//		{
			//			points[j].clusterID = refinedPoints[i].clusterID;
			//		}			
			//	}
			//}

			points.clear();
			points.resize(refinedPoints.size());
			copy(refinedPoints.begin(), refinedPoints.end(), points.begin());

			nLabel = nCurrCnt;

			printf("finish!");
			break;
		}
		nPrevCnt = nCurrCnt;

	}

	pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer2");
	viewer2.showCloud(ptr_cloud);
	while (!viewer2.wasStopped())
	{
	}


}




// 처음에, 너무 멀리 떨어진 점들을 제거하자.(outlier 제거)
void DBSCAN::voxelFitting3(vector<DB_Point>& points, int num_points, float color[][3], int &nLabel, int cubeSize)
{
	int nPrevCnt = 0;
	int nCurrCnt = 0;
	int nLabelCntTresh = 20;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<DB_Point> refinedPoints;

	refinedPoints.clear();
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());

	////처음에, 너무 멀리 떨어진 점들을 제거하자.(outlier 제거)
	//while (1)
	//{
	//	//refinedPoints.clear();
	//	//refinedPoints.resize(points.size());
	//	//copy(points.begin(), points.end(), refinedPoints.begin());
	//	printf("refinedPoints:%d\n", refinedPoints.size());

	//	float maxX = -100, maxY = -100, maxZ = -100;
	//	float minX = 100, minY = 100, minZ = 100;
	//	float sumX = 0, sumY = 0, sumZ = 0;
	//	for (int i = 0; i < refinedPoints.size(); ++i)
	//	{
	//		if (maxX < refinedPoints[i].x) maxX = refinedPoints[i].x;
	//		if (maxY < refinedPoints[i].y) maxY = refinedPoints[i].y;
	//		if (maxZ < refinedPoints[i].z) maxZ = refinedPoints[i].z;

	//		if (minX > refinedPoints[i].x) minX = refinedPoints[i].x;
	//		if (minY > refinedPoints[i].y) minY = refinedPoints[i].y;
	//		if (minZ > refinedPoints[i].z) minZ = refinedPoints[i].z;

	//		sumX += refinedPoints[i].x;
	//		sumY += refinedPoints[i].y;
	//		sumZ += refinedPoints[i].z;
	//	}
	//	float avrX = sumX / refinedPoints.size();
	//	float avrY = sumY / refinedPoints.size();
	//	float avrZ = sumZ / refinedPoints.size();

	//	float medianX = (maxX - minX) / 2;
	//	float medianY = (maxY - minY) / 2;
	//	float medianZ = (maxZ - minZ) / 2;

	//	for (int i = 0; i < refinedPoints.size(); ++i)
	//	{
	//		if (refinedPoints[i].x < medianX)
	//			;
	//	}



	//	printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);
	//	printf("avrX:%f, avrY:%f, avrZ:%f\n", avrX, avrY, avrZ);
	//	printf("medianX:%f, medianY:%f, medianZ:%f\n", medianX, medianY, medianZ);
	//}
	//printf("refinedPoints:%d\n", refinedPoints.size());



	int nAllCnt = 0;
	while (1)
	{
		float maxX = -100, maxY = -100, maxZ = -100;
		float minX = 100, minY = 100, minZ = 100;
		float sumX = 0, sumY = 0, sumZ = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			if (maxX < refinedPoints[i].x) maxX = refinedPoints[i].x;
			if (maxY < refinedPoints[i].y) maxY = refinedPoints[i].y;
			if (maxZ < refinedPoints[i].z) maxZ = refinedPoints[i].z;

			if (minX > refinedPoints[i].x) minX = refinedPoints[i].x;
			if (minY > refinedPoints[i].y) minY = refinedPoints[i].y;
			if (minZ > refinedPoints[i].z) minZ = refinedPoints[i].z;

			sumX += refinedPoints[i].x;
			sumY += refinedPoints[i].y;
			sumZ += refinedPoints[i].z;
		}
		float avrX = sumX / refinedPoints.size();
		float avrY = sumY / refinedPoints.size();
		float avrZ = sumZ / refinedPoints.size();

		printf("avrX:%f, avrY:%f, avrZ:%f\n", avrX, avrY, avrZ);
		printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

		float cubeW = maxX - minX;
		float cubeH = maxY - minY;
		float cubeD = maxZ - minZ;
		float maxLen = max(max(cubeW, cubeH), maxZ);

		ptr_cloud->clear();
		ptr_cloud->points.clear();

		int _cubeSize = 10;
		//vector<DB_Point> cube_points[10][10][10];
		int width = _cubeSize * cubeW / maxLen;   //x
		int height = _cubeSize * cubeH / maxLen;  //y
		int depth = _cubeSize * cubeD / maxLen;   //z
		if (width <= 0) width = 1;
		if (height <= 0) height = 1;
		if (depth <= 0) depth = 1;
		printf("width:%d, height:%d, depth:%d\n", width, height, depth);

		vector<DB_Point> ***cube_points = new vector<DB_Point> **[width];
		for (int x = 0; x < width; ++x)
		{
			cube_points[x] = new vector<DB_Point> *[height];
			for (int y = 0; y < height; ++y)
			{
				cube_points[x][y] = new vector<DB_Point>[depth];
			}
		}

		for (int x = 1; x <= width; ++x)
		{
			float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			for (int y = 1; y <= height; ++y)
			{
				float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
				float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
				for (int z = 1; z <= depth; ++z)
				{
					float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
					float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

					/*
					여기 조건문 확인좀 해보자...!
					*/
					for (int i = 0; i < refinedPoints.size(); ++i)
					{
						if ((minRangeX <= refinedPoints[i].x && refinedPoints[i].x <= maxRangeX)
							&& (minRangeY <= refinedPoints[i].y && refinedPoints[i].y <= maxRangeY)
							&& (minRangeZ <= refinedPoints[i].z && refinedPoints[i].z <= maxRangeZ))
						{
							cube_points[x - 1][y - 1][z - 1].push_back(refinedPoints[i]);

						}
					}

					//// Copy point to noew cloud
					//pcl::PointXYZRGB pt_color;
					//pt_color.x = minRangeX;
					//pt_color.y = minRangeY;
					//pt_color.z = minRangeZ;
					//pt_color.r = 255;
					//pt_color.g = 0;
					//pt_color.b = 0;

					//ptr_cloud->points.push_back(pt_color);
				}
			}
		}

		int threshold = refinedPoints.size() * 0.01;
		//int threshold = 50;
		printf("11 refinedPoints:%d\n\n", refinedPoints.size());

		printf("threshold:%d\n", threshold);
		nCurrCnt = 0;
		refinedPoints.clear();
		for (int x = 0; x < width; ++x)
		{
			for (int y = 0; y < height; ++y)
			{
				for (int z = 0; z < depth; ++z)
				{
					int size = cube_points[x][y][z].size();
					//printf("x:%d, y:%d, z:%d,  size:%d\n", x, y, z, size);
					if (size >= threshold)
					{
						++nCurrCnt;

						//printf("x:%d, y:%d, z:%d,  nCurrCnt:%d\n", x, y, z, nCurrCnt);
						for (int i = 0; i < cube_points[x][y][z].size(); ++i)
						{
							cube_points[x][y][z][i].clusterID = nCurrCnt;
							refinedPoints.push_back(cube_points[x][y][z][i]);

							// Copy point to noew cloud
							pcl::PointXYZRGB pt_color;
							pt_color.x = cube_points[x][y][z][i].x;
							pt_color.y = cube_points[x][y][z][i].y;
							pt_color.z = cube_points[x][y][z][i].z;

							pt_color.r = color[nCurrCnt - 1][0];
							pt_color.g = color[nCurrCnt - 1][1];
							pt_color.b = color[nCurrCnt - 1][2];


							ptr_cloud->points.push_back(pt_color);
						}
					}


				}
			}
		}

		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				delete[] cube_points[x][y];
			}
		}
		for (int x = 0; x < width; x++)
		{
			delete[] cube_points[x];
		}
		delete[] cube_points;


		printf("nPrevCnt:%d, nCurrCnt:%d\n", nPrevCnt, nCurrCnt);
		printf("22 refinedPoints:%d\n\n", refinedPoints.size());

		int debug = 1;
		if (debug)
		{
			maxX = -100, maxY = -100, maxZ = -100;
			minX = 100, minY = 100, minZ = 100;
			for (int i = 0; i < refinedPoints.size(); ++i)
			{
				if (maxX < refinedPoints[i].x) maxX = refinedPoints[i].x;
				if (maxY < refinedPoints[i].y) maxY = refinedPoints[i].y;
				if (maxZ < refinedPoints[i].z) maxZ = refinedPoints[i].z;

				if (minX > refinedPoints[i].x) minX = refinedPoints[i].x;
				if (minY > refinedPoints[i].y) minY = refinedPoints[i].y;
				if (minZ > refinedPoints[i].z) minZ = refinedPoints[i].z;
			}
			printf("2222 maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

			cubeW = maxX - minX;
			cubeH = maxY - minY;
			cubeD = maxZ - minZ;
			maxLen = max(max(cubeW, cubeH), maxZ);

			width = _cubeSize * cubeW / maxLen;   //x
			height = _cubeSize * cubeH / maxLen;  //y
			depth = _cubeSize * cubeD / maxLen;   //z
			if (width <= 0) width = 1;
			if (height <= 0) height = 1;
			if (depth <= 0) depth = 1;
			printf("2222  width:%d, height:%d, depth:%d\n\n", width, height, depth);

			for (int x = 1; x <= width; ++x)
			{
				float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
				float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

				for (int y = 1; y <= height; ++y)
				{
					float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
					float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
					for (int z = 1; z <= depth; ++z)
					{
						float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
						float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

						// Copy point to noew cloud
						pcl::PointXYZRGB pt_color;
						pt_color.x = minRangeX;
						pt_color.y = minRangeY;
						pt_color.z = minRangeZ;
						pt_color.r = 255;
						pt_color.g = 0;
						pt_color.b = 0;

						ptr_cloud->points.push_back(pt_color);
					}
				}
			}

		}

		if (nAllCnt > 1) break;
		++nAllCnt;

	}  //while


	printf("\n -------------------------------\n");

	float maxX = -100, maxY = -100, maxZ = -100;
	float minX = 100, minY = 100, minZ = 100;
	float sumX = 0, sumY = 0, sumZ = 0;
	for (int i = 0; i < refinedPoints.size(); ++i)
	{
		if (maxX < refinedPoints[i].x) maxX = refinedPoints[i].x;
		if (maxY < refinedPoints[i].y) maxY = refinedPoints[i].y;
		if (maxZ < refinedPoints[i].z) maxZ = refinedPoints[i].z;

		if (minX > refinedPoints[i].x) minX = refinedPoints[i].x;
		if (minY > refinedPoints[i].y) minY = refinedPoints[i].y;
		if (minZ > refinedPoints[i].z) minZ = refinedPoints[i].z;
	}
	printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

	float cubeW = maxX - minX;
	float cubeH = maxY - minY;
	float cubeD = maxZ - minZ;
	float maxLen = max(max(cubeW, cubeH), maxZ);

	ptr_cloud->clear();
	ptr_cloud->points.clear();

	//int cubeSize = 70;
	int width = cubeSize * cubeW / maxLen;   //x
	int height = cubeSize * cubeH / maxLen;  //y
	int depth = cubeSize * cubeD / maxLen;   //z
	if (width <= 0) width = 1;
	if (height <= 0) height = 1;
	if (depth <= 0) depth = 1;
	printf("width:%d, height:%d, depth:%d\n", width, height, depth);

	vector<DB_Point> ***cube_points = new vector<DB_Point> **[width];
	for (int x = 0; x < width; ++x)
	{
		cube_points[x] = new vector<DB_Point> *[height];
		for (int y = 0; y < height; ++y)
		{
			cube_points[x][y] = new vector<DB_Point>[depth];
		}
	}

	for (int x = 1; x <= width; ++x)
	{
		float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
		float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

		for (int y = 1; y <= height; ++y)
		{
			float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
			float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
			for (int z = 1; z <= depth; ++z)
			{
				float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
				float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

				for (int i = 0; i < refinedPoints.size(); ++i)
				{
					if ((minRangeX <= refinedPoints[i].x && refinedPoints[i].x <= maxRangeX)
						&& (minRangeY <= refinedPoints[i].y && refinedPoints[i].y <= maxRangeY)
						&& (minRangeZ <= refinedPoints[i].z && refinedPoints[i].z <= maxRangeZ))
					{
						cube_points[x - 1][y - 1][z - 1].push_back(refinedPoints[i]);

					}
				}
			}
		}
	}

	int threshold = refinedPoints.size() * 0.001;
	printf("--- refinedPoints:%d\n\n", refinedPoints.size());
	threshold = 50;
	printf("threshold:%d\n", threshold);
	nCurrCnt = 0;
	refinedPoints.clear();
	ptr_cloud->clear();
	ptr_cloud->points.clear();
	for (int x = 0; x < width; ++x)
	{
		for (int y = 0; y < height; ++y)
		{
			for (int z = 0; z < depth; ++z)
			{
				int size = cube_points[x][y][z].size();
				if (size > threshold)
				{
					++nCurrCnt;

					//printf("x:%d, y:%d, z:%d,  nCurrCnt:%d\n", x, y, z, nCurrCnt);
					for (int i = 0; i < cube_points[x][y][z].size(); ++i)
					{
						cube_points[x][y][z][i].clusterID = nCurrCnt;
						refinedPoints.push_back(cube_points[x][y][z][i]);

						// Copy point to noew cloud
						pcl::PointXYZRGB pt_color;
						pt_color.x = cube_points[x][y][z][i].x;
						pt_color.y = cube_points[x][y][z][i].y;
						pt_color.z = cube_points[x][y][z][i].z;

						pt_color.r = color[nCurrCnt - 1][0];
						pt_color.g = color[nCurrCnt - 1][1];
						pt_color.b = color[nCurrCnt - 1][2];

						ptr_cloud->points.push_back(pt_color);
					}
				}


			}
		}
	}

	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			delete[] cube_points[x][y];
		}
	}
	for (int x = 0; x < width; x++)
	{
		delete[] cube_points[x];
	}
	delete[] cube_points;


	printf("--- nPrevCnt:%d, nCurrCnt:%d\n", nPrevCnt, nCurrCnt);
	printf("--- refinedPoints:%d\n\n", refinedPoints.size());


	pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer2");
	viewer2.showCloud(ptr_cloud);
	while (!viewer2.wasStopped())
	{
	}



	points.clear();
	points.resize(refinedPoints.size());
	copy(refinedPoints.begin(), refinedPoints.end(), points.begin());

	nLabel = nCurrCnt;

	printf("finish!");

}