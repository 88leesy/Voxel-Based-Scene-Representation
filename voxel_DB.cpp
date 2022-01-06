#include "voxel_DB.h"


void VOXEL_DB::printResults(vector<DB_Point_3D>& points, int num_points, bool bAllpoints, int color[][3])
{
	//#define NUM_COLOR 300
	//int color[NUM_COLOR][3] = { 0 };
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


	if (bAllpoints)
	{
		cloud.is_dense = false;
		cloud.points.resize(points.size());
		int color_lb = 0;
		for (int i = 0; i < points.size(); ++i)
		{
			color_lb = points[i].clusterID - 1;
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
		vector<DB_Point_3D> tmp_points;
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


float VOXEL_DB::PlaneFitting(vector<DB_Point_3D>& points, int num_points, vector<DB_Point_3D>& inlier_points, int color[][3])
{
	//#define NUM_COLOR 100
	//int color[NUM_COLOR][3] = { 0 };
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


	vector<DB_Point_3D> tmp_points;
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
	seg.setModelType(pcl::SACMODEL_PLANE); //PLANE 모델 사용   //SACMODEL_PERPENDICULAR_PLANE, SACMODEL_PLANE, SACMODEL_NORMAL_PLANE, SACMODEL_PARALLEL_PLANE
	seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용  , SAC_PROSAC
	//seg.setDistanceThreshold(0.08); //determines how close a point must be to the model in order to be considered an inlier
	seg.setDistanceThreshold(0.1);  //0.1, 


	// Create pointcloud to publish inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
	int original_size(ptr_cloud->size());
	int n_planes(0);
	int maxIterCntPlane = 1;
	float meanErr = 0;
	while ((ptr_cloud->size() > original_size*0.5) && (n_planes < maxIterCntPlane)) {

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
		meanErr = mean_error;

		//float variance = 0;
		//float sum = 0;
		//for (int i = 0; i < err.size(); i++) {
		//	sum += (meanErr - err[i])*(meanErr - err[i]);
		//}
		//variance = sum/inliers->indices.size();
		//printf("meanErr:%f, variance:%f, std:%f\n", meanErr, variance, sqrt(variance));


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

	return meanErr;

	//printf("n_planes:%d\n", n_planes);

	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//viewer.showCloud(cloud_pub);
	//while (!viewer.wasStopped())
	//{
	//}


}

// plane coefficient도 리턴.

float VOXEL_DB::PlaneFitting_ver2(vector<DB_Point_3D>& points, int num_points, vector<DB_Point_3D>& inlier_points, int color[][3], float coef[4])
{

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 포인트클라우드의 파라미터 설정 : width, height, is_dense
	//cloud.width = 500;
	//cloud.height = 500;
	cloud.is_dense = true;
	cloud.points.resize(points.size());


	vector<DB_Point_3D> tmp_points;
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

	*ptr_cloud = cloud;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	seg.setOptimizeCoefficients(true);  // Optional
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE); //PLANE 모델 사용   //SACMODEL_PERPENDICULAR_PLANE, SACMODEL_PLANE, SACMODEL_NORMAL_PLANE, SACMODEL_PARALLEL_PLANE
	seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용  , SAC_PROSAC
	//seg.setDistanceThreshold(0.08); //determines how close a point must be to the model in order to be considered an inlier
	seg.setDistanceThreshold(0.1);  //0.1


	// Create pointcloud to publish inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
	int original_size(ptr_cloud->size());
	int n_planes(0);
	int maxIterCntPlane = 1;
	float meanErr = 0;
	int minErrIdx = 0;
	DB_Point_3D minDistPt;
	while ((ptr_cloud->size() > original_size*0.5) && (n_planes < maxIterCntPlane)) {

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
			if (d < min_error)
			{
				min_error = d;
				minErrIdx = i;
			}

			inlier_points.push_back(tmp_points[inliers->indices[i]]);
		}
		mean_error /= inliers->indices.size();
		meanErr = mean_error;

		minDistPt = tmp_points[inliers->indices[minErrIdx]];


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
	
	coef[0] = coefficients->values[0];
	coef[1] = coefficients->values[1];
	coef[2] = coefficients->values[2];
	coef[3] = coefficients->values[3];

	return meanErr;

}

float VOXEL_DB::PlaneFitting_ver3(vector<DB_Point_3D>& points, int num_points, vector<DB_Point_3D>& inlier_points, int color[][3], 
	float coef[4], DB_Point_3D& minDistPt)
{

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 포인트클라우드의 파라미터 설정 : width, height, is_dense
	//cloud.width = 500;
	//cloud.height = 500;
	cloud.is_dense = true;
	cloud.points.resize(points.size());


	vector<DB_Point_3D> tmp_points;
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

	*ptr_cloud = cloud;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	seg.setOptimizeCoefficients(true);  // Optional
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE); //PLANE 모델 사용   //SACMODEL_PERPENDICULAR_PLANE, SACMODEL_PLANE, SACMODEL_NORMAL_PLANE, SACMODEL_PARALLEL_PLANE
	seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용  , SAC_PROSAC
	//seg.setDistanceThreshold(0.08); //determines how close a point must be to the model in order to be considered an inlier
	seg.setDistanceThreshold(0.1);  //0.1


	// Create pointcloud to publish inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
	int original_size(ptr_cloud->size());
	int n_planes(0);
	int maxIterCntPlane = 1;
	float meanErr = 0;
	int minErrIdx = 0;
	while ((ptr_cloud->size() > original_size*0.5) && (n_planes < maxIterCntPlane)) {

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
			if (d < min_error)
			{
				min_error = d;
				minErrIdx = i;
			}

			inlier_points.push_back(tmp_points[inliers->indices[i]]);
		}
		mean_error /= inliers->indices.size();
		meanErr = mean_error;

		minDistPt = tmp_points[inliers->indices[minErrIdx]];


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

	coef[0] = coefficients->values[0];
	coef[1] = coefficients->values[1];
	coef[2] = coefficients->values[2];
	coef[3] = coefficients->values[3];

	return meanErr;

}


float VOXEL_DB::PlaneFitting_multi(vector<DB_Point_3D>& points, int num_points, vector<DB_Point_3D>& inlier_points, int color[][3], float coef[4])
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 포인트클라우드의 파라미터 설정 : width, height, is_dense
	//cloud.width = 500;
	//cloud.height = 500;
	cloud.is_dense = true;
	cloud.points.resize(points.size());


	vector<DB_Point_3D> tmp_points;
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

	*ptr_cloud = cloud;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	seg.setOptimizeCoefficients(true);  // Optional
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE); //PLANE 모델 사용   //SACMODEL_PERPENDICULAR_PLANE, SACMODEL_PLANE, SACMODEL_NORMAL_PLANE, SACMODEL_PARALLEL_PLANE
	seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용  , SAC_PROSAC
	//seg.setDistanceThreshold(0.08); //determines how close a point must be to the model in order to be considered an inlier
	seg.setDistanceThreshold(0.1);  //0.1


	// Create pointcloud to publish inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
	int original_size(ptr_cloud->size());
	int n_planes(0);
	int maxIterCntPlane = 3;
	float meanErr = 0;
	int minErrIdx = 0;
	DB_Point_3D minDistPt;

	//inlier_points.clear();
	while ((ptr_cloud->size() > original_size*0.3) && (n_planes < maxIterCntPlane)) {

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
			if (d < min_error)
			{
				min_error = d;
				minErrIdx = i;
			}

			inlier_points.push_back(tmp_points[inliers->indices[i]]);
		}
		mean_error /= inliers->indices.size();
		meanErr = mean_error;

		minDistPt = tmp_points[inliers->indices[minErrIdx]];


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

	coef[0] = coefficients->values[0];
	coef[1] = coefficients->values[1];
	coef[2] = coefficients->values[2];
	coef[3] = coefficients->values[3];

	printf("n_planes:%d\n", n_planes);

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud_pub);
	while (!viewer.wasStopped())
	{
	}

	return meanErr;

}






// 처음에, 너무 멀리 떨어진 점들을 제거하자.(outlier 제거)
void VOXEL_DB::voxelFitting3(vector<DB_Point_3D>& points, int num_points, int color[][3], int &nLabel)
{
	int nPrevCnt = 0;
	int nCurrCnt = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<DB_Point_3D> refinedPoints;

	refinedPoints.clear();
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());


	int debug = 1;
	int _cubeSize = 30;    // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10

	int nAllCnt = 0;
	while (1)
	{
		float maxX = -100000, maxY = -100000, maxZ = -100000;
		float minX = 100000, minY = 100000, minZ = 100000;
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
		float maxLen = max(max(cubeW, cubeH), cubeD);

		ptr_cloud->clear();
		ptr_cloud->points.clear();

		//vector<DB_Point_3D> cube_points[10][10][10];
		int width = _cubeSize * cubeW / maxLen;   //x
		int height = _cubeSize * cubeH / maxLen;  //y
		int depth = _cubeSize * cubeD / maxLen;   //z
		if (width <= 0) width = 1;
		if (height <= 0) height = 1;
		if (depth <= 0) depth = 1;
		printf("width:%d, height:%d, depth:%d\n", width, height, depth);

		vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
		for (int x = 0; x < width; ++x)
		{
			cube_points[x] = new vector<DB_Point_3D> *[height];
			for (int y = 0; y < height; ++y)
			{
				cube_points[x][y] = new vector<DB_Point_3D>[depth];
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

				}
			}
		}

		int threshold = refinedPoints.size() * 0.001;   // Aachen = 0.0001;  others = 0.01
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

		
		if (debug)
		{
			//maxX = -1000000, maxY = -1000000, maxZ = -1000000;
			//minX = 10000000, minY = 1000000, minZ = 1000000;
			//for (int i = 0; i < refinedPoints.size(); ++i)
			//{
			//	if (maxX < refinedPoints[i].x) maxX = refinedPoints[i].x;
			//	if (maxY < refinedPoints[i].y) maxY = refinedPoints[i].y;
			//	if (maxZ < refinedPoints[i].z) maxZ = refinedPoints[i].z;

			//	if (minX > refinedPoints[i].x) minX = refinedPoints[i].x;
			//	if (minY > refinedPoints[i].y) minY = refinedPoints[i].y;
			//	if (minZ > refinedPoints[i].z) minZ = refinedPoints[i].z;
			//}
			//printf("2222 maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

			//cubeW = maxX - minX;
			//cubeH = maxY - minY;
			//cubeD = maxZ - minZ;
			//maxLen = max(max(cubeW, cubeH), cubeD);

			//width = _cubeSize * cubeW / maxLen;   //x
			//height = _cubeSize * cubeH / maxLen;  //y
			//depth = _cubeSize * cubeD / maxLen;   //z
			//if (width <= 0) width = 1;
			//if (height <= 0) height = 1;
			//if (depth <= 0) depth = 1;
			//printf("2222  width:%d, height:%d, depth:%d\n\n", width, height, depth);

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

		if (nAllCnt > 2) break;   // CMU, Aachen => 2
		++nAllCnt;

	}  //while


	if (1) {
		//결과 보기
		pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer2");
		viewer1.showCloud(ptr_cloud);
		while (!viewer1.wasStopped())
		{
		}
	}



	printf("\n -------------------------------------------------------\n");

	float maxX = -100000, maxY = -100000, maxZ = -100000;
	float minX = 1000000, minY = 1000000, minZ = 1000000;
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
	float maxLen = max(max(cubeW, cubeH), cubeD);

	ptr_cloud->clear();
	ptr_cloud->points.clear();

	int width = m_cubeSize * cubeW / maxLen;   //x
	int height = m_cubeSize * cubeH / maxLen;  //y
	int depth = m_cubeSize * cubeD / maxLen;   //z
	if (width <= 0) width = 1;
	if (height <= 0) height = 1;
	if (depth <= 0) depth = 1;
	printf("width:%d, height:%d, depth:%d\n", width, height, depth);

	vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
	for (int x = 0; x < width; ++x)
	{
		cube_points[x] = new vector<DB_Point_3D> *[height];
		for (int y = 0; y < height; ++y)
		{
			cube_points[x][y] = new vector<DB_Point_3D>[depth];
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


	//int threshold = refinedPoints.size() * 0.001;
	printf("--- refinedPoints:%d\n\n", refinedPoints.size());
	int threshold = 20;  //20 or 50
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


	if (debug)
	{
		//maxX = -1000000, maxY = -1000000, maxZ = -1000000;
		//minX = 10000000, minY = 1000000, minZ = 1000000;
		//for (int i = 0; i < refinedPoints.size(); ++i)
		//{
		//	if (maxX < refinedPoints[i].x) maxX = refinedPoints[i].x;
		//	if (maxY < refinedPoints[i].y) maxY = refinedPoints[i].y;
		//	if (maxZ < refinedPoints[i].z) maxZ = refinedPoints[i].z;

		//	if (minX > refinedPoints[i].x) minX = refinedPoints[i].x;
		//	if (minY > refinedPoints[i].y) minY = refinedPoints[i].y;
		//	if (minZ > refinedPoints[i].z) minZ = refinedPoints[i].z;
		//}
		//printf("debug maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

		//cubeW = maxX - minX;
		//cubeH = maxY - minY;
		//cubeD = maxZ - minZ;
		//maxLen = max(max(cubeW, cubeH), cubeD);

		//width = m_cubeSize * cubeW / maxLen;   //x
		//height = m_cubeSize * cubeH / maxLen;  //y
		//depth = m_cubeSize * cubeD / maxLen;   //z
		//if (width <= 0) width = 1;
		//if (height <= 0) height = 1;
		//if (depth <= 0) depth = 1;
		//printf("debug  width:%d, height:%d, depth:%d\n\n", width, height, depth);

		for (int x = 1; x <= width+1; ++x)
		{
			float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			for (int y = 1; y <= height+1; ++y)
			{
				float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
				float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
				for (int z = 1; z <= depth+1; ++z)
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


    //결과 보기
	pcl::visualization::CloudViewer viewer2("Cloud Viewer");
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



void VOXEL_DB::voxelFitting4(vector<DB_Point_3D>& points, int num_points, int color[][3], int &nLabel)
{
	int nPrevCnt = 0;
	int nCurrCnt = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<DB_Point_3D> refinedPoints;

	refinedPoints.clear();
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());


	int debug = 1;
	int _cubeSize = 10;  // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10

	int nAllCnt = 0;
	while (1)
	{
		float maxX = -100000, maxY = -100000, maxZ = -100000;
		float minX = 100000, minY = 100000, minZ = 100000;
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
		float maxLen = max(max(cubeW, cubeH), cubeD);

		ptr_cloud->clear();
		ptr_cloud->points.clear();


		int width = _cubeSize * cubeW / maxLen;   //x
		int height = _cubeSize * cubeH / maxLen;  //y
		int depth = _cubeSize * cubeD / maxLen;   //z
		if (width <= 0) width = 1;
		if (height <= 0) height = 1;
		if (depth <= 0) depth = 1;
		printf("width:%d, height:%d, depth:%d\n", width, height, depth);

		vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
		for (int x = 0; x < width; ++x)
		{
			cube_points[x] = new vector<DB_Point_3D> *[height];
			for (int y = 0; y < height; ++y)
			{
				cube_points[x][y] = new vector<DB_Point_3D>[depth];
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

				}
			}
		}

		int threshold = refinedPoints.size() * 0.01;   // Aachen = 0.0001;
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


		if (debug)
		{
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

		if (nAllCnt > 1) break;   // CMU, Aachen => 2, cambridge = 1
		++nAllCnt;

	}  //while


	if (1) {
		//결과 보기
		pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer2");
		viewer1.showCloud(ptr_cloud);
		while (!viewer1.wasStopped())
		{
		}
	}

	//outlier 제거 후 첫 클러스터링...
	printf("\n -------------------------------------------------------\n");
	printf("\n --- Hierarchical Clustering initialization... [level 0] ---\n");

	float maxX = -100000, maxY = -100000, maxZ = -100000;
	float minX = 1000000, minY = 1000000, minZ = 1000000;
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
	float maxLen = max(max(cubeW, cubeH), cubeD);

	ptr_cloud->clear();
	ptr_cloud->points.clear();
	nCurrCnt = 0;

	//int width = 10 * cubeW / maxLen;   //x
	//int height = 10 * cubeH / maxLen;  //y
	//int depth = 10 * cubeD / maxLen;   //z
	int width = m_cubeSize;   //x
	int height = m_cubeSize;  //y
	int depth = m_cubeSize;   //z
	if (width <= 0) width = 1;
	if (height <= 0) height = 1;
	if (depth <= 0) depth = 1;
	printf("width:%d, height:%d, depth:%d\n", width, height, depth);

	vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
	for (int x = 0; x < width; ++x)
	{
		cube_points[x] = new vector<DB_Point_3D> *[height];
		for (int y = 0; y < height; ++y)
		{
			cube_points[x][y] = new vector<DB_Point_3D>[depth];
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

				++nCurrCnt;

				for (int i = 0; i < refinedPoints.size(); ++i)
				{
					if ((minRangeX <= refinedPoints[i].x && refinedPoints[i].x <= maxRangeX)
						&& (minRangeY <= refinedPoints[i].y && refinedPoints[i].y <= maxRangeY)
						&& (minRangeZ <= refinedPoints[i].z && refinedPoints[i].z <= maxRangeZ))
					{
						refinedPoints[i].clusterID = nCurrCnt;
						cube_points[x - 1][y - 1][z - 1].push_back(refinedPoints[i]);
					}
				}
			}
		}
	}

	//To do!!!! - 2021.03.15 - 계층적 클러스터링 구현하기!
	int level = 2;
	nCurrCnt = 0;
	for (int lv = 1; lv < level; ++lv)
	{
		printf("\n -------------------------------------------------------\n");
		printf("\n --- Hierarchical Clustering [level:%d] ---\n", lv);


		for (int x = 0; x < width; ++x)
		{
			for (int y = 0; y < height; ++y)
			{
				for (int z = 0; z < depth; ++z)
				{
					
					vector<DB_Point_3D> tmp_db_pt = cube_points[x][y][z];

					float maxX = -100000, maxY = -100000, maxZ = -100000;
					float minX = 1000000, minY = 1000000, minZ = 1000000;
					float sumX = 0, sumY = 0, sumZ = 0;
					for (int i = 0; i < tmp_db_pt.size(); ++i)
					{
						if (maxX < tmp_db_pt[i].x) maxX = tmp_db_pt[i].x;
						if (maxY < tmp_db_pt[i].y) maxY = tmp_db_pt[i].y;
						if (maxZ < tmp_db_pt[i].z) maxZ = tmp_db_pt[i].z;

						if (minX > tmp_db_pt[i].x) minX = tmp_db_pt[i].x;
						if (minY > tmp_db_pt[i].y) minY = tmp_db_pt[i].y;
						if (minZ > tmp_db_pt[i].z) minZ = tmp_db_pt[i].z;
					}
					printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);


					//float cubeW = maxX - minX;
					//float cubeH = maxY - minY;
					//float cubeD = maxZ - minZ;
					//float maxLen = max(max(cubeW, cubeH), cubeD);
					//int width = m_cubeSize * cubeW / maxLen;   //x
					//int height = m_cubeSize * cubeH / maxLen;  //y
					//int depth = m_cubeSize * cubeD / maxLen;   //z
					int width = m_cubeSize;   //x
					int height = m_cubeSize;  //y
					int depth = m_cubeSize;   //z
					if (width <= 0) width = 1;
					if (height <= 0) height = 1;
					if (depth <= 0) depth = 1;
					printf("width:%d, height:%d, depth:%d\n", width, height, depth);


					for (int xx = 1; xx <= width; ++xx)
					{
						float minRangeX = minX + (maxX - minX) * ((float)(xx - 1) / width);
						float maxRangeX = minX + (maxX - minX) * ((float)(xx) / width);

						for (int yy = 1; yy <= height; ++yy)
						{
							float minRangeY = minY + (maxY - minY) * ((float)(yy - 1) / height);
							float maxRangeY = minY + (maxY - minY) * ((float)(yy) / height);
							for (int zz = 1; zz <= depth; ++zz)
							{
								float minRangeZ = minZ + (maxZ - minZ) * ((float)(zz - 1) / depth);
								float maxRangeZ = minZ + (maxZ - minZ) * ((float)(zz) / depth);

								++nCurrCnt;

								for (int i = 0; i < tmp_db_pt.size(); ++i)
								{
									if ((minRangeX <= tmp_db_pt[i].x && tmp_db_pt[i].x <= maxRangeX)
										&& (minRangeY <= tmp_db_pt[i].y && tmp_db_pt[i].y <= maxRangeY)
										&& (minRangeZ <= tmp_db_pt[i].z && tmp_db_pt[i].z <= maxRangeZ))
									{
										tmp_db_pt[i].clusterID = nCurrCnt;
										//cube_points[x - 1][y - 1][z - 1].push_back(tmp_db_pt[i]);
									}
								}
							}
						}
					}

					cube_points[x][y][z] = tmp_db_pt;
					
					
				}  //z
			}
		}
	}



	printf("--- refinedPoints:%d\n\n", refinedPoints.size());
	int threshold = 20;  
	printf("threshold:%d\n", threshold);
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
					for (int i = 0; i < cube_points[x][y][z].size(); ++i)
					{
						refinedPoints.push_back(cube_points[x][y][z][i]);

						int nLabel = cube_points[x][y][z][i].clusterID;

						// Copy point to noew cloud
						pcl::PointXYZRGB pt_color;
						pt_color.x = cube_points[x][y][z][i].x;
						pt_color.y = cube_points[x][y][z][i].y;
						pt_color.z = cube_points[x][y][z][i].z;

						pt_color.r = color[nLabel - 1][0];
						pt_color.g = color[nLabel - 1][1];
						pt_color.b = color[nLabel - 1][2];

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


	if (debug)
	{
		for (int x = 1; x <= width + 1; ++x)
		{
			float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			for (int y = 1; y <= height + 1; ++y)
			{
				float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
				float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
				for (int z = 1; z <= depth + 1; ++z)
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

	//결과 보기
	pcl::visualization::CloudViewer viewer2("Cloud Viewer");
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


void VOXEL_DB::voxelFitting5(vector<DB_Point_3D>& points, vector<DB_Voxels>& voxels, int num_points, int color[][3], int &nLabel)
{
	int nPrevCnt = 0;
	int nCurrCnt = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<DB_Point_3D> refinedPoints;

	refinedPoints.clear();
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());

	bool bShow = 1;
	int debug = 0;
	int _cubeSize = 15;    // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10

	int nAllCnt = 0;
	while (1)
	{
		float maxX = -100000, maxY = -100000, maxZ = -100000;
		float minX = 100000, minY = 100000, minZ = 100000;
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
		float maxLen = max(max(cubeW, cubeH), cubeD);

		ptr_cloud->clear();
		ptr_cloud->points.clear();

		//vector<DB_Point_3D> cube_points[10][10][10];
		int width = _cubeSize * cubeW / maxLen;   //x
		int height = _cubeSize * cubeH / maxLen;  //y
		int depth = _cubeSize * cubeD / maxLen;   //z
		if (width <= 0) width = 1;
		if (height <= 0) height = 1;
		if (depth <= 0) depth = 1;
		printf("width:%d, height:%d, depth:%d\n", width, height, depth);

		vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
		for (int x = 0; x < width; ++x)
		{
			cube_points[x] = new vector<DB_Point_3D> *[height];
			for (int y = 0; y < height; ++y)
			{
				cube_points[x][y] = new vector<DB_Point_3D>[depth];
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

		int threshold = refinedPoints.size() * 0.001;   // Aachen = 0.0001;,  0.01
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


		if (debug)
		{

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

		if (nAllCnt > 1) break;   // CMU, Aachen => 2 , Cambridge : 1
		++nAllCnt;

	}  //while


	if (bShow) {
		//결과 보기
		pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer2");
		viewer1.showCloud(ptr_cloud);
		while (!viewer1.wasStopped())
		{
		}
	}



	printf("\n -------------------------------------------------------\n");

	float maxX = -100000, maxY = -100000, maxZ = -100000;
	float minX = 1000000, minY = 1000000, minZ = 1000000;
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
	float maxLen = max(max(cubeW, cubeH), cubeD);

	ptr_cloud->clear();
	ptr_cloud->points.clear();

	int width = m_cubeSize * cubeW / maxLen;   //x
	int height = m_cubeSize * cubeH / maxLen;  //y
	int depth = m_cubeSize * cubeD / maxLen;   //z
	if (width <= 0) width = 1;
	if (height <= 0) height = 1;
	if (depth <= 0) depth = 1;
	printf("width:%d, height:%d, depth:%d\n", width, height, depth);

	vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
	for (int x = 0; x < width; ++x)
	{
		cube_points[x] = new vector<DB_Point_3D> *[height];
		for (int y = 0; y < height; ++y)
		{
			cube_points[x][y] = new vector<DB_Point_3D>[depth];
		}
	}

	vector<DB_Voxels> tmpVoxels;
	
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

				////TEST
				//float inMaxX = -100000, inMaxY = -100000, inMaxZ = -100000;
				//float inMinX = 1000000, inMinY = 1000000, inMinZ = 1000000;
				//for (int i = 0; i < cube_points[x - 1][y - 1][z - 1].size(); ++i)
				//{
				//	if (inMaxX < cube_points[x - 1][y - 1][z - 1][i].x) inMaxX = cube_points[x - 1][y - 1][z - 1][i].x;
				//	if (inMaxY < cube_points[x - 1][y - 1][z - 1][i].y) inMaxY = cube_points[x - 1][y - 1][z - 1][i].y;
				//	if (inMaxZ < cube_points[x - 1][y - 1][z - 1][i].z) inMaxZ = cube_points[x - 1][y - 1][z - 1][i].z;

				//	if (inMinX > cube_points[x - 1][y - 1][z - 1][i].x) inMinX = cube_points[x - 1][y - 1][z - 1][i].x;
				//	if (inMinY > cube_points[x - 1][y - 1][z - 1][i].y) inMinY = cube_points[x - 1][y - 1][z - 1][i].y;
				//	if (inMinZ > cube_points[x - 1][y - 1][z - 1][i].z) inMinZ = cube_points[x - 1][y - 1][z - 1][i].z;
				//}
				////printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", inMaxX, inMaxY, inMaxZ, inMinX, inMinY, inMinZ);

				//DB_Voxels tmp;
				//tmp.planes[0].x[0] = inMaxX; tmp.planes[0].y[0] = inMaxY; tmp.planes[0].z[0] = inMaxZ;
				//tmp.planes[0].x[1] = inMaxX; tmp.planes[0].y[1] = inMinY; tmp.planes[0].z[1] = inMaxZ;
				//tmp.planes[0].x[2] = inMinX; tmp.planes[0].y[2] = inMinY; tmp.planes[0].z[2] = inMaxZ;
				//tmp.planes[0].x[3] = inMinX; tmp.planes[0].y[3] = inMaxY; tmp.planes[0].z[3] = inMaxZ;
				//plane_normVec(tmp.planes[0]);

				//tmp.planes[1].x[0] = inMaxX; tmp.planes[1].y[0] = inMaxY; tmp.planes[1].z[0] = inMinZ;
				//tmp.planes[1].x[1] = inMaxX; tmp.planes[1].y[1] = inMinY; tmp.planes[1].z[1] = inMinZ;
				//tmp.planes[1].x[2] = inMaxX; tmp.planes[1].y[2] = inMinY; tmp.planes[1].z[2] = inMaxZ;
				//tmp.planes[1].x[3] = inMaxX; tmp.planes[1].y[3] = inMaxY; tmp.planes[1].z[3] = inMaxZ;
				//plane_normVec(tmp.planes[1]);

				//tmp.planes[2].x[0] = inMinX; tmp.planes[2].y[0] = inMaxY; tmp.planes[2].z[0] = inMinZ;
				//tmp.planes[2].x[1] = inMinX; tmp.planes[2].y[1] = inMinY; tmp.planes[2].z[1] = inMinZ;
				//tmp.planes[2].x[2] = inMaxX; tmp.planes[2].y[2] = inMinY; tmp.planes[2].z[2] = inMinZ;
				//tmp.planes[2].x[3] = inMaxX; tmp.planes[2].y[3] = inMaxY; tmp.planes[2].z[3] = inMinZ;
				//plane_normVec(tmp.planes[2]);

				//tmp.planes[3].x[0] = inMinX; tmp.planes[3].y[0] = inMaxY; tmp.planes[3].z[0] = inMaxZ;
				//tmp.planes[3].x[1] = inMinX; tmp.planes[3].y[1] = inMinY; tmp.planes[3].z[1] = inMaxZ;
				//tmp.planes[3].x[2] = inMinX; tmp.planes[3].y[2] = inMinY; tmp.planes[3].z[2] = inMinZ;
				//tmp.planes[3].x[3] = inMinX; tmp.planes[3].y[3] = inMaxY; tmp.planes[3].z[3] = inMinZ;
				//plane_normVec(tmp.planes[3]);

				//tmp.planes[4].x[0] = inMaxX; tmp.planes[4].y[0] = inMaxY; tmp.planes[4].z[0] = inMinZ;
				//tmp.planes[4].x[1] = inMaxX; tmp.planes[4].y[1] = inMaxY; tmp.planes[4].z[1] = inMaxZ;
				//tmp.planes[4].x[2] = inMinX; tmp.planes[4].y[2] = inMaxY; tmp.planes[4].z[2] = inMaxZ;
				//tmp.planes[4].x[3] = inMinX; tmp.planes[4].y[3] = inMaxY; tmp.planes[4].z[3] = inMinZ;
				//plane_normVec(tmp.planes[4]);

				//tmp.planes[5].x[0] = inMaxX; tmp.planes[5].y[0] = inMinY; tmp.planes[5].z[0] = inMaxZ;
				//tmp.planes[5].x[1] = inMaxX; tmp.planes[5].y[1] = inMinY; tmp.planes[5].z[1] = inMinZ;
				//tmp.planes[5].x[2] = inMinX; tmp.planes[5].y[2] = inMinY; tmp.planes[5].z[2] = inMinZ;
				//tmp.planes[5].x[3] = inMinX; tmp.planes[5].y[3] = inMinY; tmp.planes[5].z[3] = inMaxZ;
				//plane_normVec(tmp.planes[5]);
				//tmpVoxels.push_back(tmp);

				//하나의 복셀에 해당하는 육면체 정보..
				DB_Voxels tmp;
				tmp.planes[0].x[0] = maxRangeX; tmp.planes[0].y[0] = maxRangeY; tmp.planes[0].z[0] = maxRangeZ; 
				tmp.planes[0].x[1] = maxRangeX; tmp.planes[0].y[1] = minRangeY; tmp.planes[0].z[1] = maxRangeZ;
				tmp.planes[0].x[2] = minRangeX; tmp.planes[0].y[2] = minRangeY; tmp.planes[0].z[2] = maxRangeZ;
				tmp.planes[0].x[3] = minRangeX; tmp.planes[0].y[3] = maxRangeY; tmp.planes[0].z[3] = maxRangeZ;
				plane_normVec(tmp.planes[0]);

				tmp.planes[1].x[0] = maxRangeX; tmp.planes[1].y[0] = maxRangeY; tmp.planes[1].z[0] = minRangeZ;
				tmp.planes[1].x[1] = maxRangeX; tmp.planes[1].y[1] = minRangeY; tmp.planes[1].z[1] = minRangeZ;
				tmp.planes[1].x[2] = maxRangeX; tmp.planes[1].y[2] = minRangeY; tmp.planes[1].z[2] = maxRangeZ;
				tmp.planes[1].x[3] = maxRangeX; tmp.planes[1].y[3] = maxRangeY; tmp.planes[1].z[3] = maxRangeZ;
				plane_normVec(tmp.planes[1]);

				tmp.planes[2].x[0] = minRangeX; tmp.planes[2].y[0] = maxRangeY; tmp.planes[2].z[0] = minRangeZ;
				tmp.planes[2].x[1] = minRangeX; tmp.planes[2].y[1] = minRangeY; tmp.planes[2].z[1] = minRangeZ;
				tmp.planes[2].x[2] = maxRangeX; tmp.planes[2].y[2] = minRangeY; tmp.planes[2].z[2] = minRangeZ;
				tmp.planes[2].x[3] = maxRangeX; tmp.planes[2].y[3] = maxRangeY; tmp.planes[2].z[3] = minRangeZ;
				plane_normVec(tmp.planes[2]);

				tmp.planes[3].x[0] = minRangeX; tmp.planes[3].y[0] = maxRangeY; tmp.planes[3].z[0] = maxRangeZ;
				tmp.planes[3].x[1] = minRangeX; tmp.planes[3].y[1] = minRangeY; tmp.planes[3].z[1] = maxRangeZ;
				tmp.planes[3].x[2] = minRangeX; tmp.planes[3].y[2] = minRangeY; tmp.planes[3].z[2] = minRangeZ;
				tmp.planes[3].x[3] = minRangeX; tmp.planes[3].y[3] = maxRangeY; tmp.planes[3].z[3] = minRangeZ;
				plane_normVec(tmp.planes[3]);

				tmp.planes[4].x[0] = maxRangeX; tmp.planes[4].y[0] = maxRangeY; tmp.planes[4].z[0] = minRangeZ;
				tmp.planes[4].x[1] = maxRangeX; tmp.planes[4].y[1] = maxRangeY; tmp.planes[4].z[1] = maxRangeZ;
				tmp.planes[4].x[2] = minRangeX; tmp.planes[4].y[2] = maxRangeY; tmp.planes[4].z[2] = maxRangeZ;
				tmp.planes[4].x[3] = minRangeX; tmp.planes[4].y[3] = maxRangeY; tmp.planes[4].z[3] = minRangeZ;
				plane_normVec(tmp.planes[4]);

				tmp.planes[5].x[0] = maxRangeX; tmp.planes[5].y[0] = minRangeY; tmp.planes[5].z[0] = maxRangeZ;
				tmp.planes[5].x[1] = maxRangeX; tmp.planes[5].y[1] = minRangeY; tmp.planes[5].z[1] = minRangeZ;
				tmp.planes[5].x[2] = minRangeX; tmp.planes[5].y[2] = minRangeY; tmp.planes[5].z[2] = minRangeZ;
				tmp.planes[5].x[3] = minRangeX; tmp.planes[5].y[3] = minRangeY; tmp.planes[5].z[3] = maxRangeZ;
				plane_normVec(tmp.planes[5]);
				tmpVoxels.push_back(tmp);
			}
		}
	}


	//int threshold = refinedPoints.size() * 0.001;
	printf("--- refinedPoints:%d\n\n", refinedPoints.size());
	int threshold = 5;  //20, 10
	printf("threshold:%d\n", threshold);
	nCurrCnt = 0;
	refinedPoints.clear();
	ptr_cloud->clear();
	ptr_cloud->points.clear();

	int voxelCnt = 0;
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

					tmpVoxels[voxelCnt].clusterID = nCurrCnt;
					voxels.push_back(tmpVoxels[voxelCnt]);

				}
				++voxelCnt;

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


	if (debug)
	{

		for (int x = 1; x <= width + 1; ++x)
		{
			float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			for (int y = 1; y <= height + 1; ++y)
			{
				float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
				float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
				for (int z = 1; z <= depth + 1; ++z)
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

	if (bShow)
	{
		//결과 보기
		pcl::visualization::CloudViewer viewer2("Cloud Viewer");
		viewer2.showCloud(ptr_cloud);
		while (!viewer2.wasStopped())
		{
		}
	}




	points.clear();
	points.resize(refinedPoints.size());
	copy(refinedPoints.begin(), refinedPoints.end(), points.begin());

	nLabel = nCurrCnt;

	printf("finish!");

}


void VOXEL_DB::voxelFitting_octree(vector<DB_Point_3D>& points, int num_points, int color[][3], int &nLabel)
{

	int nPrevCnt = 0; 
	int nCurrCnt = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<DB_Point_3D> refinedPoints;

	refinedPoints.clear();
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());

	bool bShow = 1;
	int debug = 0;
	int _cubeSize = 15;    // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10

	int nAllCnt = 0;
	while (1)
	{
		float maxX = -100000, maxY = -100000, maxZ = -100000;
		float minX = 100000, minY = 100000, minZ = 100000;
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
		float maxLen = max(max(cubeW, cubeH), cubeD);

		ptr_cloud->clear();
		ptr_cloud->points.clear();

		//vector<DB_Point_3D> cube_points[10][10][10];
		int width = _cubeSize * cubeW / maxLen;   //x
		int height = _cubeSize * cubeH / maxLen;  //y
		int depth = _cubeSize * cubeD / maxLen;   //z
		if (width <= 0) width = 1;
		if (height <= 0) height = 1;
		if (depth <= 0) depth = 1;
		printf("width:%d, height:%d, depth:%d\n", width, height, depth);

		vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
		for (int x = 0; x < width; ++x)
		{
			cube_points[x] = new vector<DB_Point_3D> *[height];
			for (int y = 0; y < height; ++y)
			{
				cube_points[x][y] = new vector<DB_Point_3D>[depth];
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

		int threshold = refinedPoints.size() * 0.0001;   // Aachen = 0.0001;
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


		if (debug)
		{

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

		if (nAllCnt > 1) break;   // CMU, Aachen => 2
		++nAllCnt;

	}  //while


	//printf("\n -------------------------------------------------------\n");


	ptr_cloud->clear();
	ptr_cloud->points.clear();
	for (int i = 0; i < refinedPoints.size(); ++i)
	{
		// Copy point to noew cloud
		pcl::PointXYZRGB pt_color;
		pt_color.x = refinedPoints[i].x;
		pt_color.y = refinedPoints[i].y;
		pt_color.z = refinedPoints[i].z;
		pt_color.r = 255;
		pt_color.g = 255;
		pt_color.b = 255;

		ptr_cloud->points.push_back(pt_color);
	}


	float resolution = 2.0f;    //복셀 크기 설정(Set octree voxel resolution) 2.0f == 200cm
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution); 
	octree.setInputCloud(ptr_cloud);    //입력
	//octree.setTreeDepth(5);
	octree.enableDynamicDepth(100);
	octree.addPointsFromInputCloud();   //Octree 생성 (Build Octree)

	//pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::Iterator b_it;
	pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ>::LeafNodeIterator it;
	const pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ>::LeafNodeIterator it_end = octree.leaf_end();
	std::vector<std::vector<int>> indices_vec;
	indices_vec.reserve(octree.getLeafCount());
	int nCnt = 0;
	for (auto it = octree.leaf_begin(); it != it_end; ++it)
	{
		// add points from leaf node to indexVector
		std::vector<int> indexVector;
		auto leaf = it.getLeafContainer();
		std::vector<int> indices;
		
		leaf.getPointIndices(indices);
		indices_vec.push_back(indices);
		printf("%d\n", it.getCurrentOctreeDepth());

		++nCnt;
		int aiwifsh = 0;
	}
	std::cout << "nCnt:" << nCnt <<  endl;

	//pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ>::DepthFirstIterator it2;
	//const pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ>::DepthFirstIterator it_end = octree.depth_end();
	//it2.




	nCurrCnt = 0;
	//refinedPoints.clear();
	vector<DB_Point_3D> refinedPoints2;

	for (auto i = 0; i < indices_vec.size(); ++i) 
	{
		if (indices_vec[i].size() >= 20)
			++nCurrCnt;
		else
			continue;
		for (auto j = 0; j < indices_vec[i].size(); ++j)
		{
			DB_Point_3D tmp;
			tmp = refinedPoints[indices_vec[i][j]];
			tmp.clusterID = nCurrCnt;
			refinedPoints2.push_back(tmp);

			ptr_cloud->points[indices_vec[i][j]].r = color[nCurrCnt][0];
			ptr_cloud->points[indices_vec[i][j]].g = color[nCurrCnt][1];
			ptr_cloud->points[indices_vec[i][j]].b = color[nCurrCnt][2];
		}
	}
	std::cout << "nCurrCnt:" << nCurrCnt;

	//pcl::PointXYZRGB searchPoint1 = ptr_cloud->points[6500]; //기준점(searchPoint) 설정 방법 #2(6500번째 포인트)
	//std::vector<int> pointIdxVec;    //결과물 포인트의 Index 저장(Save the result vector of the voxel neighbor search) 
	//if (octree.voxelSearch(searchPoint1, pointIdxVec))  //기준점과 동일한 복셀내 존재 하는 하는 포인트 탐색(Voxel Neighbor Search)
	//{
	//	std::cout << "Neighbors within voxel search at (" << searchPoint1.x
	//		<< " " << searchPoint1.y
	//		<< " " << searchPoint1.z << ")"
	//		<< std::endl;

	//	for (size_t i = 0; i < pointIdxVec.size(); ++i) {
	//		ptr_cloud->points[pointIdxVec[i]].r = 255;
	//		ptr_cloud->points[pointIdxVec[i]].g = 0;
	//		ptr_cloud->points[pointIdxVec[i]].b = 0;
	//	}
	//}
	// 생성된 포인트클라우드 저장 
	//pcl::io::savePCDFile<pcl::PointXYZRGB>("Octree_AllinOne.pcd", *ptr_cloud);

	if (bShow)
	{
		//결과 보기
		pcl::visualization::CloudViewer viewer2("Cloud Viewer");
		viewer2.showCloud(ptr_cloud);
		while (!viewer2.wasStopped())
		{
		}
	}


	//for (int x = 0; x < width; x++)
	//{
	//	for (int y = 0; y < height; y++)
	//	{
	//		delete[] cube_points[x][y];
	//	}
	//}
	//for (int x = 0; x < width; x++)
	//{
	//	delete[] cube_points[x];
	//}
	//delete[] cube_points;


	points.clear();
	points.resize(refinedPoints.size());
	//copy(refinedPoints.begin(), refinedPoints.end(), points.begin());
	copy(refinedPoints2.begin(), refinedPoints2.end(), points.begin());

	nLabel = nCurrCnt;

	printf("finish!");

}



void VOXEL_DB::voxelFitting_aachen(vector<DB_Point_3D>& points, vector<DB_Voxels>& voxels, int num_points, int color[][3], int &nLabel)
{
	int nPrevCnt = 0;
	int nCurrCnt = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<DB_Point_3D> refinedPoints;

	refinedPoints.clear();
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());

	bool bShow = 0;
	int debug = 0;
	int _cubeSize = 15;    // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10

	int nAllCnt = 0;
	while (1)
	{
		float maxX = -100000, maxY = -100000, maxZ = -100000;
		float minX = 100000, minY = 100000, minZ = 100000;
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
		float maxLen = max(max(cubeW, cubeH), cubeD);

		ptr_cloud->clear();
		ptr_cloud->points.clear();

		//vector<DB_Point_3D> cube_points[10][10][10];
		int width = _cubeSize * cubeW / maxLen;   //x
		int height = _cubeSize * cubeH / maxLen;  //y
		int depth = _cubeSize * cubeD / maxLen;   //z
		if (width <= 0) width = 1;
		if (height <= 0) height = 1;
		if (depth <= 0) depth = 1;
		printf("width:%d, height:%d, depth:%d\n", width, height, depth);

		vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
		for (int x = 0; x < width; ++x)
		{
			cube_points[x] = new vector<DB_Point_3D> *[height];
			for (int y = 0; y < height; ++y)
			{
				cube_points[x][y] = new vector<DB_Point_3D>[depth];
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

		int threshold = refinedPoints.size() * 0.01;   // Aachen = 0.0001;
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


		if (debug)
		{

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

		if (nAllCnt > 1) break;   // CMU, Aachen => 2
		++nAllCnt;

	}  //while


	if (bShow) {
		//결과 보기
		pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer2");
		viewer1.showCloud(ptr_cloud);
		while (!viewer1.wasStopped())
		{
		}
	}



	printf("\n -------------------------------------------------------\n");

	float maxX = -100000, maxY = -100000, maxZ = -100000;
	float minX = 1000000, minY = 1000000, minZ = 1000000;
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
	float maxLen = max(max(cubeW, cubeH), cubeD);

	ptr_cloud->clear();
	ptr_cloud->points.clear();

	int width = m_cubeSize * cubeW / maxLen;   //x
	int height = m_cubeSize * cubeH / maxLen;  //y
	int depth = m_cubeSize * cubeD / maxLen;   //z
	if (width <= 0) width = 1;
	if (height <= 0) height = 1;
	if (depth <= 0) depth = 1;
	printf("width:%d, height:%d, depth:%d\n", width, height, depth);

	vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
	for (int x = 0; x < width; ++x)
	{
		cube_points[x] = new vector<DB_Point_3D> *[height];
		for (int y = 0; y < height; ++y)
		{
			cube_points[x][y] = new vector<DB_Point_3D>[depth];
		}
	}

	vector<DB_Voxels> tmpVoxels;

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


				//하나의 복셀에 해당하는 육면체 정보..
				DB_Voxels tmp;
				tmp.planes[0].x[0] = maxRangeX; tmp.planes[0].y[0] = maxRangeY; tmp.planes[0].z[0] = maxRangeZ;
				tmp.planes[0].x[1] = maxRangeX; tmp.planes[0].y[1] = minRangeY; tmp.planes[0].z[1] = maxRangeZ;
				tmp.planes[0].x[2] = minRangeX; tmp.planes[0].y[2] = minRangeY; tmp.planes[0].z[2] = maxRangeZ;
				tmp.planes[0].x[3] = minRangeX; tmp.planes[0].y[3] = maxRangeY; tmp.planes[0].z[3] = maxRangeZ;
				plane_normVec(tmp.planes[0]);

				tmp.planes[1].x[0] = maxRangeX; tmp.planes[1].y[0] = maxRangeY; tmp.planes[1].z[0] = minRangeZ;
				tmp.planes[1].x[1] = maxRangeX; tmp.planes[1].y[1] = minRangeY; tmp.planes[1].z[1] = minRangeZ;
				tmp.planes[1].x[2] = maxRangeX; tmp.planes[1].y[2] = minRangeY; tmp.planes[1].z[2] = maxRangeZ;
				tmp.planes[1].x[3] = maxRangeX; tmp.planes[1].y[3] = maxRangeY; tmp.planes[1].z[3] = maxRangeZ;
				plane_normVec(tmp.planes[1]);

				tmp.planes[2].x[0] = minRangeX; tmp.planes[2].y[0] = maxRangeY; tmp.planes[2].z[0] = minRangeZ;
				tmp.planes[2].x[1] = minRangeX; tmp.planes[2].y[1] = minRangeY; tmp.planes[2].z[1] = minRangeZ;
				tmp.planes[2].x[2] = maxRangeX; tmp.planes[2].y[2] = minRangeY; tmp.planes[2].z[2] = minRangeZ;
				tmp.planes[2].x[3] = maxRangeX; tmp.planes[2].y[3] = maxRangeY; tmp.planes[2].z[3] = minRangeZ;
				plane_normVec(tmp.planes[2]);

				tmp.planes[3].x[0] = minRangeX; tmp.planes[3].y[0] = maxRangeY; tmp.planes[3].z[0] = maxRangeZ;
				tmp.planes[3].x[1] = minRangeX; tmp.planes[3].y[1] = minRangeY; tmp.planes[3].z[1] = maxRangeZ;
				tmp.planes[3].x[2] = minRangeX; tmp.planes[3].y[2] = minRangeY; tmp.planes[3].z[2] = minRangeZ;
				tmp.planes[3].x[3] = minRangeX; tmp.planes[3].y[3] = maxRangeY; tmp.planes[3].z[3] = minRangeZ;
				plane_normVec(tmp.planes[3]);

				tmp.planes[4].x[0] = maxRangeX; tmp.planes[4].y[0] = maxRangeY; tmp.planes[4].z[0] = minRangeZ;
				tmp.planes[4].x[1] = maxRangeX; tmp.planes[4].y[1] = maxRangeY; tmp.planes[4].z[1] = maxRangeZ;
				tmp.planes[4].x[2] = minRangeX; tmp.planes[4].y[2] = maxRangeY; tmp.planes[4].z[2] = maxRangeZ;
				tmp.planes[4].x[3] = minRangeX; tmp.planes[4].y[3] = maxRangeY; tmp.planes[4].z[3] = minRangeZ;
				plane_normVec(tmp.planes[4]);

				tmp.planes[5].x[0] = maxRangeX; tmp.planes[5].y[0] = minRangeY; tmp.planes[5].z[0] = maxRangeZ;
				tmp.planes[5].x[1] = maxRangeX; tmp.planes[5].y[1] = minRangeY; tmp.planes[5].z[1] = minRangeZ;
				tmp.planes[5].x[2] = minRangeX; tmp.planes[5].y[2] = minRangeY; tmp.planes[5].z[2] = minRangeZ;
				tmp.planes[5].x[3] = minRangeX; tmp.planes[5].y[3] = minRangeY; tmp.planes[5].z[3] = maxRangeZ;
				plane_normVec(tmp.planes[5]);
				tmpVoxels.push_back(tmp);
			}
		}
	}


	//int threshold = refinedPoints.size() * 0.001;
	printf("--- refinedPoints:%d\n\n", refinedPoints.size());
	int threshold = 20;  //20
	printf("threshold:%d\n", threshold);
	nCurrCnt = 0;
	refinedPoints.clear();
	ptr_cloud->clear();
	ptr_cloud->points.clear();

	int voxelCnt = 0;
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

					tmpVoxels[voxelCnt].clusterID = nCurrCnt;
					voxels.push_back(tmpVoxels[voxelCnt]);

				}
				++voxelCnt;

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


	if (debug)
	{

		for (int x = 1; x <= width + 1; ++x)
		{
			float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			for (int y = 1; y <= height + 1; ++y)
			{
				float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
				float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
				for (int z = 1; z <= depth + 1; ++z)
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

	if (bShow)
	{
		//결과 보기
		pcl::visualization::CloudViewer viewer2("Cloud Viewer");
		viewer2.showCloud(ptr_cloud);
		while (!viewer2.wasStopped())
		{
		}
	}




	points.clear();
	points.resize(refinedPoints.size());
	copy(refinedPoints.begin(), refinedPoints.end(), points.begin());

	nLabel = nCurrCnt;

	printf("finish!");

}