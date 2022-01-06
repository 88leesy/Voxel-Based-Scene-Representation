#include "H_voxel_DB.h"


void H_VOXEL_DB::printResults(vector<H_Voxels>& points, int num_points, bool bAllpoints, int color[][3])
{
	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


	if (bAllpoints)
	{
		//cloud.is_dense = false;
		//cloud.points.resize(points.size());
		int color_lb = 0;
		for (int i = 0; i < points.size(); ++i)
		{
			for (int pt_id = 0; pt_id < points[i].vec_pt_3d_db.size(); ++pt_id)
			{
				color_lb = points[i].vec_pt_3d_db[pt_id].clusterID - 1;
				//color_lb = i;

				pcl::PointXYZRGB pt_color;
				pt_color.x = points[i].vec_pt_3d_db[pt_id].x;
				pt_color.y = points[i].vec_pt_3d_db[pt_id].y;
				pt_color.z = points[i].vec_pt_3d_db[pt_id].z;

				pt_color.r = color[color_lb][0];
				pt_color.g = color[color_lb][1];
				pt_color.b = color[color_lb][2];

				ptr_cloud->points.push_back(pt_color);
			}

		}

		//*ptr_cloud = cloud;

		pcl::visualization::CloudViewer viewer("plane fitting Cloud Viewer");
		viewer.showCloud(ptr_cloud);
		while (!viewer.wasStopped())
		{
		}
	}
	else
	{
		vector<H_Voxels> tmp_points;
		for (int i = 0; i < num_points; ++i)
		{
			for (int pt_id = 0; pt_id < points[i].vec_pt_3d_db.size(); ++pt_id)
			{
				if (points[i].vec_pt_3d_db[pt_id].clusterID != -1)
				{
					tmp_points.push_back(points[i]);
				}
			}
			
		}

		// 포인트클라우드의 파라미터 설정 : width, height, is_dense
		//cloud.width = 500;
		//cloud.height = 500;
		//cloud.is_dense = false;
		//cloud.points.resize(tmp_points.size());

		for (int i = 0; i < tmp_points.size(); ++i)
		{
			for (int pt_id = 0; pt_id < tmp_points[i].vec_pt_3d_db.size(); ++pt_id)
			{
				int color_lb = tmp_points[i].vec_pt_3d_db[pt_id].clusterID - 1;
				pcl::PointXYZRGB pt_color;
				pt_color.x = tmp_points[i].vec_pt_3d_db[pt_id].x;
				pt_color.y = tmp_points[i].vec_pt_3d_db[pt_id].y;
				pt_color.z = tmp_points[i].vec_pt_3d_db[pt_id].z;

				pt_color.r = color[color_lb][0];
				pt_color.g = color[color_lb][1];
				pt_color.b = color[color_lb][2];

				ptr_cloud->points.push_back(pt_color);
			}

		}

		//*ptr_cloud = cloud;

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


void H_VOXEL_DB::H_planeFitResults(vector<H_Voxels>& points, int num_points, bool bAllpoints, int color[][3])
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (bAllpoints)
	{
		int nLb = 0;

		for (int i = 0; i < points.size(); ++i)
		{
			for (int j = 0; j < points[i].vec_H_voxels.size(); ++j)
			{
				//if (points[i].vec_H_voxels[j].vec_pt_3d_db.size() > 3)
				{
					printf("lb:%d, points[i].vec_pt_3d_db size : %d\n", nLb, points[i].vec_H_voxels[j].vec_pt_3d_db.size());
					for (int pt_id = 0; pt_id < points[i].vec_H_voxels[j].vec_pt_3d_db.size(); ++pt_id)
					{
						//int color_lb = points[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].clusterID - 1;
						int color_lb = nLb;
						pcl::PointXYZRGB pt_color;
						pt_color.x = points[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].x;
						pt_color.y = points[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].y;
						pt_color.z = points[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].z;

						pt_color.r = color[color_lb][0];
						pt_color.g = color[color_lb][1];
						pt_color.b = color[color_lb][2];

						ptr_cloud->points.push_back(pt_color);
					}
				}

				++nLb;  //라벨을 체크
			}
		}

		//*ptr_cloud = cloud;

		pcl::visualization::CloudViewer viewer("plane fitting Cloud Viewer");
		viewer.showCloud(ptr_cloud);
		while (!viewer.wasStopped())
		{
		}
	}
	else
	{
		vector<H_Voxels> tmp_points;
		for (int i = 0; i < num_points; ++i)
		{
			for (int pt_id = 0; pt_id < points[i].vec_pt_3d_db.size(); ++pt_id)
			{
				if (points[i].vec_pt_3d_db[pt_id].clusterID != -1)
				{
					tmp_points.push_back(points[i]);
				}
			}

		}

		for (int i = 0; i < tmp_points.size(); ++i)
		{
			for (int pt_id = 0; pt_id < tmp_points[i].vec_pt_3d_db.size(); ++pt_id)
			{
				int color_lb = tmp_points[i].vec_pt_3d_db[pt_id].clusterID - 1;
				pcl::PointXYZRGB pt_color;
				pt_color.x = tmp_points[i].vec_pt_3d_db[pt_id].x;
				pt_color.y = tmp_points[i].vec_pt_3d_db[pt_id].y;
				pt_color.z = tmp_points[i].vec_pt_3d_db[pt_id].z;

				pt_color.r = color[color_lb][0];
				pt_color.g = color[color_lb][1];
				pt_color.b = color[color_lb][2];

				ptr_cloud->points.push_back(pt_color);
			}

		}

		//*ptr_cloud = cloud;

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

void H_VOXEL_DB::H_planeFitResults2(vector<H_Voxels>& points, int num_points, bool bAllpoints, int color[][3])
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (bAllpoints)
	{
		int nLb = 0;

		for (int i = 0; i < points.size(); ++i)
		{
			for (int j = 0; j < points[i].vec_H_voxels.size(); ++j)
			{
				for(int k = 0; k < points[i].vec_H_voxels[j].vec_H_voxels.size(); ++k)
				{
					H_Voxels tmpVoxels = points[i].vec_H_voxels[j].vec_H_voxels[k];
					printf("lb:%d, points[i].vec_pt_3d_db size : %d\n", nLb, tmpVoxels.vec_pt_3d_db.size());

					//if ((2000 < nLb && nLb < 4000) )
					{
						for (int pt_id = 0; pt_id < tmpVoxels.vec_pt_3d_db.size(); ++pt_id)
						{
							//int color_lb = points[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].clusterID - 1;
							int color_lb = nLb;
							pcl::PointXYZRGB pt_color;
							pt_color.x = tmpVoxels.vec_pt_3d_db[pt_id].x;
							pt_color.y = tmpVoxels.vec_pt_3d_db[pt_id].y;
							pt_color.z = tmpVoxels.vec_pt_3d_db[pt_id].z;

							pt_color.r = color[color_lb][0];
							pt_color.g = color[color_lb][1];
							pt_color.b = color[color_lb][2];

							ptr_cloud->points.push_back(pt_color);
						}

					}

					++nLb;  //라벨을 체크
				}
			}
		}

		//*ptr_cloud = cloud;

		pcl::visualization::CloudViewer viewer("plane fitting Cloud Viewer");
		viewer.showCloud(ptr_cloud);
		while (!viewer.wasStopped())
		{
		}
	}
}


void H_VOXEL_DB::H_planeFitResults3(vector<H_Voxels>& points, int num_points, bool bAllpoints, int color[][3])
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (bAllpoints)
	{
		int nLb = 0;

		for (int i = 0; i < points.size(); ++i)
		{
			for (int j = 0; j < points[i].vec_H_voxels.size(); ++j)
			{
				//if (points[i].vec_H_voxels[j].vec_pt_3d_db.size() > 3)
				{
					printf("lb:%d, points[i].vec_pt_3d_db size : %d\n", nLb, points[i].vec_H_voxels[j].vec_pt_3d_db.size());
					for (int pt_id = 0; pt_id < points[i].vec_H_voxels[j].vec_pt_3d_db.size(); ++pt_id)
					{
						int color_lb = nLb;
						pcl::PointXYZRGB pt_color;
						pt_color.x = points[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].x;
						pt_color.y = points[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].y;
						pt_color.z = points[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].z;

						pt_color.r = color[color_lb][0];
						pt_color.g = color[color_lb][1];
						pt_color.b = color[color_lb][2];

						ptr_cloud->points.push_back(pt_color);
					}
				}

				++nLb;  //라벨을 체크
			}
		}

		//*ptr_cloud = cloud;

		pcl::visualization::CloudViewer viewer("plane fitting Cloud Viewer");
		viewer.showCloud(ptr_cloud);
		while (!viewer.wasStopped())
		{
		}
	}
}

float H_VOXEL_DB::PlaneFitting(vector<H_Voxels>& points, int num_points, vector<H_Voxels>& inlier_points, int color[][3])
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


	vector<H_Voxels> tmp_points;
	for (int i = 0; i < num_points; ++i)
	{
		for (int pt_id = 0; pt_id < points[i].vec_pt_3d_db.size(); ++pt_id)
		{
			if (points[i].vec_pt_3d_db[pt_id].clusterID != -1)
			{
				tmp_points.push_back(points[i]);
			}
		}
		
	}
	for (int i = 0; i < tmp_points.size(); ++i)
	{
		for (int pt_id = 0; pt_id < tmp_points[i].vec_pt_3d_db.size(); ++pt_id)
		{
			cloud.points[i].x = tmp_points[i].vec_pt_3d_db[pt_id].x;
			cloud.points[i].y = tmp_points[i].vec_pt_3d_db[pt_id].y;
			cloud.points[i].z = tmp_points[i].vec_pt_3d_db[pt_id].z;
		}

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
	seg.setDistanceThreshold(0.2);  //0.1


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


float H_VOXEL_DB::PlaneFitting_ver2(vector<H_Voxels>& points, int num_points, vector<H_Voxels>& inlier_points, int color[][3], float coef[4])
{

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 포인트클라우드의 파라미터 설정 : width, height, is_dense
	//cloud.width = 500;
	//cloud.height = 500;
	cloud.is_dense = true;
	cloud.points.resize(points.size());


	vector<H_Voxels> tmp_points;
	//for (int i = 0; i < num_points; ++i)
	//{
	//	for (int pt_id = 0; pt_id < points[i].vec_pt_3d_db.size(); ++pt_id)
	//	{
	//		if (points[i].vec_pt_3d_db[pt_id].clusterID != -1)
	//		{
	//			tmp_points.push_back(points[i]);
	//		}
	//	}
	//}
	std::copy(points.begin(), points.end(), tmp_points.begin());
	for (int i = 0; i < tmp_points.size(); ++i)
	{
		for (int pt_id = 0; pt_id < tmp_points[i].vec_pt_3d_db.size(); ++pt_id)
		{
			cloud.points[i].x = tmp_points[i].vec_pt_3d_db[pt_id].x;
			cloud.points[i].y = tmp_points[i].vec_pt_3d_db[pt_id].y;
			cloud.points[i].z = tmp_points[i].vec_pt_3d_db[pt_id].z;
		}
		
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
	H_Voxels minDistPt;
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


float H_VOXEL_DB::PlaneFitting_Hierarchy(vector<DB_Point_3D>& points, int num_points, vector<DB_Point_3D>& inlier_points, 
	float fdistThres, int color[][3], float coef[4])
{

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 포인트클라우드의 파라미터 설정 : width, height, is_dense
	//cloud.width = 500;
	//cloud.height = 500;
	cloud.is_dense = true;
	cloud.points.resize(points.size());


	vector<DB_Point_3D> tmp_points;
	//tmp_points.clear();
	//tmp_points.assign(points.begin(), points.end()); // 전체 복사
	tmp_points.clear();
	tmp_points.resize(points.size());
	copy(points.begin(), points.end(), tmp_points.begin());

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
	seg.setDistanceThreshold(fdistThres);  //0.1


	// Create pointcloud to publish inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
	int original_size(ptr_cloud->size());
	int n_planes(0);
	int maxIterCntPlane = 1;
	float meanErr = 0;
	int minErrIdx = 0;
	DB_Point_3D minDistPt;
	while ((ptr_cloud->size() > original_size*0.3) && (n_planes < maxIterCntPlane)) {

		seg.setInputCloud(ptr_cloud);
		seg.segment(*inliers, *coefficients);

		// Check result
		if (inliers->indices.size() <= 3)  //inlier개수가 3개 이하이면, 제낌... => 모델이 제대로 만들어 질 수 없데...
			break;

		// Iterate inliers
		double mean_error(0);
		double max_error(0);
		double min_error(1000000);
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



float H_VOXEL_DB::PlaneFitting_Hierarchy_ver2(vector<DB_Point_3D>& points, int num_points, vector<vector<DB_Point_3D>>& inlier_points,
	float fdistThres, int color[][3], vector<vector<float>>& coef)
{

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 포인트클라우드의 파라미터 설정 : width, height, is_dense
	//cloud.width = 500;
	//cloud.height = 500;
	cloud.is_dense = true;
	cloud.points.resize(points.size());


	vector<DB_Point_3D> tmp_points;
	//tmp_points.clear();
	//tmp_points.assign(points.begin(), points.end()); // 전체 복사
	tmp_points.clear();
	tmp_points.resize(points.size());
	copy(points.begin(), points.end(), tmp_points.begin());

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
	/*
	SAC_RANSAC - RANdom SAmple Consensus
	SAC_LMEDS - Least Median of Squares
	SAC_MSAC - M-Estimator SAmple Consensus
	SAC_RRANSAC - Randomized RANSAC
	SAC_RMSAC - Randomized MSAC
	SAC_MLESAC - Maximum LikeLihood Estimation SAmple Consensus
	SAC_PROSAC - PROgressive SAmple Consensus
	*/
	seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용  , SAC_RANSAC, SAC_PROSAC, SAC_MLESAC, SAC_MSAC
	//seg.setDistanceThreshold(0.08); //determines how close a point must be to the model in order to be considered an inlier
	seg.setDistanceThreshold(fdistThres);  //0.1
	//seg.setMaxIterations(1000);               //최대 실행 수


	// Create pointcloud to publish inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
	int original_size = ptr_cloud->size();
	int n_planes(0);
	int maxIterCntPlane = 3;
	float meanErr = 0;
	int minErrIdx = 0;
	DB_Point_3D minDistPt;
	                            //&& ptr_cloud->size()>3
	while (( (ptr_cloud->size() > (original_size*0.2)) && (ptr_cloud->size() > 3)) && (n_planes < maxIterCntPlane)) {

		seg.setInputCloud(ptr_cloud);
		seg.segment(*inliers, *coefficients);

		printf("original_size:%.2f, ptr_cloud->size():%d, inliersSize:%d\n", original_size*0.2, ptr_cloud->size(), inliers->indices.size());

		// Check result
		if (inliers->indices.size() <= 3)  //inlier개수가 3개 이하이면, 제낌... => 모델이 제대로 만들어 질 수 없데...
			break;

		// Iterate inliers
		double mean_error(0);
		double max_error(0);
		double min_error(1000000);
		std::vector<double> err;

		//inlier_points.clear();
		inlier_points.push_back(vector<DB_Point_3D>());
		coef.push_back(vector<float>());

		for (int i = 0; i < inliers->indices.size(); i++) {

			// Get Point
			pcl::PointXYZ pt = ptr_cloud->points[inliers->indices[i]];

			// Compute distance
			double d = point2planedistnace(pt, coefficients); //* 1000;// mm
			err.push_back(d);

			// Update statistics
			mean_error += d;
			if (d > max_error) max_error = d;
			if (d < min_error)
			{
				min_error = d;
				minErrIdx = i;
			}

			inlier_points[n_planes].push_back(tmp_points[inliers->indices[i]]);
		}
		mean_error /= inliers->indices.size();
		meanErr = mean_error;
		printf("planeToPoints meandist:%d\n", meanErr);

		minDistPt = tmp_points[inliers->indices[minErrIdx]];

		tmp_points.clear();
		tmp_points.resize(inlier_points[n_planes].size());
		copy(inlier_points[n_planes].begin(), inlier_points[n_planes].end(), tmp_points.begin());

		coef[n_planes].push_back(coefficients->values[0]);
		coef[n_planes].push_back(coefficients->values[1]);
		coef[n_planes].push_back(coefficients->values[2]);
		coef[n_planes].push_back(coefficients->values[3]);


		//double sigma(0);
		//for (int i = 0; i < inliers->indices.size(); i++) {
		//	sigma += pow(err[i] - mean_error, 2);
		//	// Get Point
		//	pcl::PointXYZ pt = ptr_cloud->points[inliers->indices[i]];
		//	// Copy point to noew cloud
		//	pcl::PointXYZRGB pt_color;
		//	pt_color.x = pt.x;
		//	pt_color.y = pt.y;
		//	pt_color.z = pt.z;
		//	pt_color.r = color[n_planes][0];
		//	pt_color.g = color[n_planes][1];
		//	pt_color.b = color[n_planes][2];
		//	cloud_pub->points.push_back(pt_color);
		//}
		//sigma = sqrt(sigma / inliers->indices.size());


		// Extract inliers
		extract.setInputCloud(ptr_cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);              //true면, filter를 거치고 난 후 inlier의 나머지를 반환해줌.
		pcl::PointCloud<pcl::PointXYZ> cloudF;
		extract.filter(cloudF);
		ptr_cloud->swap(cloudF);

		// Nest iteration
		n_planes++;

	}

	
	return meanErr;

}


//PlaneFitting_Hierarchy_ver3 => 틀림... 잘못구함.. 다시해야함..
float H_VOXEL_DB::PlaneFitting_Hierarchy_ver3(vector<DB_Point_3D>& points, int num_points, vector<vector<DB_Point_3D>>& inlier_points,
	float fdistThres, int color[][3], vector<vector<float>>& coef)
{

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 포인트클라우드의 파라미터 설정 : width, height, is_dense
	//cloud.width = 500;
	//cloud.height = 500;
	cloud.is_dense = true;
	cloud.points.resize(points.size());


	vector<DB_Point_3D> tmp_points;
	//tmp_points.clear();
	//tmp_points.assign(points.begin(), points.end()); // 전체 복사
	tmp_points.clear();
	tmp_points.resize(points.size());
	copy(points.begin(), points.end(), tmp_points.begin());

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
	/*
	SAC_RANSAC - RANdom SAmple Consensus
	SAC_LMEDS - Least Median of Squares
	SAC_MSAC - M-Estimator SAmple Consensus
	SAC_RRANSAC - Randomized RANSAC
	SAC_RMSAC - Randomized MSAC
	SAC_MLESAC - Maximum LikeLihood Estimation SAmple Consensus
	SAC_PROSAC - PROgressive SAmple Consensus
	*/
	seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용  , SAC_RANSAC, SAC_PROSAC, SAC_MLESAC, SAC_MSAC
	//seg.setDistanceThreshold(fdistThres); //determines how close a point must be to the model in order to be considered an inlier
	

	// Create pointcloud to publish inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
	int original_size = ptr_cloud->size();
	int n_planes(0);
	int maxIterCntPlane = 3;
	float meanErr = 0;
	int minErrIdx = 0;
	DB_Point_3D minDistPt;
	//&& ptr_cloud->size()>3
	while (((ptr_cloud->size() > (original_size*0.2)) && (ptr_cloud->size() > 3)) && (n_planes < maxIterCntPlane)) {

		//ptr_cloud... 포인트들의 분포...(mean, variance) 분석 후, 정규분포의 T%내의 포인트들로만 구성하여, 평면 정합 수행?
		float meanX = 0, meanY = 0, meanZ = 0;
		float varX = 0, varY = 0, varZ = 0;
		float stdX = 0, stdY = 0, stdZ = 0;
		int numPoints = ptr_cloud->size();
		for (int i = 0; i < numPoints; i++) {

			// Get Point
			pcl::PointXYZ pt = ptr_cloud->points[i];

			// Compute distance
			//double d = point2planedistnace(pt, coefficients) * 1000;// mm

			// Update statistics each Axis...
			meanX += pt.x;
			meanY += pt.y;
			meanZ += pt.z;
		}
		meanX = meanX / numPoints;
		meanY = meanY / numPoints;
		meanZ = meanZ / numPoints;

		for (int i = 0; i < numPoints; i++) {

			// Get Point
			pcl::PointXYZ pt = ptr_cloud->points[i];

			// Update statistics each Axis...
			varX += (pt.x - meanX) * (pt.x - meanX);
			varY += (pt.y - meanY) * (pt.y - meanY);
			varZ += (pt.z - meanZ) * (pt.z - meanZ);
		}
		varX = varX / numPoints;
		varY = varY / numPoints;
		varZ = varZ / numPoints;

		stdX = sqrt(varX);
		stdY = sqrt(varY);
		stdZ = sqrt(varZ);

		//printf("meanX:%f, meanY:%f, meanZ:%f \n", meanX, meanY, meanZ);
		printf("varX:%f, varY:%f, varZ:%f \n", varX, varY, varZ);
		printf("stdX:%f, stdY:%f, stdZ:%f \n", stdX, stdY, stdZ);
		float variance_Thresh = (stdX + stdY + stdZ) / 3 * 0.3;   //0.5, 0.1, 0.2
		printf("variance_Thresh:%f \n", variance_Thresh);

		seg.setDistanceThreshold(variance_Thresh);  //0.1

		seg.setInputCloud(ptr_cloud);
		seg.segment(*inliers, *coefficients);

		printf("original_size*0.2:%.2f, ptr_cloud->size():%d, inliersSize:%d\n", original_size*0.2, ptr_cloud->size(), inliers->indices.size());

		// Check result
		if (inliers->indices.size() <= 3)  //inlier개수가 3개 이하이면, 제낌... => 모델이 제대로 만들어 질 수 없데...
			break;

		// Iterate inliers
		double mean_error(0);
		double max_error(0);
		double min_error(1000000);
		std::vector<double> err;

		//inlier_points.clear();
		inlier_points.push_back(vector<DB_Point_3D>());
		coef.push_back(vector<float>());

		vector<bool> remain_pt_idx(ptr_cloud->size(), true);
		for (int i = 0; i < inliers->indices.size(); i++) {

			// Get Point
			pcl::PointXYZ pt = ptr_cloud->points[inliers->indices[i]];

			// Compute distance
			double d = point2planedistnace(pt, coefficients); //* 1000;// mm
			err.push_back(d);

			// Update statistics
			mean_error += d;
			if (d > max_error) max_error = d;
			if (d < min_error)
			{
				min_error = d;
				minErrIdx = i;
			}

			inlier_points[n_planes].push_back(tmp_points[inliers->indices[i]]);
			remain_pt_idx[inliers->indices[i]] = false;
		}
		mean_error /= inliers->indices.size();
		meanErr = mean_error;
		printf("planeToPoints meandist:%f\n", meanErr); 

		minDistPt = tmp_points[inliers->indices[minErrIdx]];

		//tmp_points.clear();
		//tmp_points.resize(inlier_points[n_planes].size());
		//copy(inlier_points[n_planes].begin(), inlier_points[n_planes].end(), tmp_points.begin());
		vector<DB_Point_3D> tmp_point2;
		for (int i = 0; i < tmp_points.size(); ++i)
		{
			if (remain_pt_idx[i] == false) continue;
			tmp_point2.push_back(tmp_points[i]);
		}
		printf("tmp_points:%d, tmp_point2:%d, remain_pt_idx:%d\n", tmp_points.size(), tmp_point2.size(), remain_pt_idx.size());
		tmp_points.clear();
		tmp_points.resize(tmp_point2.size());
		copy(tmp_point2.begin(), tmp_point2.end(), tmp_points.begin());


		coef[n_planes].push_back(coefficients->values[0]);
		coef[n_planes].push_back(coefficients->values[1]);
		coef[n_planes].push_back(coefficients->values[2]);
		coef[n_planes].push_back(coefficients->values[3]);


		//double sigma(0);
		//for (int i = 0; i < inliers->indices.size(); i++) {
		//	sigma += pow(err[i] - mean_error, 2);
		//	// Get Point
		//	pcl::PointXYZ pt = ptr_cloud->points[inliers->indices[i]];
		//	// Copy point to noew cloud
		//	pcl::PointXYZRGB pt_color;
		//	pt_color.x = pt.x;
		//	pt_color.y = pt.y;
		//	pt_color.z = pt.z;
		//	pt_color.r = color[n_planes][0];
		//	pt_color.g = color[n_planes][1];
		//	pt_color.b = color[n_planes][2];
		//	cloud_pub->points.push_back(pt_color);
		//}
		//sigma = sqrt(sigma / inliers->indices.size());


		// Extract inliers
		extract.setInputCloud(ptr_cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);              //true면, filter를 거치고 난 후 inlier의 나머지를 반환해줌.
		pcl::PointCloud<pcl::PointXYZ> cloudF;
		extract.filter(cloudF);
		ptr_cloud->swap(cloudF);

		// Nest iteration
		n_planes++;

	}


	return meanErr;
}








void H_VOXEL_DB::dfs(int nLv, H_Voxels* h_vox, vector<H_Voxels>& h_vox_child, int nLabel)
{
	if (nLv == 2)
		return;

	//outlier 제거 후 첫 클러스터링...
	printf("\n -------------------------------------------------------\n");
	printf("\n --- Hierarchical Clustering initialization... [level %d, nLabel:%d] ---\n", nLv, nLabel);

	h_vox->nLevel = nLv;
	//h_vox->nLabel = nLabel;

	float maxX = -90000000, maxY = -90000000, maxZ = -90000000;
	float minX =  90000000, minY =  90000000, minZ =  90000000;
	float sumX = 0, sumY = 0, sumZ = 0;
	for (int pt_id = 0; pt_id < h_vox->vec_pt_3d_db.size(); ++pt_id)
	{
		if (maxX < h_vox->vec_pt_3d_db[pt_id].x) maxX = h_vox->vec_pt_3d_db[pt_id].x;
		if (maxY < h_vox->vec_pt_3d_db[pt_id].y) maxY = h_vox->vec_pt_3d_db[pt_id].y;
		if (maxZ < h_vox->vec_pt_3d_db[pt_id].z) maxZ = h_vox->vec_pt_3d_db[pt_id].z;

		if (minX > h_vox->vec_pt_3d_db[pt_id].x) minX = h_vox->vec_pt_3d_db[pt_id].x;
		if (minY > h_vox->vec_pt_3d_db[pt_id].y) minY = h_vox->vec_pt_3d_db[pt_id].y;
		if (minZ > h_vox->vec_pt_3d_db[pt_id].z) minZ = h_vox->vec_pt_3d_db[pt_id].z;
	}
	printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

	float cubeW = maxX - minX;
	float cubeH = maxY - minY;
	float cubeD = maxZ - minZ;
	float maxLen = max(max(cubeW, cubeH), cubeD);

	//그냥 그대로 w,h,d 하는 것과, 그 w,h,d를 나누는 구간을 최대한 동일하게 맞추기 위한 방법... 2가지로 나눠 실험해보기.
	int width = m_cubeSize;   //x
	int height = m_cubeSize;  //y
	int depth = m_cubeSize;   //z
	//int width = m_cubeSize * cubeW / maxLen;   //x
	//int height = m_cubeSize * cubeH / maxLen;  //y
	//int depth = m_cubeSize * cubeD / maxLen;   //z
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

	//vector<H_Voxels> tmp_vec_vox;
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

				for (int pt_id = 0; pt_id < h_vox->vec_pt_3d_db.size(); ++pt_id)
				{
					if ((minRangeX <= h_vox->vec_pt_3d_db[pt_id].x && h_vox->vec_pt_3d_db[pt_id].x <= maxRangeX)
						&& (minRangeY <= h_vox->vec_pt_3d_db[pt_id].y && h_vox->vec_pt_3d_db[pt_id].y <= maxRangeY)
						&& (minRangeZ <= h_vox->vec_pt_3d_db[pt_id].z && h_vox->vec_pt_3d_db[pt_id].z <= maxRangeZ))
					{
						h_vox->vec_pt_3d_db[pt_id].clusterID = nLabel+1;
						cube_points[x - 1][y - 1][z - 1].push_back(h_vox->vec_pt_3d_db[pt_id]);
					}
				}

			} // z
		} // y
	}

	int threshold = 5;  //20, 10
	printf("threshold:%d\n", threshold);

	//if (nLabel == 78)

	int voxelCnt = 0;
	nLabel = 0;
	for (int x = 0; x < width; ++x)
	{
		for (int y = 0; y < height; ++y)
		{
			for (int z = 0; z < depth; ++z)
			{
				int size = cube_points[x][y][z].size();
				if (size > threshold)
				{
					++nLabel;

					H_Voxels tmp_db_pt;
					tmp_db_pt.nLabel = nLabel;
					tmp_db_pt.nLevel = nLv;
					for (int k = 0; k < cube_points[x][y][z].size(); ++k)
						cube_points[x][y][z][k].clusterID = nLabel;
					tmp_db_pt.vec_pt_3d_db = cube_points[x][y][z];
					h_vox_child.push_back(tmp_db_pt);
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

	printf("\n --- h_vox_child Size : %d ---\n", h_vox_child.size());
	for (int id = 0; id < h_vox_child.size(); ++id)
	{
		dfs(nLv + 1, &h_vox_child[id], h_vox_child[id].vec_H_voxels, id);
	}


}

void H_VOXEL_DB::dfs_2(int nLv, H_Voxels* h_vox, vector<H_Voxels>& h_vox_child, int nLabel, int &nAllvoxCnt)
{
	if (nLv == 2)
		return;

	//outlier 제거 후 첫 클러스터링...
	printf("\n -------------------------------------------------------\n");
	printf("\n --- Hierarchical Clustering initialization... [level %d, nLabel:%d] ---\n", nLv, nLabel);

	h_vox->nLevel = nLv-1;
	//h_vox->nLabel = nLabel;

	float maxX = -90000000, maxY = -90000000, maxZ = -90000000;
	float minX = 90000000, minY = 90000000, minZ = 90000000;
	float sumX = 0, sumY = 0, sumZ = 0;
	for (int pt_id = 0; pt_id < h_vox->vec_pt_3d_db.size(); ++pt_id)
	{
		if (maxX < h_vox->vec_pt_3d_db[pt_id].x) maxX = h_vox->vec_pt_3d_db[pt_id].x;
		if (maxY < h_vox->vec_pt_3d_db[pt_id].y) maxY = h_vox->vec_pt_3d_db[pt_id].y;
		if (maxZ < h_vox->vec_pt_3d_db[pt_id].z) maxZ = h_vox->vec_pt_3d_db[pt_id].z;

		if (minX > h_vox->vec_pt_3d_db[pt_id].x) minX = h_vox->vec_pt_3d_db[pt_id].x;
		if (minY > h_vox->vec_pt_3d_db[pt_id].y) minY = h_vox->vec_pt_3d_db[pt_id].y;
		if (minZ > h_vox->vec_pt_3d_db[pt_id].z) minZ = h_vox->vec_pt_3d_db[pt_id].z;
	}
	printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

	float cubeW = maxX - minX;
	float cubeH = maxY - minY;
	float cubeD = maxZ - minZ;
	float maxLen = max(max(cubeW, cubeH), cubeD);

	//그냥 그대로 w,h,d 하는 것과, 그 w,h,d를 나누는 구간을 최대한 동일하게 맞추기 위한 방법... 2가지로 나눠 실험해보기.
	//int width = m_cubeSize;   //x
	//int height = m_cubeSize;  //y
	//int depth = m_cubeSize;   //z
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

				for (int pt_id = 0; pt_id < h_vox->vec_pt_3d_db.size(); ++pt_id)
				{
					if ((minRangeX <= h_vox->vec_pt_3d_db[pt_id].x && h_vox->vec_pt_3d_db[pt_id].x <= maxRangeX)
						&& (minRangeY <= h_vox->vec_pt_3d_db[pt_id].y && h_vox->vec_pt_3d_db[pt_id].y <= maxRangeY)
						&& (minRangeZ <= h_vox->vec_pt_3d_db[pt_id].z && h_vox->vec_pt_3d_db[pt_id].z <= maxRangeZ))
					{
						h_vox->vec_pt_3d_db[pt_id].clusterID = nLabel + 1;
						cube_points[x - 1][y - 1][z - 1].push_back(h_vox->vec_pt_3d_db[pt_id]);
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

			} // z
		} // y
	}

	int threshold = 5;  //20, 10
	printf("threshold:%d\n", threshold);


	int voxelCnt = 0;
	nLabel = 0;
	for (int x = 0; x < width; ++x)
	{
		for (int y = 0; y < height; ++y)
		{
			for (int z = 0; z < depth; ++z)
			{
				int size = cube_points[x][y][z].size();
				if (size > threshold)
				{
					++nAllvoxCnt;
					++nLabel;

					H_Voxels tmp_db_pt;
					tmp_db_pt.nLabel = nLabel;
					tmp_db_pt.nLevel = nLv;
					for (int k = 0; k < cube_points[x][y][z].size(); ++k)
						cube_points[x][y][z][k].clusterID = nAllvoxCnt;

					tmp_db_pt.vec_pt_3d_db = cube_points[x][y][z];
					h_vox_child.push_back(tmp_db_pt);
					h_vox_child[nLabel - 1].planes = tmpVoxels[voxelCnt];
					tmpVoxels[voxelCnt].clusterID = nLabel;

				}
				
				++voxelCnt;
			}  //z

			
		}
	}


	//graph 만들기
	nLabel = 0;
	int kernelSize = 2;
	for (int x = 0; x < width; ++x)
	{
		for (int y = 0; y < height; ++y)
		{
			for (int z = 0; z < depth; ++z)
			{
				int size = cube_points[x][y][z].size();
				if (size > threshold)
				{
					++nLabel;
					for (int xx = x - kernelSize; xx <= x + kernelSize; ++xx)
					{
						if (xx < 0 || xx > width - 1) continue;
						for (int yy = y - kernelSize; yy <= y + kernelSize; ++yy)
						{
							if (yy < 0 || yy > height - 1) continue;
							for (int zz = z - kernelSize; zz <= z + kernelSize; ++zz)
							{
								if (zz < 0 || zz > depth - 1) continue;
								if (xx == x && yy == y && zz == z)
									continue;

								//있을때만 추가.
								if (cube_points[xx][yy][zz].size() > threshold)
									h_vox_child[nLabel - 1].graph_vox.node.push_back(cube_points[xx][yy][zz][0].clusterID);
							}
						}
					}

				}

			} //z
		} //y
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

	//여기 손좀 보자... h_vox_child는 특정 threshold가 넘을 떄만 할당되므로... Hierarchical K-means 처럼 부모와 그 자식들이 각각 다 같은 개수를 같는것이 아니다.
	printf("\n --- h_vox_child Size : %d ---\n", h_vox_child.size());
	for (int id = 0; id < h_vox_child.size(); ++id)
	{
		dfs_2(nLv + 1, &h_vox_child[id], h_vox_child[id].vec_H_voxels, id, nAllvoxCnt);
	}


}


void H_VOXEL_DB::H_voxelFitting(vector<H_Voxels>& points, vector<DB_Voxels>& voxels, int num_points, int color[][3], int &nLabel)
{
	int nPrevCnt = 0;
	int nCurrCnt = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<H_Voxels> refinedPoints;

	//처음은 voxel에 대한 라벨링이 안되어있으므로, 사이즈를 1만큼 준다.
	//refinedPoints.clear();
	//refinedPoints.resize(1);
	//refinedPoints[0].nLabel = -1;
	//refinedPoints[0].nLevel = 0;
	//refinedPoints[0].vec_pt_3d_db.clear();
	//refinedPoints[0].vec_pt_3d_db.resize(points.size());
	//copy(points.begin(), points.end(), refinedPoints[0].vec_pt_3d_db.begin());
	refinedPoints.clear(); 
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());


	////결과 보기
	//for (int i = 0; i < refinedPoints.size(); ++i)
	//{

	//	for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
	//	{
	//		float X = refinedPoints[i].vec_pt_3d_db[pt_id].x;
	//		float Y = refinedPoints[i].vec_pt_3d_db[pt_id].y;
	//		float Z = refinedPoints[i].vec_pt_3d_db[pt_id].z;


	//		// Copy point to noew cloud
	//		pcl::PointXYZRGB pt_color;
	//		pt_color.x = X;
	//		pt_color.y = Y;
	//		pt_color.z = Z;

	//		pt_color.r = 255;
	//		pt_color.g = 0;
	//		pt_color.b = 0;

	//		ptr_cloud->points.push_back(pt_color);
	//	}
	//}
	//pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer2");
	//viewer1.showCloud(ptr_cloud);
	//while (!viewer1.wasStopped())
	//{
	//}



	int debug = 1;
	int _cubeSize = 15;  // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10

	int nAllCnt = 0;
	while (1)
	{
		float maxX = -90000000, maxY = -90000000, maxZ = -90000000;
		float minX = 90000000, minY = 90000000, minZ = 90000000;
		float sumX = 0, sumY = 0, sumZ = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
			{
				if (maxX < refinedPoints[i].vec_pt_3d_db[pt_id].x) maxX = refinedPoints[i].vec_pt_3d_db[pt_id].x;
				if (maxY < refinedPoints[i].vec_pt_3d_db[pt_id].y) maxY = refinedPoints[i].vec_pt_3d_db[pt_id].y;
				if (maxZ < refinedPoints[i].vec_pt_3d_db[pt_id].z) maxZ = refinedPoints[i].vec_pt_3d_db[pt_id].z;

				if (minX > refinedPoints[i].vec_pt_3d_db[pt_id].x) minX = refinedPoints[i].vec_pt_3d_db[pt_id].x;
				if (minY > refinedPoints[i].vec_pt_3d_db[pt_id].y) minY = refinedPoints[i].vec_pt_3d_db[pt_id].y;
				if (minZ > refinedPoints[i].vec_pt_3d_db[pt_id].z) minZ = refinedPoints[i].vec_pt_3d_db[pt_id].z;

				sumX += refinedPoints[i].vec_pt_3d_db[pt_id].x;
				sumY += refinedPoints[i].vec_pt_3d_db[pt_id].y;
				sumZ += refinedPoints[i].vec_pt_3d_db[pt_id].z;
			}
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

		//vector<H_Voxels> ***cube_points = new vector<H_Voxels> **[width];
		//for (int x = 0; x < width; ++x)
		//{
		//	cube_points[x] = new vector<H_Voxels> *[height];
		//	for (int y = 0; y < height; ++y)
		//	{
		//		cube_points[x][y] = new vector<H_Voxels>[depth];
		//	}
		//}

		//vector<H_Voxels> cube_points;

		vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
		for (int x = 0; x < width; ++x)
		{
			cube_points[x] = new vector<DB_Point_3D> *[height];
			for (int y = 0; y < height; ++y)
			{
				cube_points[x][y] = new vector<DB_Point_3D>[depth];
			}
		}

		int nLabel = 0;
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

					++nLabel;

					for (int i = 0; i < refinedPoints.size(); ++i)
					{
						for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
						{
							if ((minRangeX <= refinedPoints[i].vec_pt_3d_db[pt_id].x && refinedPoints[i].vec_pt_3d_db[pt_id].x <= maxRangeX)
								&& (minRangeY <= refinedPoints[i].vec_pt_3d_db[pt_id].y && refinedPoints[i].vec_pt_3d_db[pt_id].y <= maxRangeY)
								&& (minRangeZ <= refinedPoints[i].vec_pt_3d_db[pt_id].z && refinedPoints[i].vec_pt_3d_db[pt_id].z <= maxRangeZ))
							{
								//refinedPoints[i].vec_pt_3d_db[pt_id].clusterID = nLabel;
								//cube_points.push_back(refinedPoints[i]);
								cube_points[x - 1][y - 1][z - 1].push_back(refinedPoints[i].vec_pt_3d_db[pt_id]);
							}

						}

					}

				}
			}
		}

		int nPts_sum = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			nPts_sum += refinedPoints[i].vec_pt_3d_db.size();
		}

		int threshold = nPts_sum * 0.0001;   // Aachen = 0.0001;
		printf("11 refinedPoints:%d\n\n", nPts_sum);

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
					if (size >= threshold)
					{
						++nCurrCnt;

						//2021.06.23 TO do!!!!!!!!!!!!!!!!
						H_Voxels tmp;
						tmp.nLabel = nCurrCnt;
						tmp.nLevel = 0;
						tmp.vec_pt_3d_db = cube_points[x][y][z];
						refinedPoints.push_back(tmp);

						for (int i = 0; i < cube_points[x][y][z].size(); ++i)
						{
							cube_points[x][y][z][i].clusterID = nCurrCnt;
							//refinedPoints.push_back(cube_points[x][y][z][i]);

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

		int nPts_sum2 = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			nPts_sum2 += refinedPoints[i].vec_pt_3d_db.size();
		}

		printf("nPrevCnt:%d, nCurrCnt:%d\n", nPrevCnt, nCurrCnt);
		printf("22 refinedPoints:%d\n\n", nPts_sum2);


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

		if (nAllCnt > 2) break;   // CMU, Aachen => 2, cambridge = 1
		++nAllCnt;

	}  //while

	if (debug)
	{
		//결과 보기
		pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer2");
		viewer1.showCloud(ptr_cloud);
		while (!viewer1.wasStopped())
		{
		}
	}

	//To do...: dfs 로 구현 해야 할 듯... 재귀함수 이용해서...
	//하나의 복셀에 해당하는 육면체 정보.. -> DB_Voxels tmp;  이 정보 넣고, point cloud 에 최하단 레벨에 해당하는 최종 label 계산해서 이미지로 보기.!
	printf("\n -------------------------------------------------------\n");
	printf("\n --- Hierarchical Clustering initialization... [level %d] ---\n", 0);

	for (int i = 0; i < refinedPoints.size(); ++i)
	{
		dfs(1, &refinedPoints[i], refinedPoints[i].vec_H_voxels, i);
	}

	//int nAll_Label_cnt = refinedPoints.size();
	//for (int i = 0; i < refinedPoints.size(); ++i)
	//{
	//	nAll_Label_cnt += refinedPoints[i].vec_H_voxels.size();
	//}
	//printf("\n --- nAll_Label_cnt: %d ---\n", nAll_Label_cnt);

	ptr_cloud->clear();
	ptr_cloud->points.clear();

	//vector<int> nCntLabel = vector<int>(nCurrCnt, 0);         //각 라벨에 속하는 포인트의 수

	// 추후에 level 깊이에 따른... 검색을 위해 DFS로 구현해야함.
	int nAll_Label_cnt =0;
	int nEachLabel = 0;
	for (int i = 0; i < refinedPoints.size(); ++i)
	{
		int nLabel1 = refinedPoints[i].nLabel;
		nAll_Label_cnt += refinedPoints[i].vec_H_voxels.size();
		for (int j = 0; j < refinedPoints[i].vec_H_voxels.size(); ++j)
		{
			int nLabel2 = refinedPoints[i].vec_H_voxels[j].nLabel;
			++nEachLabel;

			printf("lb1:%d, lb2:%d, nLabel:%d\n", nLabel1, nLabel2, nEachLabel);
			
			for (int pt_id = 0; pt_id < refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db.size(); ++pt_id)
			{
				// Copy point to noew cloud
				pcl::PointXYZRGB pt_color;
				pt_color.x = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].x;
				pt_color.y = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].y;
				pt_color.z = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].z;

				pt_color.r = color[nEachLabel - 1][0];
				pt_color.g = color[nEachLabel - 1][1];
				pt_color.b = color[nEachLabel - 1][2];

				ptr_cloud->points.push_back(pt_color);
			}


		}
	}
	printf("\n --- nAll_Label_cnt: %d ---\n", nAll_Label_cnt);
	nEachLabel = nLabel;

	if (debug)
	{
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			//for (int j = 0; j < refinedPoints[i].vec_H_voxels.size(); ++j)
			//{
			//	float maxX = -90000000, maxY = -90000000, maxZ = -90000000;
			//	float minX = 90000000, minY = 90000000, minZ = 90000000;
			//	for (int pt_id = 0; pt_id < refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db.size(); ++pt_id)
			//	{
			//		float x = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].x;
			//		float y = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].y;
			//		float z = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].z;

			//		if (maxX < x) maxX = x;
			//		if (maxY < y) maxY = y;
			//		if (maxZ < z) maxZ = z;

			//		if (minX > x) minX = x;
			//		if (minY > y) minY = y;
			//		if (minZ > z) minZ = z;
			//	}

			//	int width = m_cubeSize;   //x
			//	int height = m_cubeSize;  //y
			//	int depth = m_cubeSize;   //z

			//	//vector<H_Voxels> tmp_vec_vox;
			//	for (int x = 1; x <= width; ++x)
			//	{
			//		float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			//		float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			//		for (int y = 1; y <= height; ++y)
			//		{
			//			float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
			//			float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
			//			for (int z = 1; z <= depth; ++z)
			//			{
			//				float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
			//				float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

			//				// Copy point to noew cloud
			//				pcl::PointXYZRGB pt_color;
			//				pt_color.x = minRangeX;
			//				pt_color.y = minRangeY;
			//				pt_color.z = minRangeZ;
			//				pt_color.r = 255;
			//				pt_color.g = 0;
			//				pt_color.b = 0;

			//				ptr_cloud->points.push_back(pt_color);

			//			} // z
			//		} // y
			//	} //x
			//} // j -> refinedPoints[i].vec_H_voxels.size()

			//float maxX = -90000000, maxY = -90000000, maxZ = -90000000;
			//float minX = 90000000, minY = 90000000, minZ = 90000000;
			//for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
			//{
			//	float x = refinedPoints[i].vec_pt_3d_db[pt_id].x;
			//	float y = refinedPoints[i].vec_pt_3d_db[pt_id].y;
			//	float z = refinedPoints[i].vec_pt_3d_db[pt_id].z;

			//	if (maxX < x) maxX = x;
			//	if (maxY < y) maxY = y;
			//	if (maxZ < z) maxZ = z;

			//	if (minX > x) minX = x;
			//	if (minY > y) minY = y;
			//	if (minZ > z) minZ = z;
			//}

			//int width = m_cubeSize;   //x
			//int height = m_cubeSize;  //y
			//int depth = m_cubeSize;   //z

			//for (int x = 1; x <= width; ++x)
			//{
			//	float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			//	float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			//	for (int y = 1; y <= height; ++y)
			//	{
			//		float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
			//		float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
			//		for (int z = 1; z <= depth; ++z)
			//		{
			//			float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
			//			float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

			//			// Copy point to noew cloud
			//			pcl::PointXYZRGB pt_color;
			//			pt_color.x = minRangeX;
			//			pt_color.y = minRangeY;
			//			pt_color.z = minRangeZ;
			//			pt_color.r = 255;
			//			pt_color.g = 0;
			//			pt_color.b = 0;

			//			ptr_cloud->points.push_back(pt_color);

			//		} // z
			//	} // y
			//} //x

		}

	}


	if (debug)
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
















	
	////outlier 제거 후 첫 클러스터링...
	//printf("\n -------------------------------------------------------\n");
	//printf("\n --- Hierarchical Clustering initialization... [level 0] ---\n");

	//float maxX = -100000, maxY = -100000, maxZ = -100000;
	//float minX = 1000000, minY = 1000000, minZ = 1000000;
	//float sumX = 0, sumY = 0, sumZ = 0;
	//for (int i = 0; i < refinedPoints.size(); ++i)
	//{
	//	if (maxX < refinedPoints[i].x) maxX = refinedPoints[i].x;
	//	if (maxY < refinedPoints[i].y) maxY = refinedPoints[i].y;
	//	if (maxZ < refinedPoints[i].z) maxZ = refinedPoints[i].z;

	//	if (minX > refinedPoints[i].x) minX = refinedPoints[i].x;
	//	if (minY > refinedPoints[i].y) minY = refinedPoints[i].y;
	//	if (minZ > refinedPoints[i].z) minZ = refinedPoints[i].z;
	//}
	//printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

	//float cubeW = maxX - minX;
	//float cubeH = maxY - minY;
	//float cubeD = maxZ - minZ;
	//float maxLen = max(max(cubeW, cubeH), cubeD);

	//ptr_cloud->clear();
	//ptr_cloud->points.clear();
	//nCurrCnt = 0;

	//int width = m_cubeSize;   //x
	//int height = m_cubeSize;  //y
	//int depth = m_cubeSize;   //z
	//if (width <= 0) width = 1;
	//if (height <= 0) height = 1;
	//if (depth <= 0) depth = 1;
	//printf("width:%d, height:%d, depth:%d\n", width, height, depth);

	//vector<H_Voxels> ***cube_points = new vector<H_Voxels> **[width];
	//for (int x = 0; x < width; ++x)
	//{
	//	cube_points[x] = new vector<H_Voxels> *[height];
	//	for (int y = 0; y < height; ++y)
	//	{
	//		cube_points[x][y] = new vector<H_Voxels>[depth];
	//	}
	//}

	//for (int x = 1; x <= width; ++x)
	//{
	//	float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
	//	float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

	//	for (int y = 1; y <= height; ++y)
	//	{
	//		float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
	//		float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
	//		for (int z = 1; z <= depth; ++z)
	//		{
	//			float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
	//			float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

	//			++nCurrCnt;

	//			for (int i = 0; i < refinedPoints.size(); ++i)
	//			{
	//				if ((minRangeX <= refinedPoints[i].x && refinedPoints[i].x <= maxRangeX)
	//					&& (minRangeY <= refinedPoints[i].y && refinedPoints[i].y <= maxRangeY)
	//					&& (minRangeZ <= refinedPoints[i].z && refinedPoints[i].z <= maxRangeZ))
	//				{
	//					refinedPoints[i].clusterID = nCurrCnt;
	//					cube_points[x - 1][y - 1][z - 1].push_back(refinedPoints[i]);
	//				}
	//			}
	//		}
	//	}
	//}

	////To do!!!! - 2021.03.15 - 계층적 클러스터링 구현하기!
	//int level = 2;
	//nCurrCnt = 0;
	//vector<DB_Voxels> tmpVoxels;
	//for (int lv = 1; lv < level; ++lv)
	//{
	//	printf("\n -------------------------------------------------------\n");
	//	printf("\n --- Hierarchical Clustering [level:%d] ---\n", lv);


	//	for (int x = 0; x < width; ++x)
	//	{
	//		for (int y = 0; y < height; ++y)
	//		{
	//			for (int z = 0; z < depth; ++z)
	//			{

	//				vector<H_Voxels> tmp_db_pt = cube_points[x][y][z];

	//				float maxX = -100000, maxY = -100000, maxZ = -100000;
	//				float minX = 1000000, minY = 1000000, minZ = 1000000;
	//				float sumX = 0, sumY = 0, sumZ = 0;
	//				for (int i = 0; i < tmp_db_pt.size(); ++i)
	//				{
	//					if (maxX < tmp_db_pt[i].x) maxX = tmp_db_pt[i].x;
	//					if (maxY < tmp_db_pt[i].y) maxY = tmp_db_pt[i].y;
	//					if (maxZ < tmp_db_pt[i].z) maxZ = tmp_db_pt[i].z;

	//					if (minX > tmp_db_pt[i].x) minX = tmp_db_pt[i].x;
	//					if (minY > tmp_db_pt[i].y) minY = tmp_db_pt[i].y;
	//					if (minZ > tmp_db_pt[i].z) minZ = tmp_db_pt[i].z;
	//				}
	//				printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);


	//				//float cubeW = maxX - minX;
	//				//float cubeH = maxY - minY;
	//				//float cubeD = maxZ - minZ;
	//				//float maxLen = max(max(cubeW, cubeH), cubeD);
	//				//int width = m_cubeSize * cubeW / maxLen;   //x
	//				//int height = m_cubeSize * cubeH / maxLen;  //y
	//				//int depth = m_cubeSize * cubeD / maxLen;   //z
	//				int width = m_cubeSize;   //x
	//				int height = m_cubeSize;  //y
	//				int depth = m_cubeSize;   //z
	//				if (width <= 0) width = 1;
	//				if (height <= 0) height = 1;
	//				if (depth <= 0) depth = 1;
	//				printf("width:%d, height:%d, depth:%d\n", width, height, depth);


	//				for (int xx = 1; xx <= width; ++xx)
	//				{
	//					float minRangeX = minX + (maxX - minX) * ((float)(xx - 1) / width);
	//					float maxRangeX = minX + (maxX - minX) * ((float)(xx) / width);

	//					for (int yy = 1; yy <= height; ++yy)
	//					{
	//						float minRangeY = minY + (maxY - minY) * ((float)(yy - 1) / height);
	//						float maxRangeY = minY + (maxY - minY) * ((float)(yy) / height);
	//						for (int zz = 1; zz <= depth; ++zz)
	//						{
	//							float minRangeZ = minZ + (maxZ - minZ) * ((float)(zz - 1) / depth);
	//							float maxRangeZ = minZ + (maxZ - minZ) * ((float)(zz) / depth);

	//							++nCurrCnt;

	//							for (int i = 0; i < tmp_db_pt.size(); ++i)
	//							{
	//								if ((minRangeX <= tmp_db_pt[i].x && tmp_db_pt[i].x <= maxRangeX)
	//									&& (minRangeY <= tmp_db_pt[i].y && tmp_db_pt[i].y <= maxRangeY)
	//									&& (minRangeZ <= tmp_db_pt[i].z && tmp_db_pt[i].z <= maxRangeZ))
	//								{
	//									tmp_db_pt[i].clusterID = nCurrCnt;
	//									//cube_points[x - 1][y - 1][z - 1].push_back(tmp_db_pt[i]);
	//								}
	//							}

	//							//하나의 복셀에 해당하는 육면체 정보..
	//							DB_Voxels tmp;
	//							tmp.planes[0].x[0] = maxRangeX; tmp.planes[0].y[0] = maxRangeY; tmp.planes[0].z[0] = maxRangeZ;
	//							tmp.planes[0].x[1] = maxRangeX; tmp.planes[0].y[1] = minRangeY; tmp.planes[0].z[1] = maxRangeZ;
	//							tmp.planes[0].x[2] = minRangeX; tmp.planes[0].y[2] = minRangeY; tmp.planes[0].z[2] = maxRangeZ;
	//							tmp.planes[0].x[3] = minRangeX; tmp.planes[0].y[3] = maxRangeY; tmp.planes[0].z[3] = maxRangeZ;
	//							plane_normVec(tmp.planes[0]);

	//							tmp.planes[1].x[0] = maxRangeX; tmp.planes[1].y[0] = maxRangeY; tmp.planes[1].z[0] = minRangeZ;
	//							tmp.planes[1].x[1] = maxRangeX; tmp.planes[1].y[1] = minRangeY; tmp.planes[1].z[1] = minRangeZ;
	//							tmp.planes[1].x[2] = maxRangeX; tmp.planes[1].y[2] = minRangeY; tmp.planes[1].z[2] = maxRangeZ;
	//							tmp.planes[1].x[3] = maxRangeX; tmp.planes[1].y[3] = maxRangeY; tmp.planes[1].z[3] = maxRangeZ;
	//							plane_normVec(tmp.planes[1]);

	//							tmp.planes[2].x[0] = minRangeX; tmp.planes[2].y[0] = maxRangeY; tmp.planes[2].z[0] = minRangeZ;
	//							tmp.planes[2].x[1] = minRangeX; tmp.planes[2].y[1] = minRangeY; tmp.planes[2].z[1] = minRangeZ;
	//							tmp.planes[2].x[2] = maxRangeX; tmp.planes[2].y[2] = minRangeY; tmp.planes[2].z[2] = minRangeZ;
	//							tmp.planes[2].x[3] = maxRangeX; tmp.planes[2].y[3] = maxRangeY; tmp.planes[2].z[3] = minRangeZ;
	//							plane_normVec(tmp.planes[2]);

	//							tmp.planes[3].x[0] = minRangeX; tmp.planes[3].y[0] = maxRangeY; tmp.planes[3].z[0] = maxRangeZ;
	//							tmp.planes[3].x[1] = minRangeX; tmp.planes[3].y[1] = minRangeY; tmp.planes[3].z[1] = maxRangeZ;
	//							tmp.planes[3].x[2] = minRangeX; tmp.planes[3].y[2] = minRangeY; tmp.planes[3].z[2] = minRangeZ;
	//							tmp.planes[3].x[3] = minRangeX; tmp.planes[3].y[3] = maxRangeY; tmp.planes[3].z[3] = minRangeZ;
	//							plane_normVec(tmp.planes[3]);

	//							tmp.planes[4].x[0] = maxRangeX; tmp.planes[4].y[0] = maxRangeY; tmp.planes[4].z[0] = minRangeZ;
	//							tmp.planes[4].x[1] = maxRangeX; tmp.planes[4].y[1] = maxRangeY; tmp.planes[4].z[1] = maxRangeZ;
	//							tmp.planes[4].x[2] = minRangeX; tmp.planes[4].y[2] = maxRangeY; tmp.planes[4].z[2] = maxRangeZ;
	//							tmp.planes[4].x[3] = minRangeX; tmp.planes[4].y[3] = maxRangeY; tmp.planes[4].z[3] = minRangeZ;
	//							plane_normVec(tmp.planes[4]);

	//							tmp.planes[5].x[0] = maxRangeX; tmp.planes[5].y[0] = minRangeY; tmp.planes[5].z[0] = maxRangeZ;
	//							tmp.planes[5].x[1] = maxRangeX; tmp.planes[5].y[1] = minRangeY; tmp.planes[5].z[1] = minRangeZ;
	//							tmp.planes[5].x[2] = minRangeX; tmp.planes[5].y[2] = minRangeY; tmp.planes[5].z[2] = minRangeZ;
	//							tmp.planes[5].x[3] = minRangeX; tmp.planes[5].y[3] = minRangeY; tmp.planes[5].z[3] = maxRangeZ;
	//							plane_normVec(tmp.planes[5]);
	//							tmpVoxels.push_back(tmp);
	//						}
	//					}
	//				}

	//				cube_points[x][y][z] = tmp_db_pt;


	//			}  //z
	//		}
	//	}
	//}



	//printf("--- refinedPoints:%d\n\n", refinedPoints.size());
	//int threshold = 20;
	//printf("threshold:%d\n", threshold);
	//refinedPoints.clear();
	//ptr_cloud->clear();
	//ptr_cloud->points.clear();

	////vector<int> nCntLabel = vector<int>(nCurrCnt, 0);         //각 라벨에 속하는 포인트의 수


	//for (int x = 0; x < width; ++x)
	//{
	//	for (int y = 0; y < height; ++y)
	//	{
	//		for (int z = 0; z < depth; ++z)
	//		{
	//			int size = cube_points[x][y][z].size();
	//			if (size > threshold)
	//			{
	//				for (int i = 0; i < cube_points[x][y][z].size(); ++i)
	//				{
	//					refinedPoints.push_back(cube_points[x][y][z][i]);

	//					int nLabel = cube_points[x][y][z][i].clusterID;

	//					// Copy point to noew cloud
	//					pcl::PointXYZRGB pt_color;
	//					pt_color.x = cube_points[x][y][z][i].x;
	//					pt_color.y = cube_points[x][y][z][i].y;
	//					pt_color.z = cube_points[x][y][z][i].z;

	//					pt_color.r = color[nLabel - 1][0];
	//					pt_color.g = color[nLabel - 1][1];
	//					pt_color.b = color[nLabel - 1][2];

	//					ptr_cloud->points.push_back(pt_color);
	//				}
	//			}


	//		}
	//	}
	//}


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

	//printf("--- nPrevCnt:%d, nCurrCnt:%d\n", nPrevCnt, nCurrCnt);
	//printf("--- refinedPoints:%d\n\n", refinedPoints.size());


	//if (debug)
	//{
	//	for (int x = 1; x <= width + 1; ++x)
	//	{
	//		float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
	//		float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

	//		for (int y = 1; y <= height + 1; ++y)
	//		{
	//			float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
	//			float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
	//			for (int z = 1; z <= depth + 1; ++z)
	//			{
	//				float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
	//				float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

	//				// Copy point to noew cloud
	//				pcl::PointXYZRGB pt_color;
	//				pt_color.x = minRangeX;
	//				pt_color.y = minRangeY;
	//				pt_color.z = minRangeZ;
	//				pt_color.r = 255;
	//				pt_color.g = 0;
	//				pt_color.b = 0;

	//				ptr_cloud->points.push_back(pt_color);
	//			}
	//		}
	//	}

	//}


	//if (debug)
	//{
	//	//결과 보기
	//	pcl::visualization::CloudViewer viewer2("Cloud Viewer");
	//	viewer2.showCloud(ptr_cloud);
	//	while (!viewer2.wasStopped())
	//	{
	//	}
	//}



	points.clear();
	points.resize(refinedPoints.size());
	copy(refinedPoints.begin(), refinedPoints.end(), points.begin());

	nLabel = nCurrCnt;
	

	printf("finish!");

}


void H_VOXEL_DB::H_voxelFitting_allCnt(vector<H_Voxels>& points, vector<DB_Voxels>& voxels, int num_points, int color[][3], int &nLabel)
{
	int nPrevCnt = 0;
	int nCurrCnt = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<H_Voxels> refinedPoints;

	//처음은 voxel에 대한 라벨링이 안되어있으므로, 사이즈를 1만큼 준다.
	//refinedPoints.clear();
	//refinedPoints.resize(1);
	//refinedPoints[0].nLabel = -1;
	//refinedPoints[0].nLevel = 0;
	//refinedPoints[0].vec_pt_3d_db.clear();
	//refinedPoints[0].vec_pt_3d_db.resize(points.size());
	//copy(points.begin(), points.end(), refinedPoints[0].vec_pt_3d_db.begin());
	refinedPoints.clear();
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());



	////결과 보기
	//for (int i = 0; i < refinedPoints.size(); ++i)
	//{

	//	for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
	//	{
	//		float X = refinedPoints[i].vec_pt_3d_db[pt_id].x;
	//		float Y = refinedPoints[i].vec_pt_3d_db[pt_id].y;
	//		float Z = refinedPoints[i].vec_pt_3d_db[pt_id].z;


	//		// Copy point to noew cloud
	//		pcl::PointXYZRGB pt_color;
	//		pt_color.x = X;
	//		pt_color.y = Y;
	//		pt_color.z = Z;

	//		pt_color.r = 255;
	//		pt_color.g = 0;
	//		pt_color.b = 0;

	//		ptr_cloud->points.push_back(pt_color);
	//	}
	//}
	//pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer2");
	//viewer1.showCloud(ptr_cloud);
	//while (!viewer1.wasStopped())
	//{
	//}


	int debug = 0;
	int _cubeSize = 15;  // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10
	//int _cubeSize = m_cubeSize;  // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10

	int nAllCnt = 0;
	while (1)
	{
		float maxX = -90000000, maxY = -90000000, maxZ = -90000000;
		float minX = 90000000, minY = 90000000, minZ = 90000000;
		float sumX = 0, sumY = 0, sumZ = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
			{
				if (maxX < refinedPoints[i].vec_pt_3d_db[pt_id].x) maxX = refinedPoints[i].vec_pt_3d_db[pt_id].x;
				if (maxY < refinedPoints[i].vec_pt_3d_db[pt_id].y) maxY = refinedPoints[i].vec_pt_3d_db[pt_id].y;
				if (maxZ < refinedPoints[i].vec_pt_3d_db[pt_id].z) maxZ = refinedPoints[i].vec_pt_3d_db[pt_id].z;

				if (minX > refinedPoints[i].vec_pt_3d_db[pt_id].x) minX = refinedPoints[i].vec_pt_3d_db[pt_id].x;
				if (minY > refinedPoints[i].vec_pt_3d_db[pt_id].y) minY = refinedPoints[i].vec_pt_3d_db[pt_id].y;
				if (minZ > refinedPoints[i].vec_pt_3d_db[pt_id].z) minZ = refinedPoints[i].vec_pt_3d_db[pt_id].z;

				sumX += refinedPoints[i].vec_pt_3d_db[pt_id].x;
				sumY += refinedPoints[i].vec_pt_3d_db[pt_id].y;
				sumZ += refinedPoints[i].vec_pt_3d_db[pt_id].z;
			}
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
		//int width = _cubeSize; //x
		//int height = _cubeSize;  //y
		//int depth = _cubeSize;   //z
		if (width <= 0) width = 1;
		if (height <= 0) height = 1;
		if (depth <= 0) depth = 1;
		printf("width:%d, height:%d, depth:%d\n", width, height, depth);

		//vector<H_Voxels> ***cube_points = new vector<H_Voxels> **[width];
		//for (int x = 0; x < width; ++x)
		//{
		//	cube_points[x] = new vector<H_Voxels> *[height];
		//	for (int y = 0; y < height; ++y)
		//	{
		//		cube_points[x][y] = new vector<H_Voxels>[depth];
		//	}
		//}

		//vector<H_Voxels> cube_points;

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

		int nLabel = 0;
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

					++nLabel;

					for (int i = 0; i < refinedPoints.size(); ++i)
					{
						for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
						{
							if ((minRangeX <= refinedPoints[i].vec_pt_3d_db[pt_id].x && refinedPoints[i].vec_pt_3d_db[pt_id].x <= maxRangeX)
								&& (minRangeY <= refinedPoints[i].vec_pt_3d_db[pt_id].y && refinedPoints[i].vec_pt_3d_db[pt_id].y <= maxRangeY)
								&& (minRangeZ <= refinedPoints[i].vec_pt_3d_db[pt_id].z && refinedPoints[i].vec_pt_3d_db[pt_id].z <= maxRangeZ))
							{
								cube_points[x - 1][y - 1][z - 1].push_back(refinedPoints[i].vec_pt_3d_db[pt_id]);
							}

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

		int nPts_sum = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			nPts_sum += refinedPoints[i].vec_pt_3d_db.size();
		}

		int threshold = nPts_sum * 0.0001;   // Aachen = 0.0001;
		printf("11 refinedPoints:%d\n\n", nPts_sum);

		printf("threshold:%d\n", threshold);
		nCurrCnt = 0;
		refinedPoints.clear();
		int voxelCnt = 0;
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

						H_Voxels tmp;
						tmp.nLabel = nCurrCnt;
						tmp.nLevel = 0;
						tmp.vec_pt_3d_db = cube_points[x][y][z];
						refinedPoints.push_back(tmp);
						refinedPoints[nCurrCnt-1].planes = tmpVoxels[voxelCnt];

						for (int i = 0; i < cube_points[x][y][z].size(); ++i)
						{
							cube_points[x][y][z][i].clusterID = nCurrCnt;
							//refinedPoints.push_back(cube_points[x][y][z][i]);

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

		int nPts_sum2 = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			nPts_sum2 += refinedPoints[i].vec_pt_3d_db.size();
		}

		printf("nPrevCnt:%d, nCurrCnt:%d\n", nPrevCnt, nCurrCnt);
		printf("22 refinedPoints:%d\n\n", nPts_sum2);


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

		if (nAllCnt > 2) break;   // CMU, Aachen => 2, cambridge = 1
		++nAllCnt;

	}  //while

	if (debug)
	{
		//결과 보기
		pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer2");
		viewer1.showCloud(ptr_cloud);
		while (!viewer1.wasStopped())
		{
		}
	}

	//To do...: dfs 로 구현 해야 할 듯... 재귀함수 이용해서...
	//하나의 복셀에 해당하는 육면체 정보.. -> DB_Voxels tmp;  이 정보 넣고, point cloud 에 최하단 레벨에 해당하는 최종 label 계산해서 이미지로 보기.!
	printf("\n -------------------------------------------------------\n");
	printf("\n --- Hierarchical Clustering initialization... [level %d] ---\n", 0);

	int nAllvoxCnt = 0;
	for (int i = 0; i < refinedPoints.size(); ++i)
	{
		dfs_2(1, &refinedPoints[i], refinedPoints[i].vec_H_voxels, i, nAllvoxCnt);
	}



	ptr_cloud->clear();
	ptr_cloud->points.clear();

	//vector<int> nCntLabel = vector<int>(nCurrCnt, 0);         //각 라벨에 속하는 포인트의 수

	// 추후에 level 깊이에 따른... 검색을 위해 DFS로 구현해야함.
	int nAll_Label_cnt = 0;
	int nEachLabel = 0;
	for (int i = 0; i < refinedPoints.size(); ++i)
	{
		int nLabel1 = refinedPoints[i].nLabel;
		nAll_Label_cnt += refinedPoints[i].vec_H_voxels.size();
		for (int j = 0; j < refinedPoints[i].vec_H_voxels.size(); ++j)
		{
			int nLabel2 = refinedPoints[i].vec_H_voxels[j].nLabel;
			++nEachLabel;
			//refinedPoints[i].vec_H_voxels[j].nLabel = nEachLabel;

			printf("lb1:%d, lb2:%d, nLabel:%d\n", nLabel1, nLabel2, nEachLabel);

			for (int pt_id = 0; pt_id < refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db.size(); ++pt_id)
			{
				// Copy point to noew cloud
				pcl::PointXYZRGB pt_color;
				pt_color.x = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].x;
				pt_color.y = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].y;
				pt_color.z = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].z;
				refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].clusterID = nEachLabel;

				pt_color.r = color[nEachLabel - 1][0];
				pt_color.g = color[nEachLabel - 1][1];
				pt_color.b = color[nEachLabel - 1][2];

				ptr_cloud->points.push_back(pt_color);
			}


		}
	}
	printf("\n --- nAll_Label_cnt: %d ---\n", nAll_Label_cnt);
	nLabel = nEachLabel;

	//그래프 만드는 것까진 했고... 다음 하자. 집가장

	if (debug)
	{
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			//for (int j = 0; j < refinedPoints[i].vec_H_voxels.size(); ++j)
			//{
			//	float maxX = -90000000, maxY = -90000000, maxZ = -90000000;
			//	float minX = 90000000, minY = 90000000, minZ = 90000000;
			//	for (int pt_id = 0; pt_id < refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db.size(); ++pt_id)
			//	{
			//		float x = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].x;
			//		float y = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].y;
			//		float z = refinedPoints[i].vec_H_voxels[j].vec_pt_3d_db[pt_id].z;

			//		if (maxX < x) maxX = x;
			//		if (maxY < y) maxY = y;
			//		if (maxZ < z) maxZ = z;

			//		if (minX > x) minX = x;
			//		if (minY > y) minY = y;
			//		if (minZ > z) minZ = z;
			//	}

			//	int width = m_cubeSize;   //x
			//	int height = m_cubeSize;  //y
			//	int depth = m_cubeSize;   //z

			//	//vector<H_Voxels> tmp_vec_vox;
			//	for (int x = 1; x <= width; ++x)
			//	{
			//		float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			//		float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			//		for (int y = 1; y <= height; ++y)
			//		{
			//			float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
			//			float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
			//			for (int z = 1; z <= depth; ++z)
			//			{
			//				float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
			//				float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

			//				// Copy point to noew cloud
			//				pcl::PointXYZRGB pt_color;
			//				pt_color.x = minRangeX;
			//				pt_color.y = minRangeY;
			//				pt_color.z = minRangeZ;
			//				pt_color.r = 255;
			//				pt_color.g = 0;
			//				pt_color.b = 0;

			//				ptr_cloud->points.push_back(pt_color);

			//			} // z
			//		} // y
			//	} //x
			//} // j -> refinedPoints[i].vec_H_voxels.size()

			//float maxX = -90000000, maxY = -90000000, maxZ = -90000000;
			//float minX = 90000000, minY = 90000000, minZ = 90000000;
			//for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
			//{
			//	float x = refinedPoints[i].vec_pt_3d_db[pt_id].x;
			//	float y = refinedPoints[i].vec_pt_3d_db[pt_id].y;
			//	float z = refinedPoints[i].vec_pt_3d_db[pt_id].z;

			//	if (maxX < x) maxX = x;
			//	if (maxY < y) maxY = y;
			//	if (maxZ < z) maxZ = z;

			//	if (minX > x) minX = x;
			//	if (minY > y) minY = y;
			//	if (minZ > z) minZ = z;
			//}

			//int width = m_cubeSize;   //x
			//int height = m_cubeSize;  //y
			//int depth = m_cubeSize;   //z

			//for (int x = 1; x <= width; ++x)
			//{
			//	float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
			//	float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

			//	for (int y = 1; y <= height; ++y)
			//	{
			//		float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
			//		float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
			//		for (int z = 1; z <= depth; ++z)
			//		{
			//			float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
			//			float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

			//			// Copy point to noew cloud
			//			pcl::PointXYZRGB pt_color;
			//			pt_color.x = minRangeX;
			//			pt_color.y = minRangeY;
			//			pt_color.z = minRangeZ;
			//			pt_color.r = 255;
			//			pt_color.g = 0;
			//			pt_color.b = 0;

			//			ptr_cloud->points.push_back(pt_color);

			//		} // z
			//	} // y
			//} //x

		}

	}


	if (debug)
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






	////To do!!!! - 2021.03.15 - 계층적 클러스터링 구현하기!
	//int level = 2;
	//nCurrCnt = 0;
	//vector<DB_Voxels> tmpVoxels;
	//for (int lv = 1; lv < level; ++lv)
	//{
	//	printf("\n -------------------------------------------------------\n");
	//	printf("\n --- Hierarchical Clustering [level:%d] ---\n", lv);


	//	for (int x = 0; x < width; ++x)
	//	{
	//		for (int y = 0; y < height; ++y)
	//		{
	//			for (int z = 0; z < depth; ++z)
	//			{

	//				vector<H_Voxels> tmp_db_pt = cube_points[x][y][z];

	//				float maxX = -100000, maxY = -100000, maxZ = -100000;
	//				float minX = 1000000, minY = 1000000, minZ = 1000000;
	//				float sumX = 0, sumY = 0, sumZ = 0;
	//				for (int i = 0; i < tmp_db_pt.size(); ++i)
	//				{
	//					if (maxX < tmp_db_pt[i].x) maxX = tmp_db_pt[i].x;
	//					if (maxY < tmp_db_pt[i].y) maxY = tmp_db_pt[i].y;
	//					if (maxZ < tmp_db_pt[i].z) maxZ = tmp_db_pt[i].z;

	//					if (minX > tmp_db_pt[i].x) minX = tmp_db_pt[i].x;
	//					if (minY > tmp_db_pt[i].y) minY = tmp_db_pt[i].y;
	//					if (minZ > tmp_db_pt[i].z) minZ = tmp_db_pt[i].z;
	//				}
	//				printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);


	//				//float cubeW = maxX - minX;
	//				//float cubeH = maxY - minY;
	//				//float cubeD = maxZ - minZ;
	//				//float maxLen = max(max(cubeW, cubeH), cubeD);
	//				//int width = m_cubeSize * cubeW / maxLen;   //x
	//				//int height = m_cubeSize * cubeH / maxLen;  //y
	//				//int depth = m_cubeSize * cubeD / maxLen;   //z
	//				int width = m_cubeSize;   //x
	//				int height = m_cubeSize;  //y
	//				int depth = m_cubeSize;   //z
	//				if (width <= 0) width = 1;
	//				if (height <= 0) height = 1;
	//				if (depth <= 0) depth = 1;
	//				printf("width:%d, height:%d, depth:%d\n", width, height, depth);


	//				for (int xx = 1; xx <= width; ++xx)
	//				{
	//					float minRangeX = minX + (maxX - minX) * ((float)(xx - 1) / width);
	//					float maxRangeX = minX + (maxX - minX) * ((float)(xx) / width);

	//					for (int yy = 1; yy <= height; ++yy)
	//					{
	//						float minRangeY = minY + (maxY - minY) * ((float)(yy - 1) / height);
	//						float maxRangeY = minY + (maxY - minY) * ((float)(yy) / height);
	//						for (int zz = 1; zz <= depth; ++zz)
	//						{
	//							float minRangeZ = minZ + (maxZ - minZ) * ((float)(zz - 1) / depth);
	//							float maxRangeZ = minZ + (maxZ - minZ) * ((float)(zz) / depth);

	//							++nCurrCnt;

	//							for (int i = 0; i < tmp_db_pt.size(); ++i)
	//							{
	//								if ((minRangeX <= tmp_db_pt[i].x && tmp_db_pt[i].x <= maxRangeX)
	//									&& (minRangeY <= tmp_db_pt[i].y && tmp_db_pt[i].y <= maxRangeY)
	//									&& (minRangeZ <= tmp_db_pt[i].z && tmp_db_pt[i].z <= maxRangeZ))
	//								{
	//									tmp_db_pt[i].clusterID = nCurrCnt;
	//									//cube_points[x - 1][y - 1][z - 1].push_back(tmp_db_pt[i]);
	//								}
	//							}

	//							//하나의 복셀에 해당하는 육면체 정보..
	//							DB_Voxels tmp;
	//							tmp.planes[0].x[0] = maxRangeX; tmp.planes[0].y[0] = maxRangeY; tmp.planes[0].z[0] = maxRangeZ;
	//							tmp.planes[0].x[1] = maxRangeX; tmp.planes[0].y[1] = minRangeY; tmp.planes[0].z[1] = maxRangeZ;
	//							tmp.planes[0].x[2] = minRangeX; tmp.planes[0].y[2] = minRangeY; tmp.planes[0].z[2] = maxRangeZ;
	//							tmp.planes[0].x[3] = minRangeX; tmp.planes[0].y[3] = maxRangeY; tmp.planes[0].z[3] = maxRangeZ;
	//							plane_normVec(tmp.planes[0]);

	//							tmp.planes[1].x[0] = maxRangeX; tmp.planes[1].y[0] = maxRangeY; tmp.planes[1].z[0] = minRangeZ;
	//							tmp.planes[1].x[1] = maxRangeX; tmp.planes[1].y[1] = minRangeY; tmp.planes[1].z[1] = minRangeZ;
	//							tmp.planes[1].x[2] = maxRangeX; tmp.planes[1].y[2] = minRangeY; tmp.planes[1].z[2] = maxRangeZ;
	//							tmp.planes[1].x[3] = maxRangeX; tmp.planes[1].y[3] = maxRangeY; tmp.planes[1].z[3] = maxRangeZ;
	//							plane_normVec(tmp.planes[1]);

	//							tmp.planes[2].x[0] = minRangeX; tmp.planes[2].y[0] = maxRangeY; tmp.planes[2].z[0] = minRangeZ;
	//							tmp.planes[2].x[1] = minRangeX; tmp.planes[2].y[1] = minRangeY; tmp.planes[2].z[1] = minRangeZ;
	//							tmp.planes[2].x[2] = maxRangeX; tmp.planes[2].y[2] = minRangeY; tmp.planes[2].z[2] = minRangeZ;
	//							tmp.planes[2].x[3] = maxRangeX; tmp.planes[2].y[3] = maxRangeY; tmp.planes[2].z[3] = minRangeZ;
	//							plane_normVec(tmp.planes[2]);

	//							tmp.planes[3].x[0] = minRangeX; tmp.planes[3].y[0] = maxRangeY; tmp.planes[3].z[0] = maxRangeZ;
	//							tmp.planes[3].x[1] = minRangeX; tmp.planes[3].y[1] = minRangeY; tmp.planes[3].z[1] = maxRangeZ;
	//							tmp.planes[3].x[2] = minRangeX; tmp.planes[3].y[2] = minRangeY; tmp.planes[3].z[2] = minRangeZ;
	//							tmp.planes[3].x[3] = minRangeX; tmp.planes[3].y[3] = maxRangeY; tmp.planes[3].z[3] = minRangeZ;
	//							plane_normVec(tmp.planes[3]);

	//							tmp.planes[4].x[0] = maxRangeX; tmp.planes[4].y[0] = maxRangeY; tmp.planes[4].z[0] = minRangeZ;
	//							tmp.planes[4].x[1] = maxRangeX; tmp.planes[4].y[1] = maxRangeY; tmp.planes[4].z[1] = maxRangeZ;
	//							tmp.planes[4].x[2] = minRangeX; tmp.planes[4].y[2] = maxRangeY; tmp.planes[4].z[2] = maxRangeZ;
	//							tmp.planes[4].x[3] = minRangeX; tmp.planes[4].y[3] = maxRangeY; tmp.planes[4].z[3] = minRangeZ;
	//							plane_normVec(tmp.planes[4]);

	//							tmp.planes[5].x[0] = maxRangeX; tmp.planes[5].y[0] = minRangeY; tmp.planes[5].z[0] = maxRangeZ;
	//							tmp.planes[5].x[1] = maxRangeX; tmp.planes[5].y[1] = minRangeY; tmp.planes[5].z[1] = minRangeZ;
	//							tmp.planes[5].x[2] = minRangeX; tmp.planes[5].y[2] = minRangeY; tmp.planes[5].z[2] = minRangeZ;
	//							tmp.planes[5].x[3] = minRangeX; tmp.planes[5].y[3] = minRangeY; tmp.planes[5].z[3] = maxRangeZ;
	//							plane_normVec(tmp.planes[5]);
	//							tmpVoxels.push_back(tmp);
	//						}
	//					}
	//				}

	//				cube_points[x][y][z] = tmp_db_pt;


	//			}  //z
	//		}
	//	}
	//}


	points.clear();
	points.resize(refinedPoints.size());
	copy(refinedPoints.begin(), refinedPoints.end(), points.begin());

	nLabel = nCurrCnt;


	printf("finish!");

}


void H_VOXEL_DB::H_voxelFitting_allCnt_2(vector<H_Voxels>& points, vector<DB_Voxels>& voxels, int num_points, int color[][3], int &nLabel)
{
	int nPrevCnt = 0;
	int nCurrCnt = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<H_Voxels> refinedPoints;

	//처음은 voxel에 대한 라벨링이 안되어있으므로, 사이즈를 1만큼 준다.
	refinedPoints.clear();
	refinedPoints.resize(points.size());
	copy(points.begin(), points.end(), refinedPoints.begin());
	printf("refinedPoints:%d\n", refinedPoints.size());


	bool bShow = 1;
	int debug = 0;
	int _cubeSize = 15;  // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10
	//int _cubeSize = m_cubeSize;  // Aachen = 15, CMU=ppt 참고, 7Scene = 5, CAU = 5, Cambridge = 10

	int nAllCnt = 0;
	while (1)
	{
		float maxX = -90000000, maxY = -90000000, maxZ = -90000000;
		float minX = 90000000, minY = 90000000, minZ = 90000000;
		float sumX = 0, sumY = 0, sumZ = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
			{
				if (maxX < refinedPoints[i].vec_pt_3d_db[pt_id].x) maxX = refinedPoints[i].vec_pt_3d_db[pt_id].x;
				if (maxY < refinedPoints[i].vec_pt_3d_db[pt_id].y) maxY = refinedPoints[i].vec_pt_3d_db[pt_id].y;
				if (maxZ < refinedPoints[i].vec_pt_3d_db[pt_id].z) maxZ = refinedPoints[i].vec_pt_3d_db[pt_id].z;

				if (minX > refinedPoints[i].vec_pt_3d_db[pt_id].x) minX = refinedPoints[i].vec_pt_3d_db[pt_id].x;
				if (minY > refinedPoints[i].vec_pt_3d_db[pt_id].y) minY = refinedPoints[i].vec_pt_3d_db[pt_id].y;
				if (minZ > refinedPoints[i].vec_pt_3d_db[pt_id].z) minZ = refinedPoints[i].vec_pt_3d_db[pt_id].z;

				sumX += refinedPoints[i].vec_pt_3d_db[pt_id].x;
				sumY += refinedPoints[i].vec_pt_3d_db[pt_id].y;
				sumZ += refinedPoints[i].vec_pt_3d_db[pt_id].z;
			}
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
		//int width = _cubeSize; //x
		//int height = _cubeSize;  //y
		//int depth = _cubeSize;   //z
		if (width <= 0) width = 1;
		if (height <= 0) height = 1;
		if (depth <= 0) depth = 1;
		printf("width:%d, height:%d, depth:%d\n", width, height, depth);

		//vector<H_Voxels> ***cube_points = new vector<H_Voxels> **[width];
		//for (int x = 0; x < width; ++x)
		//{
		//	cube_points[x] = new vector<H_Voxels> *[height];
		//	for (int y = 0; y < height; ++y)
		//	{
		//		cube_points[x][y] = new vector<H_Voxels>[depth];
		//	}
		//}

		//vector<H_Voxels> cube_points;

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

		int nLabel = 0;
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

					++nLabel;

					for (int i = 0; i < refinedPoints.size(); ++i)
					{
						for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
						{
							if ((minRangeX <= refinedPoints[i].vec_pt_3d_db[pt_id].x && refinedPoints[i].vec_pt_3d_db[pt_id].x <= maxRangeX)
								&& (minRangeY <= refinedPoints[i].vec_pt_3d_db[pt_id].y && refinedPoints[i].vec_pt_3d_db[pt_id].y <= maxRangeY)
								&& (minRangeZ <= refinedPoints[i].vec_pt_3d_db[pt_id].z && refinedPoints[i].vec_pt_3d_db[pt_id].z <= maxRangeZ))
							{
								cube_points[x - 1][y - 1][z - 1].push_back(refinedPoints[i].vec_pt_3d_db[pt_id]);
							}

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

		int nPts_sum = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			nPts_sum += refinedPoints[i].vec_pt_3d_db.size();
		}

		int threshold = nPts_sum * 0.0001;   // Aachen = 0.0001;
		printf("11 refinedPoints:%d\n\n", nPts_sum);

		printf("threshold:%d\n", threshold);
		nCurrCnt = 0;
		refinedPoints.clear();
		int voxelCnt = 0;
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

						for (int i = 0; i < cube_points[x][y][z].size(); ++i)
						{
							cube_points[x][y][z][i].clusterID = nCurrCnt;
							//refinedPoints.push_back(cube_points[x][y][z][i]);

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

						H_Voxels tmp;
						tmp.nLabel = nCurrCnt;
						tmp.nLevel = 0;
						tmp.vec_pt_3d_db = cube_points[x][y][z];
						refinedPoints.push_back(tmp);
						refinedPoints[nCurrCnt - 1].planes = tmpVoxels[voxelCnt];
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

		int nPts_sum2 = 0;
		for (int i = 0; i < refinedPoints.size(); ++i)
		{
			nPts_sum2 += refinedPoints[i].vec_pt_3d_db.size();
		}

		printf("nPrevCnt:%d, nCurrCnt:%d\n", nPrevCnt, nCurrCnt);
		printf("22 refinedPoints:%d\n\n", nPts_sum2);


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

		if (nAllCnt > 2) break;   // CMU, Aachen => 2, cambridge = 1
		++nAllCnt;

	}  //while

	if (debug)
	{
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
		for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
		{
			if (maxX < refinedPoints[i].vec_pt_3d_db[pt_id].x) maxX = refinedPoints[i].vec_pt_3d_db[pt_id].x;
			if (maxY < refinedPoints[i].vec_pt_3d_db[pt_id].y) maxY = refinedPoints[i].vec_pt_3d_db[pt_id].y;
			if (maxZ < refinedPoints[i].vec_pt_3d_db[pt_id].z) maxZ = refinedPoints[i].vec_pt_3d_db[pt_id].z;
									   												
			if (minX > refinedPoints[i].vec_pt_3d_db[pt_id].x) minX = refinedPoints[i].vec_pt_3d_db[pt_id].x;
			if (minY > refinedPoints[i].vec_pt_3d_db[pt_id].y) minY = refinedPoints[i].vec_pt_3d_db[pt_id].y;
			if (minZ > refinedPoints[i].vec_pt_3d_db[pt_id].z) minZ = refinedPoints[i].vec_pt_3d_db[pt_id].z;
		}

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
					for (int pt_id = 0; pt_id < refinedPoints[i].vec_pt_3d_db.size(); ++pt_id)
					{
						DB_Point_3D tmp = refinedPoints[i].vec_pt_3d_db[pt_id];
						if ((minRangeX <= tmp.x && tmp.x <= maxRangeX)
							&& (minRangeY <= tmp.y && tmp.y <= maxRangeY)
							&& (minRangeZ <= tmp.z && tmp.z <= maxRangeZ))
						{
							cube_points[x - 1][y - 1][z - 1].push_back(tmp);
						}
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

					H_Voxels tmp;
					tmp.nLabel = nCurrCnt;
					tmp.nLevel = 0;
					tmp.vec_pt_3d_db = cube_points[x][y][z];
					refinedPoints.push_back(tmp);
					refinedPoints[nCurrCnt - 1].planes = tmpVoxels[voxelCnt];
				}
				++voxelCnt;

			}
		}
	}



	//graph 만들기
	nLabel = 0;
	int kernelSize = 2;
	for (int x = 0; x < width; ++x)
	{
		for (int y = 0; y < height; ++y)
		{
			for (int z = 0; z < depth; ++z)
			{
				int size = cube_points[x][y][z].size();
				if (size > threshold)
				{
					++nLabel;
					for (int xx = x - kernelSize; xx <= x + kernelSize; ++xx)
					{
						if (xx < 0 || xx > width - 1) continue;
						for (int yy = y - kernelSize; yy <= y + kernelSize; ++yy)
						{
							if (yy < 0 || yy > height - 1) continue;
							for (int zz = z - kernelSize; zz <= z + kernelSize; ++zz)
							{
								if (zz < 0 || zz > depth - 1) continue;
								if (xx == x && yy == y && zz == z)
									continue;

								//있을때만 추가.
								if (cube_points[xx][yy][zz].size() > threshold)
									refinedPoints[nLabel - 1].graph_vox.node.push_back(cube_points[xx][yy][zz][0].clusterID);
							}
						}
					}

				}

			} //z
		} //y
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