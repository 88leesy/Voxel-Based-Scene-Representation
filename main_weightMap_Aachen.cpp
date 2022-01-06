
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

#include "SY_Labeling.h"


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

int main(void)
{

	//////////////////////////////////////////////////////
	const char *input_filename = "F:/Aachen-Day-Night dataset/3D-models/aachen_cvpr2018_db.nvm";


	if (input_filename == NULL)return false;
	ifstream in(input_filename);

	std::cout << "Loading cameras/points: " << input_filename << "\n";
	if (!in.is_open()) return false;

	if (strstr(input_filename, ".nvm"))
	{
		int rotation_parameter_num = 4;
		bool format_r9t = false;
		string token;
		vector<string> names;
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
		names.resize(ncam);
		float radial_distortion = 0;
		for (int i = 0; i < ncam; ++i)
		{
			double f, q[9], c[3], d[2];
			in >> token >> f;
			for (int j = 0; j < rotation_parameter_num; ++j) in >> q[j];
			in >> c[0] >> c[1] >> c[2] >> d[0] >> d[1];

			names[i] = token;
			radial_distortion = d[0];
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
		absPath = "F:/_voxelFeatureMap_Aachen2/_single_Voxel_set/";

		for (int i = 0; i < ncam; ++i)
		{
			string img_path = "F:/Aachen-Day-Night dataset/images/images_upright/" + names[i];
			cout << "camId: " << i << ",  " << names[i] << endl;

			vector<string> line_str = split(names[i], '.');
			vector<string> img_path_split = split(line_str[0], '_');
			string lb_path = absPath + img_path_split[0] + "_label" + ".png";
			string contour_path = absPath + img_path_split[0] + "_contour" + ".png";
			string fillcontour_path = absPath + img_path_split[0] + "_fillcontour" + ".png";

			string weightMap_path = absPath + img_path_split[0] + "_weightmap_" + ".png";

			Mat lb_img = imread(lb_path, IMREAD_ANYDEPTH);
			Mat contour_img = imread(contour_path, 0);
			//imshow("lb_img", lb_img);
			//waitKey(0);

			int TotalNumberOfPixels = lb_img.rows*lb_img.cols;
			int zeroPixels = TotalNumberOfPixels - countNonZero(lb_img);
			printf("None voxelLabel! : %d, %d\n", zeroPixels, TotalNumberOfPixels);
			if (zeroPixels == TotalNumberOfPixels)
			{
				continue;
			}

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

		}
	}
}





