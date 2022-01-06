
#include <string.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include <io.h>



using namespace std;

//#include "DataInterface.h"
//#include "experiment2_HierarchyVer.h"  //pair
//#include "experiment3_HierarchyVer.h"   // single
#include "experiment3_HierarchyVer2.h"

int main(void)
{
	/*
KingsCollege, OldHospital, ShopFacade, StMarysChurch, Street
*/
// ShopFacade_cube20, KingsCollege_cube50, OldHospital_cube15, StMarysChurch_cube50
// ShopFacade_cube40, KingsCollege_cube100, OldHospital_cube30, StMarysChurch_cube100
//////////////////////////////////////////////////////
	//const char *input_filename = "[ShopFacade] reconstruction.nvm";            //first argument must be filename
	//const char *input_filename = "test.nvm";            //first argument must be filename

	vector<string> data_names{ "ShopFacade/", "OldHospital/", "KingsCollege/",  "StMarysChurch/" };
	//vector<string> data_names{  "OldHospital/" };

	for (int id = 0; id < data_names.size(); ++id)   // id=1 부터 시작하는건, shop 40 완성 했으니깐.
	{
		string data_name = data_names[id];


		const char *input_filename = "";

		int cubeSize = 4;
		if (id == 0)
		{
			cubeSize = 4;
			input_filename = "[ShopFacade] reconstruction.nvm";            //first argument must be filename
		}
		else if (id == 1)
		{
			cubeSize = 4;
			input_filename = "[OldHospital] reconstruction.nvm";
		}
		else if (id == 2)
		{
			cubeSize = 4;
			input_filename = "[KingsCollege] reconstruction.nvm";
		}
		else if (id == 3)
		{
			cubeSize = 4;
			input_filename = "[StMarysChurch] reconstruction.nvm";
		}

		//string filename = data_name + "seq3/frame00008.png";
		//string img_path = "F:/RPNet_test/data/absolute_cambridge/" + filename;

		////float reSize_H = 1080, reSize_W = 1920;
		//float reSize_H = 256, reSize_W = 455;
		//Mat img = imread(img_path);
		//Mat down_img = img.clone();
		//Mat down_img2 = img.clone();
		//resize(down_img, down_img, Size(reSize_W, reSize_H));  //455, 256  
		//resize(down_img2, down_img2, Size(reSize_W, reSize_H), 0,0, INTER_AREA);  //455, 256  
		////resize(down_img2, down_img2, Size(0, 0), 0.237, 0.237 , INTER_AREA);  //455, 256  



		//filename = data_name + "seq3/frame00008.png";
		//img_path = "F:/_voxelFeatureMap9_single/" + filename;
		//Mat lb_img = imread(img_path, IMREAD_ANYDEPTH);
		//Mat lb_down_img = lb_img.clone();
		//Mat lb_down_img2 = lb_img.clone();
		//resize(lb_down_img, lb_down_img, Size(reSize_W, reSize_H));  //455, 256  
		//resize(lb_down_img2, lb_down_img2, Size(reSize_W, reSize_H), 0, 0, INTER_AREA);  //455, 256  

		//filename = data_name + "seq3/kp_frame00008.png";
		//img_path = "F:/_voxelFeatureMap9_single/" + filename;
		//Mat kp_img = imread(img_path, IMREAD_ANYDEPTH);
		//Mat kp_down_img = kp_img.clone();
		//Mat kp_down_img2 = kp_img.clone();
		//resize(kp_down_img, kp_down_img, Size(reSize_W, reSize_H));  //455, 256  
		//resize(kp_down_img2, kp_down_img2, Size(reSize_W, reSize_H), 0, 0, INTER_AREA);  //455, 256  


		/////////////////////////////////////////////////////////////////
		//CameraT, Point3D, Point2D are defined in src/pba/DataInterface.h
		vector<CameraT>        camera_data;    //camera (input/ouput)
		vector<Point3D>        point_data;     //3D point(iput/output)
		vector<Point2D>        measurements;   //measurment/projection vector
		vector<int>            camidx, ptidx;  //index of camera/point for each projection

		/////////////////////////////////////
		vector<string>         photo_names;        //from NVM file, not used in bundle adjustment
		vector<int>            point_color;        //from NVM file, not used in bundle adjustment


		/////////////////////////////////////////////////////////////
		///load the data. You will need to replace the loader for your own data

		bool test = LoadModelFile(input_filename, data_name, camera_data, point_data, measurements, ptidx, camidx, photo_names, point_color, cubeSize);
	}



	return 0;
}
