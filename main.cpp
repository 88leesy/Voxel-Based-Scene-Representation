
#include <string.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include <io.h>



using namespace std;

#include "DataInterface.h"
//#include "util.h"


//#include "experiment2_ver2.h"
//#include "experiment2_ver3.h"
//#include "experiment3.h"
//#include "experiment3_ver2.h"
//#include "experiment3_ver3.h"
//#include "experiment3_ver4.h"  //2d 포인트에서 3차원 공간으로 레이를 쏴서, 3d 평면에 맺히는 점을 찾아 2d-3d correspodence 만드는 실험..
#include "experiment3_ver5.h"
//#include "experimental_ver6_octree.h"

//experiment3_ver3는 큐브 내에 있는 3d 포인트 들의 min,max x,y,z 의 꼭지점을 구해서 그것을 전부 사영 시켜보는 실험임.


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


		const char *input_filename="";

		//int cubeSize = 40;
		//if (id == 0)
		//{
		//	cubeSize = 40;
		//	input_filename="[ShopFacade] reconstruction.nvm";            //first argument must be filename
		//}
		//else if (id == 1)
		//{
		//	cubeSize = 30;
		//	input_filename = "[OldHospital] reconstruction.nvm";
		//}
		//else if (id == 2)
		//{
		//	cubeSize = 100;
		//	input_filename = "[KingsCollege] reconstruction.nvm";
		//}
		//else if (id == 3)
		//{
		//	cubeSize = 100;
		//	input_filename = "[StMarysChurch] reconstruction.nvm";
		//}

		int cubeSize = 20;
		if (id == 0)
		{
			cubeSize = 100;
			input_filename = "E:\[ShopFacade] reconstruction.nvm";            //first argument must be filename
		}
		else if (id == 1)
		{
			cubeSize = 15;
			input_filename = "E:\[OldHospital] reconstruction.nvm";
		}
		else if (id == 2)
		{
			cubeSize = 50;
			input_filename = "E:\[KingsCollege] reconstruction.nvm";
		}
		else if (id == 3)
		{
			cubeSize = 50;
			input_filename = "[StMarysChurch] reconstruction.nvm";
		}


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

//int main(void)
//{
//	/*
//KingsCollege, OldHospital, ShopFacade, StMarysChurch, Street
//*/
//	// ShopFacade_cube20, KingsCollege_cube50, OldHospital_cube15, StMarysChurch_cube50
//	// ShopFacade_cube40, KingsCollege_cube100, OldHospital_cube30, StMarysChurch_cube100
//	//////////////////////////////////////////////////////
//	const char *input_filename = "[ShopFacade] reconstruction.nvm";            //first argument must be filename
//	//const char *input_filename = "test.nvm";            //first argument must be filename
//	string data_name = "ShopFacade/";
//
//
//	/////////////////////////////////////////////////////////////////
//	//CameraT, Point3D, Point2D are defined in src/pba/DataInterface.h
//	vector<CameraT>        camera_data;    //camera (input/ouput)
//	vector<Point3D>        point_data;     //3D point(iput/output)
//	vector<Point2D>        measurements;   //measurment/projection vector
//	vector<int>            camidx, ptidx;  //index of camera/point for each projection
//
//	/////////////////////////////////////
//	vector<string>         photo_names;        //from NVM file, not used in bundle adjustment
//	vector<int>            point_color;        //from NVM file, not used in bundle adjustment
//
//
//	/////////////////////////////////////////////////////////////
//	///load the data. You will need to replace the loader for your own data
//	bool test = LoadModelFile(input_filename, data_name, camera_data, point_data, measurements, ptidx, camidx, photo_names, point_color);
//
//	return 0;
//}



//std::vector<std::string> get_files_inDirectory(const std::string& _path, const std::string& _filter)
//{
//	std::string searching = _path + _filter;
//
//	std::vector<std::string> return_;
//
//	_finddata_t fd;
//	long handle = _findfirst(searching.c_str(), &fd);  //현재 폴더 내 모든 파일을 찾는다.
//
//	if (handle == -1)    return return_;
//
//	int result = 0;
//	do
//	{
//		return_.push_back(fd.name);
//		result = _findnext(handle, &fd);
//	} while (result != -1);
//
//	_findclose(handle);
//
//	return return_;
//}
//
//
//int main(void)
//{
//	/*
//chess, heads, fire, office, redkitchen, stairs, pumpkin
//*/
//
////////////////////////////////////////////////////////
//	string input_filedir = "F:/RGB-D dataset/";            //first argument must be filename
//	string data_dir = "chess/seq-01/";
//
//	string path = input_filedir + data_dir;
//	const std::string& _filter = "*.*";     //ex) *.exe or *.*
//	std::vector<std::string> test;
//	
//	test = get_files_inDirectory(path, _filter);
//
//
//
//
//
//
//	return 0;
//}