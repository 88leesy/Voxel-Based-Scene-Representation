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

//#include "experiment2_ver2_ChungAngUni.h"
#include "experiment3_ChungAngUni.h"

int main(void)
{

	//////////////////////////////////////////////////////
	string input_dir = "F:/_ChungAngUniv/";            

	int cubeSize = 20;   // ArtCenter = 20 , inisfri, 310_small, 101

	string data_name = "101/";             //
	string input_filename = input_dir + data_name + "ALL/all.nvm";  // [310_small] all

	/////////////////////////////////////////////////////////////////
	//CameraT, Point3D, Point2D are defined in src/pba/DataInterface.h
	vector<CameraT>        camera_data;    //camera (input/ouput)
	vector<Point3D>        point_data;     //3D point(iput/output)
	vector<Point2D>        measurements;   //measurment/projection vector
	vector<int>            camidx, ptidx;  //index of camera/point for each projection

	/////////////////////////////////////
	vector<string>         photo_names;        //from NVM file, not used in bundle adjustment
	vector<int>            point_color;        //from NVM file, not used in bundle adjustment


	bool test = LoadModelFile(input_filename, data_name, camera_data, point_data, measurements, ptidx, camidx, photo_names, point_color, cubeSize);


	return 0;
}