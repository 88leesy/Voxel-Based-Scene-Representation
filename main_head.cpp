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

//#include "experiment2_ver2_7scene.h"
#include "experiment3_7scene.h"
//#include "TESTEST.h"

int main(void)
{

	//////////////////////////////////////////////////////
	string input_dir = "F:/_RGBD_Dataset7_Scenes/";

	int cubeSize = 20;   // fire1 = 20, fire2=15 (initial cube=5, threshold:0.001), heads =  10,  heads all = 15

	string data_name = "fire/ALL/";             //heads/sequence2/, fire/sequence3/
	string input_filename = input_dir + data_name + "all.nvm";  // [head] sequence2, [fire] sequence3

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