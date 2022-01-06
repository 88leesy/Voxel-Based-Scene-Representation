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

//#include "experiment2_ver2_CMU.h"
//#include "experiment3_CMU.h"

//#include "TESTEST.h"

#include "experiment3_CMU_ver5.h"

int main(void)
{

	//////////////////////////////////////////////////////
	string input_dir = "F:/CMU-Seasons/3D-models/nvm_models/";            //first argument must be filename

	int cubeSize = 100;   // 파라미터 ppt 확인하고 다시 설정 하고 돌리자!
	for (int id = 16; id < 17; ++id)   // 7부터..해야함
	{
		int num = id+2;
		if (id >= 9)                              
		{
			num = id + 8;
			if (id >= 15)
				num = id + 9;
		}
			
		string data_name = "slice" + to_string(num) + '/';
		string input_filename = input_dir + "slice" + to_string(num) + ".nvm";

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

	}


	



	return 0;
}