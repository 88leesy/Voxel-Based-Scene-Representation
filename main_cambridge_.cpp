#include <string.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <io.h>



using namespace std;

//#include "DataInterface.h"
#include "experiment6_Cambridge_H_Test(txt)_2.h"  //Test중..


int main(void)
{

	//////////////////////////////////////////////////////

	vector<string> data_names{ "ShopFacade/", "OldHospital/", "KingsCollege/",  "StMarysChurch/", "GreatCourt/" };

	for (int id = 0; id < data_names.size(); ++id)   // id=1 부터 시작하는건, shop 40 완성 했으니깐.
	{
		string data_name = data_names[id];	
		string str1("F:/Cambridge_SP+SG_sfmDatasets/");
		string str2("sfm_superpoint+superglue/images.txt");

		string image_filePath = str1 + data_name + str2;
		string camera_filePath = str1 + data_name + "sfm_superpoint+superglue/cameras.txt";
		string points3D_filePath = str1 + data_name + "sfm_superpoint+superglue/points3D.txt";

		int cubeSize = 3;  //50, 5
		bool test = LoadModelFile(image_filePath, camera_filePath, points3D_filePath, data_name, cubeSize);
	}





	return 0;
}