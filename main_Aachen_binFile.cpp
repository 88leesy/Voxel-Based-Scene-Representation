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
//#include "experiment6_Aachen_H_Test(txt)_2.h"  //Test중..
#include "experiment3_ver7_Aachen.h"  //Test중..

int main(void)
{

	//////////////////////////////////////////////////////
	
	string image_filePath = "F:/Aachen-Day-Night dataset/hloc_database/images.txt";
	string camera_filePath = "F:/Aachen-Day-Night dataset/hloc_database/cameras.txt";
	string points3D_filePath = "F:/Aachen-Day-Night dataset/hloc_database/points3D.txt";
	
	int cubeSize = 35;  //3, 5   , 50-1899, 40-1151, 35-896 , 30-680??정도...
	bool test = LoadModelFile(image_filePath, camera_filePath, points3D_filePath, cubeSize);


	return 0;
}