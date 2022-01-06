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

//#include "experiment2_ver2_Aachen.h"
//#include "experiment3_Aachen.h"

//#include "experiment3_ver5_Aachen.h"
//#include "experiment3_ver5_Aachen_Hirer.h"


//#include "experiment5_Aachen_H_Test.h"
//#include "experiment5_Aachen_H_Test_2.h"    
#include "experiment5_Aachen_H_Test_3.h"  //Test중..


int main(void)
{

//////////////////////////////////////////////////////
	const char *input_filename = "F:/Aachen-Day-Night dataset/3D-models/aachen_cvpr2018_db.nvm";            //first argument must be filename

	//string filePath = "F:/Aachen-Day-Night dataset/hloc_database/images.bin";
	//FILE *in;
	//int ch;
	//if ((in = fopen("F:/Aachen-Day-Night dataset/hloc_database/images.bin", "rb")) == NULL) {
	//	fputs("파일 열기 에러!", stderr);
	//	exit(1);
	//}
	//fclose(in); // 파일 닫기

	//unsigned char **_data; int *datalen;
	//std::ifstream readFile(filePath, std::ifstream::binary);
	//if (readFile.is_open())    //파일이 열렸는지 확인
	//{
	//	while (!readFile.eof())    //파일 끝까지 읽었는지 확인
	//	{
	//		string token;
	//		char arr[256];
	//		readFile.getline(arr, 256);    //한줄씩 읽어오기
	//		int debug = 0;
	//	}
	//}


	int cubeSize = 5;  //50

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
	bool test = LoadModelFile(input_filename, camera_data, point_data, measurements, ptidx, camidx, photo_names, point_color, cubeSize);



	return 0;
}