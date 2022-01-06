#include "PlaneRANSAC.h"
#include <time.h>

void PlaneRANSAC::calcPlaneCoefficient(DB_Point p1, DB_Point p2, DB_Point p3, Plane &result) {

	// 	result.equation.a = p1.y * (p2.z - p3.z) + p2.y * (p3.z - p1.z) + p3.y * (p1.z - p2.z);
	// 	result.equation.b = p1.z * (p2.x - p3.x) + p2.z * (p3.x - p1.x) + p3.z * (p1.x - p2.x);
	// 	result.equation.c = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
	// 	result.equation.d = (-1) * (	p1.x * (p2.y * p3.z - p3.y * p2.z) + 	p2.x * (p3.y * p1.z - p1.y * p3.z) + 	p3.x * (p1.y * p2.z - p2.y * p1.z));

	double x1 = p2.x - p1.x;
	double y1 = p2.y - p1.y;
	double z1 = p2.z - p1.z;

	double x2 = p3.x - p1.x;
	double y2 = p3.y - p1.y;
	double z2 = p3.z - p1.z;

	//surface noramal vector
	double x3 = y1 * z2 - z1 * y2;
	double y3 = z1 * x2 - x1 * z2;
	double z3 = x1 * y2 - y1 * x2;
	result.surfaceNomalX = x3;
	result.surfaceNomalY = y3;
	result.surfaceNomalZ = z3;

	double l = sqrt(x3*x3 + y3 * y3 + z3 * z3);

	result.equation.a = (x3) / (x3*(-p1.x) + y3 * (-p1.y) + z3 * (-p1.z));
	result.equation.b = (y3) / (x3*(-p1.x) + y3 * (-p1.y) + z3 * (-p1.z));
	result.equation.c = (z3) / (x3*(-p1.x) + y3 * (-p1.y) + z3 * (-p1.z));


	//nomalized vector
 	result.equation.a = x3/l;
 	result.equation.b = y3/l;
 	result.equation.c = z3/l;
	result.equation.d = -(result.equation.a*p1.x + result.equation.b*p1.y + result.equation.c*p1.z);
}

float PlaneRANSAC::calcDistance(Plane &phi, DB_Point p) {
	return abs(phi.equation.a * p.x + phi.equation.b * p.y + phi.equation.c * p.z + phi.equation.d) / sqrt(phi.equation.a * phi.equation.a + phi.equation.b * phi.equation.b + phi.equation.c * phi.equation.c);
}

//void PlaneRANSAC::run(vector<DB_Point>& vec_originPointCloudData, vector<DB_Point>& vec_superpixelXYZ, vector<Plane> &bestInlierPlane, int &planeLabel)
void PlaneRANSAC::run(vector<DB_Point>& vec_originPointCloudData, vector<Plane> &bestInlierPlane, int &planeLabel)
{
	/*
	int superCount = 0;
	int totalSize = 0, inputSize = 0, prevtotal_inputsize = 0;

	vector<int> vec_reductionSuperpixelXYZIdx;
	vector<int> vec_reductionPointCloudDataIdx;

	for (int i = 0; i < vec_originPointCloudData.size(); i++)
	{
		vec_originPointCloudData[i].clusterID = -1;
	}

	planeLabel = 1;

	for (int i = 0; i < vec_superpixelXYZ.size(); i++)
	{
		totalSize += vec_superpixelXYZ[i].size();
	}

	for (int i = 0; i < vec_originPointCloudData.size(); i++)
		vec_reductionPointCloudDataIdx.push_back(i);

	for (int superCount = 0; superCount < vec_superpixelXYZ.size(); superCount++)
	{
		if (superCount > 0)
			prevtotal_inputsize += vec_superpixelXYZ[superCount - 1].size();
		inputSize = vec_superpixelXYZ[superCount].size();

		int iterCntPlane = 0;
		int maxIterCntPlane = m_maxPlanes;
		int iterCnt = 0;

		int maxIterCnt = m_maxIteration;
		maxIterCnt = (int)(1 + log(1. - 0.99) / log(1. - pow(0.5, 3)));

		vector<int> vec_bestInlierOriginPointCloudDataIdx;
		vector<int> vec_bestReductiontPointCloudDataIdx;
		vector<int> vec_bestInlierIdx;
		Plane tempBestInlierPlane;
		DB_Point tempBestInlier3DPoints;

		float InlierAvrDist = 0;
		float lnlierDist = 0;
		float outlierDist = 0;
		int outlierCnt = 0;
		float InlierRatio = 0;

		iterCnt = 0;
		while (iterCnt < maxIterCnt)
		{
			// inlier에 대한 평면, Point Cloud Data
			Plane tempPlane;
			vector<int> vec_tempReductionPointCloudDataIdx;
			vector<int> vec_tempInlierIdx;

			// 랜덤한 점 3개 추출하기
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			XYZ srcPoint[3];
			int srcPointIdx[3];

			// 입력 Point Cloud Data에서 중복되지 않는 점 3개를 선택한다.
			srcPointIdx[0] = rand() % inputSize;
			srcPoint[0] = vec_superpixelXYZ[superCount][srcPointIdx[0]];

			while (true) {
				srcPointIdx[1] = rand() % inputSize;
				if (srcPointIdx[0] == srcPointIdx[1]) continue;
				else break;
			}
			while (true) {
				srcPointIdx[2] = rand() % inputSize;
				if (srcPointIdx[0] == srcPointIdx[2] || srcPointIdx[1] == srcPointIdx[2]) continue;
				else break;
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



			// 점 3개를 통해 평면의 방정식을 생성한다.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			Plane phi;
			DB_Point p1 = vec_superpixelXYZ[superCount][srcPointIdx[0]].getXYZ();
			DB_Point p2 = vec_superpixelXYZ[superCount][srcPointIdx[1]].getXYZ();
			DB_Point p3 = vec_superpixelXYZ[superCount][srcPointIdx[2]].getXYZ();

			calcPlaneCoefficient(p1, p2, p3, phi);
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

			// 임의의 3점으로 계산된 평면과 나머지 Point Cloud Point들간의 거리를 구하여
			// threshold보다 작은 경우 inlier로 판단한다.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

			//<<<<<<<<<<<<<<<<<<<<<<
			float threshold = m_distanceThreshold;

			//method3
			float tempinlierDist = 0;
			float tempoutlierDist = 0;
			int tempoutliercnt = 0;
			for (int i = 0; i < inputSize; i++)
			{
				if (threshold > calcDistance(phi, vec_superpixelXYZ[superCount][i].getXYZ()))
				{
					vec_tempReductionPointCloudDataIdx.push_back(vec_reductionPointCloudDataIdx[vec_superpixelXYZ[superCount][i].idx]);

					// 	//inlier ratio 구하기 위한 것들.
					// 	vec_tempInlierIdx.push_back(i);
					// 	tempinlierDist += calcDistance(phi,vec_superpixelXYZ[superCount][i].getXYZ());
				}
				else
				{
					// 	//outlier ration 구하기 위한 것들.(oulier 거리)
					// 	tempoutlierDist += calcDistance(phi,vec_superpixelXYZ[superCount][i].getXYZ());
					//  tempoutliercnt++;
				}
			}


			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


			// iteration 만큼 반복되는 프로세스 중 가장 inlier가 많은 경우의 평면을 찾는다.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

			if (vec_tempReductionPointCloudDataIdx.size() > m_minInlier)
			{
				if (vec_tempReductionPointCloudDataIdx.size() > vec_bestInlierOriginPointCloudDataIdx.size())
				{
					// 벡터 복사
					vec_bestInlierOriginPointCloudDataIdx.clear();
					vec_bestInlierOriginPointCloudDataIdx.resize(vec_tempReductionPointCloudDataIdx.size());
					copy(vec_tempReductionPointCloudDataIdx.begin(), vec_tempReductionPointCloudDataIdx.end(), vec_bestInlierOriginPointCloudDataIdx.begin());

					// 베스트 평면 할당
					tempBestInlierPlane = phi;

					// 					//인라이어, 아웃라이어 거리
					// 					lnlierDist = tempinlierDist;
					// 					outlierDist = tempoutlierDist;
					// 					outlierCnt = tempoutliercnt;
					// 					tempBestInlier3DPoints = p1;
				}
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			iterCnt++;
		}


		// 		//가. 각 평면에 대한 lnlier의 비율
		// 		InlierRatio = (float)vec_bestInlierOriginPointCloudDataIdx.size()/(float)inputSize;
		// 
		// 		//나. 각 평면에 대한 inlier당 평균거리/outlier당 평균거리
		// 		InlierAvrDist = lnlierDist/vec_bestInlierOriginPointCloudDataIdx.size();
		// 		outlierDist = outlierDist/outlierCnt;
		// 
		// 		//가. 나. 저장
		// 		FILE *in, *in2, *in3;
		// 		in = fopen("InlierRatio.txt", "a");
		// 		fprintf(in, "%f\n", InlierRatio);
		// 		in2 = fopen("InlierAvrDist.txt", "a");
		// 		fprintf(in2, "%f\n", InlierAvrDist);
		// 		in3 = fopen("outlierDist.txt", "a");
		// 		fprintf(in3, "%f\n", outlierDist);
		// 		fclose(in);
		// 		fclose(in2);
		// 		fclose(in3);


				// 베스트 평면 할당
		bestInlierPlane.push_back(tempBestInlierPlane);

		// 원본 Point Cloud Data에서 평면을 라벨링한다.
		for (int i = vec_bestInlierOriginPointCloudDataIdx.size() - 1; i >= 0; i--)
			vec_originPointCloudData[vec_bestInlierOriginPointCloudDataIdx[i]].clusterID = planeLabel;

		cout << "best inlier cnt: " << vec_bestInlierOriginPointCloudDataIdx.size() << endl;
		cout << "bestInlierPlane: " << planeLabel << ":" << bestInlierPlane.size() << endl;

		planeLabel++;

	}
	*/


	

	// 배열로된 pointCloudData를 벡터 형식으로 변경
	vector<DB_Point> vec_reductionPointCloudData;
	for(int i=0; i < vec_originPointCloudData.size(); i++)
		vec_reductionPointCloudData.push_back(vec_originPointCloudData[i]);

	int totalSize, inputSize;
	totalSize = inputSize = vec_reductionPointCloudData.size();

	// Point Cloud Data의 인덱스도 관리한다.
	vector<int> vec_reductionPointCloudDataIdx;
	for(int i=0; i<vec_reductionPointCloudData.size(); i++)
		vec_reductionPointCloudDataIdx.push_back(i);


	int iterCntPlane = 0;
	int maxIterCntPlane = m_maxPlanes;
	int iterCnt = 0;

	int maxIterCnt = m_maxIteration;
	//int maxIterCnt = (int)(1 + log(1. - 0.99)/log(1. - pow(0.5, 3)));

	planeLabel = 1;

	// 찾은 평면이 최대 평면의 개수를 초과하는 경우 종료한다.
	while(inputSize > totalSize * m_remainPointsRatio && iterCntPlane < maxIterCntPlane){
		vector<int> vec_bestInlierOriginPointCloudDataIdx;
		vector<int> vec_bestReductiontPointCloudDataIdx;
		Plane tempBestInlierPlane;

		iterCnt = 0;
		while(iterCnt < maxIterCnt)
		{
			// inlier에 대한 평면, Point Cloud Data
			Plane tempPlane;
			vector<int> vec_tempReductionPointCloudDataIdx;
			vector<int> vec_tempReductionPointCloudDataIdxOfIdx;

			// 랜덤한 점 3개 추출하기
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			DB_Point srcPoint[3];
			int srcPointIdx[3];

			////////////////
			// seed를 고정시켜주어 입력 Point Cloud Data가 새로 들어와도
			// seed값이 0으로 고정되어 0으로 초기화 된 상태에서 rand()가 초기화 되기 때문에
			// 각 이미지에 대해 seed에서의 평면의 방정식이 바뀔일이 없다.
			srand(0);

			// 입력 Point Cloud Data에서 중복되지 않는 점 3개를 선택한다.
			srcPointIdx[0]= rand() % inputSize;
			srcPoint[0] = vec_reductionPointCloudData[srcPointIdx[0]];

			while(true){
				srcPointIdx[1]= rand() % inputSize;
				if(srcPointIdx[0] == srcPointIdx[1]) continue;
				else break;
			}
			while(true){
				srcPointIdx[2]= rand() % inputSize;
				if(srcPointIdx[0] == srcPointIdx[2] || srcPointIdx[1] == srcPointIdx[2]) continue;
				else break;
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



			// 점 3개를 통해 평면의 방정식을 생성한다.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			Plane phi;
			DB_Point p1 = vec_reductionPointCloudData[srcPointIdx[0]].getXYZ();
			DB_Point p2 = vec_reductionPointCloudData[srcPointIdx[1]].getXYZ();
			DB_Point p3 = vec_reductionPointCloudData[srcPointIdx[2]].getXYZ();

			calcPlaneCoefficient(p1,p2,p3,phi);
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

			// 임의의 3점으로 계산된 평면과 나머지 Point Cloud Point들간의 거리를 구하여
			// threshold보다 작은 경우 inlier로 판단한다.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

			//<<<<<<<<<<<<<<<<<<<<<<
			float threshold = m_distanceThreshold;
			for(int i=0; i<inputSize; i++){
				if( threshold > calcDistance(phi,vec_reductionPointCloudData[i].getXYZ()) ){
					vec_tempReductionPointCloudDataIdx.push_back( vec_reductionPointCloudDataIdx[i] );
					vec_tempReductionPointCloudDataIdxOfIdx.push_back(i);
				}
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


			// iteration 만큼 반복되는 프로세스 중 가장 inlier가 많은 경우의 평면을 찾는다.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

			if(vec_tempReductionPointCloudDataIdx.size() > m_minInlier )
			{
				if( vec_tempReductionPointCloudDataIdx.size() > vec_bestInlierOriginPointCloudDataIdx.size() )
				{
					// 벡터 복사
					vec_bestInlierOriginPointCloudDataIdx.clear();
					vec_bestInlierOriginPointCloudDataIdx.resize(vec_tempReductionPointCloudDataIdx.size());
					copy(vec_tempReductionPointCloudDataIdx.begin(), vec_tempReductionPointCloudDataIdx.end(), vec_bestInlierOriginPointCloudDataIdx.begin());

					vec_bestReductiontPointCloudDataIdx.clear();
					vec_bestReductiontPointCloudDataIdx.resize(vec_tempReductionPointCloudDataIdxOfIdx.size());
					copy(vec_tempReductionPointCloudDataIdxOfIdx.begin(),vec_tempReductionPointCloudDataIdxOfIdx.end(),vec_bestReductiontPointCloudDataIdx.begin());

					// 베스트 평면 할당
					tempBestInlierPlane = phi;
				}
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			iterCnt++;
		}

		//if(vec_bestInlierOriginPointCloudDataIdx.size() > (inputSize*0.3))
		//{
		//	// 베스트 평면 할당
		//	bestInlierPlane.push_back(tempBestInlierPlane);
		//}

		// 베스트 평면 할당
		bestInlierPlane.push_back(tempBestInlierPlane);

		// 원본 Point Cloud Data에서 평면을 라벨링한다.
		for(int i=vec_bestInlierOriginPointCloudDataIdx.size()-1; i>=0; i--)
			vec_originPointCloudData[ vec_bestInlierOriginPointCloudDataIdx[i] ].clusterID = planeLabel;

		// 입력 데이터에서 인라이어 Point Cloud Data를 제거한다.
		// 이 부분이 시간이 오래걸림
		for(int i = vec_bestReductiontPointCloudDataIdx.size()-1; i>=0; i--){
			vec_reductionPointCloudData.erase( vec_reductionPointCloudData.begin() + vec_bestReductiontPointCloudDataIdx[i] );
			vec_reductionPointCloudDataIdx.erase( vec_reductionPointCloudDataIdx.begin() + vec_bestReductiontPointCloudDataIdx[i] );
		}

		cout << "best inlier cnt: "<<vec_bestInlierOriginPointCloudDataIdx.size() << endl;
		cout << "bestInlierPlane: "<<  planeLabel << ":" <<bestInlierPlane.size() << endl;
		cout << "iterCntPlane: "<<  iterCntPlane  << endl;

		inputSize = vec_reductionPointCloudData.size();
		planeLabel++;
		iterCntPlane++;
	}
	

}