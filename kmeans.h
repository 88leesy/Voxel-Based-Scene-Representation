#pragma once


/*
K-means 알고리즘

1. 군집의 개수 k 를 설정해준다. (사용자 input) : 위 그림에선 3이 k / 무작위의 위치에 선택

2. k에 대해서 모든 데이터와의 거리를 구하고, 각 데이터에 대해서 가장 가까운 k에 포함되도록 한다.

3. 각각의 k에 포함된 데이터들의 중심을 구하고, 각 k의 중심에 k를 이동시킨다.

4. step 2,3을 반복하여 모든 k의 위치가 변함없을 때 까지 반복한다
*/

#include<iostream>
#include<iomanip>
#include<string>
#include<algorithm>
#include<vector>


using namespace std;
#define K_COUNT 20
#define DATA_COUNT 1000

class pos {

public:
	double x;
	double y;
	//double z;
};


void kmeans()
{

	pos* k = new pos[K_COUNT];
	pos* center = new pos[K_COUNT];

	double count_Group[K_COUNT] = { 0, };

	vector< pos > datas;
	vector< double > distance[K_COUNT];

	//data
	for (int i = 0; i < DATA_COUNT; i++) {
		pos tmp;
		tmp.x = (double)(rand() % 100);
		tmp.y = (double)(rand() % 100);

		cout << "(" << tmp.x << "," << tmp.y << ")" << endl;
		datas.push_back(tmp);
	}//Randomly


	//random k, init
	for (int i = 0; i < K_COUNT; i++)
	{
		k[i] = datas[i];
		center[i].x = datas[i].x;
		center[i].y = datas[i].y;
		distance[i].resize(DATA_COUNT);
	}

	bool loop = true;
	while (loop) { //when the k-positions are all same with next position.
					  //center init
		for (int i = 0; i < K_COUNT; i++) {
			center[i].x = 0;
			center[i].y = 0;
			count_Group[i] = 0;
		}
				
		// distance
		for (int i = 0; i < datas.size(); i++) {
			for (int j = 0; j < K_COUNT; j++) {
				double tmp_distance = sqrt(pow(k[j].x - datas[i].x, 2) + pow(k[j].y - datas[i].y, 2));
					distance[j][i] = tmp_distance;
			}
		}


		//get center
		for (int i = 0; i < datas.size(); i++) {
			double min = distance[0][i];
			int min_j = 0;

			for (int j = 1; j < K_COUNT; j++) {
				if (min > distance[j][i]) {
					min = distance[j][i];
					min_j = j;

				}

			}

			center[min_j].x += datas[i].x;
			center[min_j].y += datas[i].y;
			count_Group[min_j]++;
		}

		//change K
		int same_count = 0;
		for (int i = 0; i < K_COUNT; i++) {

			if (count_Group[i] != 0) {
				if ((center[i].x / count_Group[i]) == k[i].x
					&& (center[i].y / count_Group[i] == k[i].y))
					same_count++;

				k[i].x = center[i].x / count_Group[i];
				k[i].y = center[i].y / count_Group[i];
			}

			if (same_count == K_COUNT) {
				loop = false;
			}

			cout << fixed << setprecision(2);
			cout << "(" << k[i].x << "," << k[i].y << ") ";

		}cout << endl;

	}//end of loop





	for (int i = 0; i < datas.size(); i++) {

		double min = distance[0][i];

		int min_j = 0;

		for (int j = 1; j < K_COUNT; j++) {

			if (min > distance[j][i]) {

				min = distance[j][i];

				min_j = j;

			}

		}

		cout << min_j << " ";

	}cout << endl;

}

