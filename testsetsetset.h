#pragma once
//outlier 제거 후 첫 클러스터링...
printf("\n -------------------------------------------------------\n");
printf("\n --- Hierarchical Clustering initialization... [level 0] ---\n");

float maxX = -100000, maxY = -100000, maxZ = -100000;
float minX = 1000000, minY = 1000000, minZ = 1000000;
float sumX = 0, sumY = 0, sumZ = 0;
for (int i = 0; i < refinedPoints.size(); ++i)
{
	if (maxX < refinedPoints[i].x) maxX = refinedPoints[i].x;
	if (maxY < refinedPoints[i].y) maxY = refinedPoints[i].y;
	if (maxZ < refinedPoints[i].z) maxZ = refinedPoints[i].z;

	if (minX > refinedPoints[i].x) minX = refinedPoints[i].x;
	if (minY > refinedPoints[i].y) minY = refinedPoints[i].y;
	if (minZ > refinedPoints[i].z) minZ = refinedPoints[i].z;
}
printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);

float cubeW = maxX - minX;
float cubeH = maxY - minY;
float cubeD = maxZ - minZ;
float maxLen = max(max(cubeW, cubeH), cubeD);

ptr_cloud->clear();
ptr_cloud->points.clear();
nCurrCnt = 0;

//int width = 10 * cubeW / maxLen;   //x
//int height = 10 * cubeH / maxLen;  //y
//int depth = 10 * cubeD / maxLen;   //z
int width = m_cubeSize;   //x
int height = m_cubeSize;  //y
int depth = m_cubeSize;   //z
if (width <= 0) width = 1;
if (height <= 0) height = 1;
if (depth <= 0) depth = 1;
printf("width:%d, height:%d, depth:%d\n", width, height, depth);

vector<DB_Point_3D> ***cube_points = new vector<DB_Point_3D> **[width];
for (int x = 0; x < width; ++x)
{
	cube_points[x] = new vector<DB_Point_3D> *[height];
	for (int y = 0; y < height; ++y)
	{
		cube_points[x][y] = new vector<DB_Point_3D>[depth];
	}
}

for (int x = 1; x <= width; ++x)
{
	float minRangeX = minX + (maxX - minX) * ((float)(x - 1) / width);
	float maxRangeX = minX + (maxX - minX) * ((float)(x) / width);

	for (int y = 1; y <= height; ++y)
	{
		float minRangeY = minY + (maxY - minY) * ((float)(y - 1) / height);
		float maxRangeY = minY + (maxY - minY) * ((float)(y) / height);
		for (int z = 1; z <= depth; ++z)
		{
			float minRangeZ = minZ + (maxZ - minZ) * ((float)(z - 1) / depth);
			float maxRangeZ = minZ + (maxZ - minZ) * ((float)(z) / depth);

			++nCurrCnt;

			for (int i = 0; i < refinedPoints.size(); ++i)
			{
				if ((minRangeX <= refinedPoints[i].x && refinedPoints[i].x <= maxRangeX)
					&& (minRangeY <= refinedPoints[i].y && refinedPoints[i].y <= maxRangeY)
					&& (minRangeZ <= refinedPoints[i].z && refinedPoints[i].z <= maxRangeZ))
				{
					refinedPoints[i].clusterID = nCurrCnt;
					cube_points[x - 1][y - 1][z - 1].push_back(refinedPoints[i]);
				}
			}
		}
	}
}

//To do!!!! - 2021.03.15 - 계층적 클러스터링 구현하기!
int level = 2;
nCurrCnt = 0;
for (int lv = 1; lv < level; ++lv)
{
	printf("\n -------------------------------------------------------\n");
	printf("\n --- Hierarchical Clustering [level:%d] ---\n", lv);


	for (int x = 0; x < width; ++x)
	{
		for (int y = 0; y < height; ++y)
		{
			for (int z = 0; z < depth; ++z)
			{

				vector<DB_Point_3D> tmp_db_pt = cube_points[x][y][z];

				float maxX = -100000, maxY = -100000, maxZ = -100000;
				float minX = 1000000, minY = 1000000, minZ = 1000000;
				float sumX = 0, sumY = 0, sumZ = 0;
				for (int i = 0; i < tmp_db_pt.size(); ++i)
				{
					if (maxX < tmp_db_pt[i].x) maxX = tmp_db_pt[i].x;
					if (maxY < tmp_db_pt[i].y) maxY = tmp_db_pt[i].y;
					if (maxZ < tmp_db_pt[i].z) maxZ = tmp_db_pt[i].z;

					if (minX > tmp_db_pt[i].x) minX = tmp_db_pt[i].x;
					if (minY > tmp_db_pt[i].y) minY = tmp_db_pt[i].y;
					if (minZ > tmp_db_pt[i].z) minZ = tmp_db_pt[i].z;
				}
				printf("maxX:%f, maxY:%f, maxZ:%f, minX:%f, minY:%f, minZ:%f\n", maxX, maxY, maxZ, minX, minY, minZ);


				//float cubeW = maxX - minX;
				//float cubeH = maxY - minY;
				//float cubeD = maxZ - minZ;
				//float maxLen = max(max(cubeW, cubeH), cubeD);
				//int width = m_cubeSize * cubeW / maxLen;   //x
				//int height = m_cubeSize * cubeH / maxLen;  //y
				//int depth = m_cubeSize * cubeD / maxLen;   //z
				int width = m_cubeSize;   //x
				int height = m_cubeSize;  //y
				int depth = m_cubeSize;   //z
				if (width <= 0) width = 1;
				if (height <= 0) height = 1;
				if (depth <= 0) depth = 1;
				printf("width:%d, height:%d, depth:%d\n", width, height, depth);


				for (int xx = 1; xx <= width; ++xx)
				{
					float minRangeX = minX + (maxX - minX) * ((float)(xx - 1) / width);
					float maxRangeX = minX + (maxX - minX) * ((float)(xx) / width);

					for (int yy = 1; yy <= height; ++yy)
					{
						float minRangeY = minY + (maxY - minY) * ((float)(yy - 1) / height);
						float maxRangeY = minY + (maxY - minY) * ((float)(yy) / height);
						for (int zz = 1; zz <= depth; ++zz)
						{
							float minRangeZ = minZ + (maxZ - minZ) * ((float)(zz - 1) / depth);
							float maxRangeZ = minZ + (maxZ - minZ) * ((float)(zz) / depth);

							++nCurrCnt;

							for (int i = 0; i < tmp_db_pt.size(); ++i)
							{
								if ((minRangeX <= tmp_db_pt[i].x && tmp_db_pt[i].x <= maxRangeX)
									&& (minRangeY <= tmp_db_pt[i].y && tmp_db_pt[i].y <= maxRangeY)
									&& (minRangeZ <= tmp_db_pt[i].z && tmp_db_pt[i].z <= maxRangeZ))
								{
									tmp_db_pt[i].clusterID = nCurrCnt;
									//cube_points[x - 1][y - 1][z - 1].push_back(tmp_db_pt[i]);
								}
							}
						}
					}
				}

				cube_points[x][y][z] = tmp_db_pt;


			}  //z
		}
	}
}