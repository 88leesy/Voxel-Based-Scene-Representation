#include "SY_Labeling.h"
#include <stack>

SY_Labeling::SY_Labeling(int width, int height)
{
	m_nWidth = width; m_nHeight = height;
	m_nNumOfLabels = 0;
}


SY_Labeling::~SY_Labeling()
{

}

int SY_Labeling::labeling(unsigned char* srcImg, int* dstImg)
{
	std::stack<SY_Point> st_pt;

	for (int y = 1; y < m_nHeight - 1; y++)
	{
		for (int x = 1; x < m_nWidth - 1; x++)
		{
			// source image가 255일 경우 + Labeling 수행되지 않은 픽셀에서만 labeling process 시작
			if (srcImg[m_nWidth * y + x] == 0 || dstImg[m_nWidth * y + x] != 0) continue;

			m_nNumOfLabels++;

			// 새로운 label seed를 stack에 push
			st_pt.push(SY_Point(x,y));

			// 해당 label seed가 labeling될 때(stack이 빌 때) 까지 수행
			while (!st_pt.empty())
			{
				// stack top의 label point를 받고 pop
				int ky = st_pt.top().y;
				int kx = st_pt.top().x;
				st_pt.pop();

				// label seed의 label number를 result image에 저장
				dstImg[m_nWidth * ky + kx] = m_nNumOfLabels;

				// search 8-neighbor
				for (int ny = ky - 1; ny <= ky + 1; ny++){
					// y축 범위를 벗어나는 점 제외
					if (ny < 0 || ny >= m_nHeight) continue;
					for (int nx = kx - 1; nx <= kx + 1; nx++){
						// x축 범위를 벗어나는 점 제외
						if (nx < 0 || nx >= m_nWidth) continue;

						// source image가 값이 있고 labeling이 안된 좌표를 stack에 push
						if (srcImg[m_nWidth * ny + nx] == 0 || dstImg[m_nWidth * ny + nx] != 0) continue;
						st_pt.push(SY_Point(nx, ny));

						// 탐색한 픽셀이니 labeling
						dstImg[m_nWidth * ny + nx] = m_nNumOfLabels;
					}
				}
			}
		}
	}

	return m_nNumOfLabels;
}

int SY_Labeling::labeling(unsigned short* srcImg, unsigned short* dstImg)
{
	m_vecPt.clear();
	m_vecPixelCnt.clear();
	std::stack<SY_Point> st_pt;

	int pxCnt = 1;
	for (int y = 1; y < m_nHeight - 1; y++)
	{
		for (int x = 1; x < m_nWidth - 1; x++)
		{
			unsigned short estIlluRatio = srcImg[m_nWidth * y + x];

			// source image가 0이 나닌 경우 + Labeling 수행되지 않은 픽셀에서만 labeling process 시작
			if (estIlluRatio == 0 || dstImg[m_nWidth * y + x] != 0) continue;

			m_nNumOfLabels++;
			pxCnt = 1;

			// 새로운 label seed를 stack에 push
			st_pt.push(SY_Point(x, y));

			double weightedSumX = 0, weightedSumY = 0;
			double ratioSum = 0;
			weightedSumX += x*estIlluRatio; weightedSumY += y*estIlluRatio;
			ratioSum += estIlluRatio;

			// 해당 label seed가 labeling될 때(stack이 빌 때) 까지 수행
			while (!st_pt.empty())
			{
				// stack top의 label point를 받고 pop
				int ky = st_pt.top().y;
				int kx = st_pt.top().x;
				st_pt.pop();

				// label seed의 label number를 result image에 저장
				dstImg[m_nWidth * ky + kx] = m_nNumOfLabels;

				// search 8-neighbor
				for (int ny = ky - 1; ny <= ky + 1; ny++){
					// y축 범위를 벗어나는 점 제외
					if (ny < 0 || ny >= m_nHeight) continue;
					for (int nx = kx - 1; nx <= kx + 1; nx++){
						// x축 범위를 벗어나는 점 제외
						if (nx < 0 || nx >= m_nWidth) continue;

						estIlluRatio = srcImg[m_nWidth * ny + nx];
						// source image가 값이 있고 labeling이 안된 좌표를 stack에 push
						if (estIlluRatio == 0 || dstImg[m_nWidth * ny + nx] != 0) continue;
						st_pt.push(SY_Point(nx, ny));

						// 탐색한 픽셀이니 labeling
						dstImg[m_nWidth * ny + nx] = m_nNumOfLabels;
						pxCnt++;

						weightedSumX += nx*estIlluRatio; weightedSumY += ny*estIlluRatio;
						ratioSum += estIlluRatio;
					}
				}
			} //while

			double cx = weightedSumX / ratioSum;
			double cy = weightedSumY / ratioSum;	
			m_vecPt.push_back(SY_Point(cx,cy));

			m_vecPixelCnt.push_back(pxCnt);
		}
	}

	return m_nNumOfLabels;
}

//아직 구현 안함... 할 필요성이 크게 없다...
void SY_Labeling::centeroidWithWeighted(int *labeledImg, int cx, int cy)
{
	for (int y = 0; y < m_nHeight; y++)
	{
		for (int x = 0; x < m_nWidth; x++)
		{
			int label = labeledImg[y*m_nWidth + x];
			if (label != 0)
			{
				;
			}
		}
	}
}