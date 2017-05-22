#include <iostream>
#include "PathPlanner.h"
#include "tileadaptor.hpp"
using namespace std;
#include <opencv/cv.h>
#include <opencv/highgui.h>
using namespace cv;

#ifdef _WIN32
#pragma once
#include <opencv2/core/version.hpp>
#define CV_VERSION_ID CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) \
  CVAUX_STR(CV_SUBMINOR_VERSION)
#ifdef _DEBUG
#define cvLIB(name) "opencv_" name CV_VERSION_ID "d"
#else
#define cvLIB(name) "opencv_" name CV_VERSION_ID
#endif //_DEBUG

#pragma comment( lib, cvLIB("core") )
//#pragma comment( lib, cvLIB("imgproc") )
#pragma comment( lib, cvLIB("highgui") )

#endif //_WIN32

inline bool IsTraversable(const Vectori &vec, const void *pMapDataPtr)
{
	IplImage *pMapImage = (IplImage *)pMapDataPtr;
	return ((uchar)cvGet2D(pMapImage, vec.y, vec.x).val[0]) > 200;
}

PlannerHandle createPathPlanner(const char* pMapPath)
{
	IplImage *pMapImg = cvLoadImage(pMapPath, false);
	if (NULL == pMapImg)
	{
		return NULL;
	}

	Vectori mapSize = {pMapImg->width, pMapImg->height };
	TileAdaptor *pAdaptor = new TileAdaptor(mapSize, (void *)pMapImg, IsTraversable);
	Pathfinder *pPathfinder = new Pathfinder(*pAdaptor, 1.00f);
	return (PlannerHandle)pPathfinder;
}

void initPathPlanner(PlannerHandle handle)
{
	Pathfinder *pPathPlanner = (Pathfinder *)handle;
	pPathPlanner->generateNodes();
}

void findPath(PlannerHandle handle, 
	int startX, int startY,
	int goalX, int goalY, 
	unsigned int *ids,
	unsigned int &idNum,
	unsigned int maxIdNum)
{
	Pathfinder *pPathPlanner = (Pathfinder *)handle;
	TileAdaptor *adaptor = (TileAdaptor *)pPathPlanner->getAdaptor();
	Vectori startPoint = { startX, startY };
	Vectori goalPoint = { goalX, goalY };

	std::vector<Vectori> path;
	auto nodePath = pPathPlanner->search(adaptor->posToId(startPoint), adaptor->posToId(goalPoint));
	idNum = nodePath.size();
	unsigned int cnt = 0;
	for (const auto id : nodePath)
	{
		ids[cnt] = id;
		cnt++;
		if (cnt >= maxIdNum) {
			break;
		}
	}
}

void idToPos(PlannerHandle handle, unsigned int id, int &x, int &y)
{
	Pathfinder *pPathPlanner = (Pathfinder *)handle;
	TileAdaptor *adaptor = (TileAdaptor *)pPathPlanner->getAdaptor();
	Vectori posPt = adaptor->idToPos(id);
	x = posPt.x;
	y = posPt.y;
}

void deletePathPlanner(PlannerHandle handle)
{
	Pathfinder *pPathPlanner = (Pathfinder *)handle;
	TileAdaptor *pAdaptor = (TileAdaptor *)pPathPlanner->getAdaptor();
	void *pMapPtr = pAdaptor->getMapDataPtr();
	if (NULL != pMapPtr)
	{
		IplImage *pMapImage = (IplImage *)pMapPtr;
		cvReleaseImage(&pMapImage);
		pMapImage = NULL;
	}

	if (NULL != pAdaptor)
	{
		delete pAdaptor;
		pAdaptor = NULL;
	}

	if (NULL != pPathPlanner)
	{
		delete pPathPlanner;
		pPathPlanner = NULL;
	}
}