#include <iostream>

#include "..\PathPlanner\PathPlanner.h"
#include "..\PathPlanner\\utility.hpp"

using namespace std;

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
//#pragma comment( lib, "PathPlanner.lib")

#endif //_WIN32

#include <opencv/cv.h>
#include <opencv/highgui.h>
using namespace cv;

class MapDisplayer
{
public:
	MapDisplayer()
	{
	}

	~MapDisplayer()
	{
	}

	bool initialize(const char* pImgPath, const char* pWindowName = "MapShow")
	{
		mWindowName = pWindowName;
		mShowImage = imread(pImgPath);
		if (mShowImage.empty())
		{
			return false;
		}
		namedWindow(mWindowName);
		setMouseCallback("MapShow", this->onMouseEvent, this);
		return true;
	}

	static void onMouseEvent(int event, int x, int y, int flags, void* userdata)
	{
		MapDisplayer *player = (MapDisplayer *)userdata;
		switch (event)
		{
		case EVENT_LBUTTONDBLCLK:
			if (!player->mbSetStart)
			{
				player->mStartPos = Point(x, y);
				player->mbSetStart = true;
			}
			else
			{
				player->mGoalPos = Point(x, y);
				player->mbSetGoal = true;
			}
			break;
		case EVENT_RBUTTONDOWN:
			player->mbSetGoal = false;
			player->mbSetStart = false;
			break;
		default:
			break;
		}
	}

	void showRobotPosInMap(Point pos, float alpha, vector<Vectori> &path)
	{
		Mat showMap = mShowImage.clone();
		circle(showMap, pos, 3, (255, 0, 0), -1);
		Point newcurrentPos = Point(int(pos.x + 8 * cos(alpha)),
			int(pos.y - 8 * sin(alpha)));

		line(showMap, pos, newcurrentPos, (0, 0, 255), 2);
		if (mbSetGoal)
		{
			circle(showMap, mGoalPos, 3, (0, 0, 255), -1);
		}

		if (!path.empty())
		{
			for (unsigned int i = 1; i < path.size(); i++)
			{
				auto node1 = path[i - 1];
				auto node2 = path[i];
				Point pt1(node1.x, node1.y);
				Point pt2(node2.x, node2.y);
				line(showMap, pt1, pt2, cvScalar(255, 0, 0));
			}
		}

		imshow(mWindowName, showMap);
	}

	bool getStartStatus()
	{
		return mbSetStart;
	}
	
	Point getStartPosition()
	{
		return mStartPos;
	}

	bool getGoalStatus()
	{
		return mbSetGoal;
	}

	void setGoalStatus(bool bGoalStatus)
	{
		mbSetGoal = bGoalStatus;
	}

	Point getGoalPosition()
	{
		return mGoalPos;
	}

private:
	string mWindowName;
	Mat mShowImage;
	Point mStartPos;
	Point mGoalPos;
	bool mbSetStart;
	bool mbSetGoal;
};

int main()
{
	const char* mapImagePath = "office_full.jpg";
	Mat mapImg = imread(mapImagePath, false);

// 	TileAdaptor adaptor(mapSize, [&mapImg](const Vectori& vec){return mapImg.at<uchar>(vec.y, vec.x) > 200;});
// 	Pathfinder pathfinder(adaptor, 1.00f);

	PlannerHandle handle = createPathPlanner(mapImagePath);

	int64 t0 = cvGetTickCount();
	//pathfinder.generateNodes();
	initPathPlanner(handle);
	int64 t1 = cvGetTickCount();
	printf("generateNodes const time is: %f ms\n", (t1 - t0) /(1000* cvGetTickFrequency()));

	MapDisplayer displayer;
	displayer.initialize(mapImagePath);
	Point startPos = Point(1, 1);
	Point goalPos = Point(664, 235);

	std::vector<Vectori> path;
	bool bInloop = true;
	unsigned int ids[1024] = { 0 };
	while (bInloop)
	{
		if (displayer.getStartStatus())
		{
			startPos = displayer.getStartPosition();
		}

		if (displayer.getGoalStatus())
		{
			path.clear();
			goalPos = displayer.getGoalPosition();
			Vectori startPoint = { startPos.x, startPos.y };
			Vectori endPoint = { goalPos.x, goalPos.y };
			t0 = cvGetTickCount();
			//auto nodePath = pathfinder.search(adaptor.posToId(startPoint), adaptor.posToId(endPoint));
			unsigned int idNum = 0;
			findPath(handle, startPoint.x, startPoint.y, goalPos.x, goalPos.y, ids, idNum);
			t1 = cvGetTickCount();
			printf("search const time is: %f ms\n", (t1 - t0) / (1000 * cvGetTickFrequency()));
			for (unsigned int i = 0; i < idNum; i++)
			{
				int x = 0;
				int y = 0;
				idToPos(handle, ids[i], x, y);
				Vectori node = { x, y };
				path.push_back(node);
			}
			displayer.setGoalStatus(false);
		}

		displayer.showRobotPosInMap(startPos, 0.0, path);
		int a = waitKey(40);
		switch (a)
		{
		case 27:
			bInloop = false;
			break;
		default:
			break;
		}
	}

    return 0;
}
