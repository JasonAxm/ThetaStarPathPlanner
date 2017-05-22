#ifndef YANGTZE_PATHPLANNER_H_
#define YANGTZE_PATHPLANNER_H_

#include <cstdint>

#define PATHPLANNERDLL_EXPORTS
#ifdef PATHPLANNERDLL_EXPORTS
#define PATHPLANNERDLL_API __declspec(dllexport)
#else
#ifdef __linux__
#define PATHPLANNERDLL_API  
#else
#define PATHPLANNERDLL_API __declspec(dllimport) 
#endif
#endif

typedef int PlannerHandle;

#ifdef __cplusplus
extern "C" {
#endif
	PATHPLANNERDLL_API PlannerHandle createPathPlanner(const char* pMapPath);
	PATHPLANNERDLL_API void initPathPlanner(PlannerHandle handle);
	PATHPLANNERDLL_API void findPath(PlannerHandle handle,
		int startX, int startY,
		int goalX, int goalY,
		unsigned int *ids,
		unsigned int &idNum,
		unsigned int maxIdNum = 256);
	PATHPLANNERDLL_API void idToPos(PlannerHandle handle, unsigned int id, int &x, int &y);
	PATHPLANNERDLL_API void deletePathPlanner(PlannerHandle handle);

#ifdef __cplusplus
}
#endif

#endif
