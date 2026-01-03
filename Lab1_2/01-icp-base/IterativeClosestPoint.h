#ifndef _ITERATIVE_CLOSEST_POINT_INCLUDE
#define _ITERATIVE_CLOSEST_POINT_INCLUDE


#include "PointCloud.h"
#include "NearestNeighbors.h"


class IterativeClosestPoint
{

public:
	void setClouds(PointCloud *pointCloud1, PointCloud *pointCloud2);
	
	void markBorderPoints();
	vector<int> *computeCorrespondence();
	glm::mat4 computeICPStep();
	
	vector<int> *computeFullICP(unsigned int maxSteps = 100);
	
private:
	PointCloud *cloud1, *cloud2;
	NearestNeighbors knn;
  std::vector<glm::vec3> P_corr; 
  std::vector<glm::vec3> Q_corr; 
  std::vector<bool> isBorder; // Per cloud1
  glm::mat4 T_total = glm::mat4(1.0f); // Trasformazione totale applicata
};


#endif // _ITERATIVE_CLOSEST_POINT_INCLUDE


