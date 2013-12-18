#pragma once

#include <vector>

#include "opencv2/opencv.hpp"
//#include "opencv2/core/core.hpp"

#include "Scene3DRenderer.h"

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

class Clustering
{
	Scene3DRenderer &_scene3d;
	int _K;

public:
	Clustering(Scene3DRenderer& scene3d, int);
	virtual ~Clustering(void);
	void initializeColorModel();
};

} // end namespace nl_uu_science_gmt