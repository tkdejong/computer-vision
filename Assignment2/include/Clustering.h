#pragma once

#include <vector>

#include "opencv2/opencv.hpp"
//#include "opencv2/core/core.hpp"

#include "Scene3DRenderer.h"
#include "ColorModel.h"
#include "MeanColorModel.h"

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

class Clustering
{
	Scene3DRenderer &_scene3d;
	int _K;
	vector<ColorModel> _models;

public:
	Clustering(Scene3DRenderer& scene3d, int);
	virtual ~Clustering(void);
	void initializeColorModel();
	bool isLocalMinimum(Mat& centers);
};

} // end namespace nl_uu_science_gmt