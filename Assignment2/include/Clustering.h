#pragma once

#include <vector>

#include "opencv2/opencv.hpp"

#include "Scene3DRenderer.h"
#include "ColorHistogram.h"


using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

class Clustering
{
	Scene3DRenderer &_scene3d;
	int _K;
	vector<ColorHistogram> _models;
	vector<Camera*> _cams;

	int _min_z;
	int _max_z;

public:
	Clustering(Scene3DRenderer& scene3d, int);
	virtual ~Clustering(void);
	void initializeColorModel();
	void RunTracking();
	void generate();
	bool isLocalMinimum(Mat& centers);
	vector<vector<Scalar>> Clustering::getVoxelColorsBGR(vector<Reconstructor::Voxel*>);
	vector<Scalar> Clustering::getVoxelColorsBunchedBGR(vector<Reconstructor::Voxel*>);
	vector<vector<Scalar>> Clustering::getVoxelColorsHSV(vector<Reconstructor::Voxel*>);
	vector<Scalar> Clustering::getVoxelColorsBunchedHSV(vector<Reconstructor::Voxel*>);
};

} // end namespace nl_uu_science_gmt