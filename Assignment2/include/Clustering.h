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
	vector<Camera*> _cams;
	int _K;
	vector<ColorHistogram> _models;
	
	int _min_z;
	int _max_z;
	float _person_radius;
	int _init_frame;
	vector<Scalar> drawcolors;
	Scalar unlabeledColor;
	//If true, the initial labeling is shown, instead of the final one.
	//Unlabeled voxels will be shown as gray
	bool _show_initial;

	//vector<Mat> occlusionMap;
	//vector<vector<bool>> occluded;
	vector<Point2f> _prev_centers;


public:
	Clustering(Scene3DRenderer& scene3d, int, bool);
	virtual ~Clustering(void);
	void initializeColorModel();
	vector<Point2f> processFrame();
	void makeTextFile();
	void generate();
	bool isLocalMinimum(Mat& centers);
	void processOcclusions(vector<Reconstructor::Voxel*>);

	vector<Scalar> Clustering::getVoxelColors(Reconstructor::Voxel*, int, vector<Mat>);	
	vector<Scalar> Clustering::getVoxelColorsBunchedHSV(vector<Reconstructor::Voxel*>);
};

} // end namespace nl_uu_science_gmt