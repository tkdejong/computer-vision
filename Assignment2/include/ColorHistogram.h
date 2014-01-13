#pragma once
#include "colormodel.h"
#include "Reconstructor.h"

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
using namespace nl_uu_science_gmt;

class ColorHistogram: public ColorModel
{
	int _bins;
	vector<float> _bin;
	float _bin_size;

	int _v_bins;
	vector<float> _v_bin;
	float _v_bin_size;

	int _model_nr;

public:
	ColorHistogram(vector<Scalar>, int, int);
	virtual ~ColorHistogram(void);

	float distanceTo(Scalar);
	Mat& visualisationImage();
};

