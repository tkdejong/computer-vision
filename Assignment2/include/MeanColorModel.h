#pragma once
#include "colormodel.h"
#include "Reconstructor.h"

#include "opencv2/opencv.hpp"


using namespace std;
using namespace cv;
using namespace nl_uu_science_gmt;

class MeanColorModel : public ColorModel
{
	Scalar _meanColor;
	int _model_nr;
public:
	MeanColorModel(vector<Scalar>, int);
	virtual ~MeanColorModel(void);

	float distanceTo(Scalar);
	Mat& visualisationImage();
};