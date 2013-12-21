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
public:
	MeanColorModel(vector<Reconstructor::Voxel*>);
	virtual ~MeanColorModel(void);

	float distanceTo(Scalar);
};