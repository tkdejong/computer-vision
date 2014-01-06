#pragma once

#include "opencv2/opencv.hpp"
using namespace cv;

class ColorModel
{
public:
	//ColorModel(void);
	virtual ~ColorModel(void);
	virtual float distanceTo(Scalar) = 0;
	virtual Mat& visualisationImage() = 0;
};

