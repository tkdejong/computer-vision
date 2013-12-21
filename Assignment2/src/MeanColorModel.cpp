#include "MeanColorModel.h"


MeanColorModel::MeanColorModel(vector<Reconstructor::Voxel*> voxels)
{
	_meanColor = Scalar(0.0, 0.0, 0.0);
	//Calculate the mean color of the voxels
	//for (
}


MeanColorModel::~MeanColorModel(void)
{
}

float MeanColorModel::distanceTo(Scalar color)
{
	return norm(color - _meanColor);
}