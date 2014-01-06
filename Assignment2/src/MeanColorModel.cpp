#include "MeanColorModel.h"

//Constructs a 'mean color' model out of the given colors.
//This is a very simple model, which only stores the mean color for later comparison.
//The model currently uses BGR colors.
MeanColorModel::MeanColorModel(vector<Scalar> colors, int modelNr)
{
	//Model number used to differentiate the models
	_model_nr = modelNr;

	float B, G, R;
	B = G = R = 0.0;

	//Calculate the mean color
	for (int c = 0; c < colors.size(); c++)
	{
		B += colors[c][0];
		G += colors[c][1];
		R += colors[c][2];
	}

	B /= colors.size();
	G /= colors.size();
	R /= colors.size();

	_meanColor = Scalar(B, G, R);
	cout << "Created MeanColorModel #" << _model_nr << " with mean color: " << _meanColor;
}

MeanColorModel::~MeanColorModel(void)
{
}

//Returns the dissimilarity of a color to this model
float MeanColorModel::distanceTo(Scalar color)
{
	//Use three-dimensional Euclidean distance (on B, G, R)
	return norm(color - _meanColor);
}

//Shows an image that visualises the color model.
//Also returns this image.
//In this case, it just shows the mean color.
Mat& MeanColorModel::visualisationImage()
{
	Mat image = Mat(200, 200, CV_8UC3);
	image = _meanColor;
	imshow("Model #" + _model_nr, image);

	return image;
}