#include "..\include\ColorHistogram.h"

//Constructs a Hue Histogram out of the given colors.
//All colors (Scalars) are expected to be HSV values, with H between 1-180.
ColorHistogram::ColorHistogram(vector<Scalar> colors, int bins, int modelNr)
{
	//Model number used to differentiate the models
	_model_nr = modelNr;

	//Store for each bin the percentage of voxels with that hue
	_bins = bins;
	_bin = vector<float>(bins, 0.0);
	//The (converted) Hue values range from 1 to 180
	_bin_size = 180.0 / (float) bins;

	//Fill the bins with the hue of the sample colors
	//The sum of the bins will be 1
	float totalColors = (float) colors.size();
	for (int c = 0; c < colors.size(); c++)
	{
		float hue = (float) colors[c][0];
		//Add to the correct bin the correct percentage
		_bin[floor(hue / _bin_size)] += 1.0 / totalColors;
	}
}

ColorHistogram::~ColorHistogram(void)
{
}

//Returns the dissimilarity of an HSV color to this model
float ColorHistogram::distanceTo(Scalar color)
{
	//Corresponding bin height, = % of the sample colors in that went in the same bin
	float height = _bin[floor(color[0] / _bin_size)];
	
	//return the inverse of the 'chance' of the color being in the model
	//result: if the corresponding bin value is X times as large, the distance is X times smaller
	float dist = (height == 0.0) ? 1000000.0 : (1/height);
	return dist;
}

//Shows an image that visualises the color model.
//Also returns this image.
Mat& ColorHistogram::visualisationImage()
{
	//Make the size easily adjustable
	int height = 300;
	int width = 300;

	//Make a black image with colored bars for the bins
	//The hue of each bin is the 'middle hue' of those that fit in the bin.
	Mat preview = Mat(height, width, CV_8UC3, Scalar(255, 0, 0));
	for (int b = 0; b < _bin.size(); b++)
	{
		int left = floor(width * (_bin_size * b) / 180.0);
		int right = floor(width * (_bin_size * (b+1)) / 180.0);
		int top = floor(height - _bin[b] * height);
		Scalar color = Scalar(_bin_size * (((float) b) + 0.5), 255, 255);
		//Scalar color = Scalar(0, 0, _bin_size * (((float) b) + 0.5));

		rectangle(preview, Point(left, top), Point(right, height), color, CV_FILLED);
	}
		
	//The bins were drawn with HSV in mind, so convert it to the standard BGR before showing
	cvtColor(preview, preview, CV_HSV2BGR);
	imshow("Color Histogram #" + to_string(_model_nr), preview);

	return preview;
}