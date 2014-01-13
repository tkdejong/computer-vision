#include "..\include\ColorHistogram.h"

//Constructs a Hue histogram out of the given colors.
//Also constructs a Value histogram to use for colors with low S or V values,
//  for which the Hue is less distinctive.
//All colors (Scalars) are expected to be HSV values, with H between 0-179 and S and V between 0-255 
ColorHistogram::ColorHistogram(vector<Scalar> colors, int bins, int modelNr)
{
	//Model number used to differentiate the models
	_model_nr = modelNr;

	
	_bins = bins;
	//Store for each bin the percentage of voxels with that Hue
	_bin = vector<float>(bins, 0.0);
	//The (converted) Hue values range from 0 to 179
	_bin_size = 180.0 / (float) bins;

	//Store the same things for Value in a separate histogram
	_v_bins = 40;
	_v_bin = vector<float>(_v_bins, 0.0);
	_v_bin_size = 256.0 / (float) _v_bins;

	


	//Fill the bins with the hue and value of the sample colors
	//The sum of the bins will be 1
	float totalColors = (float) colors.size();
	for (int c = 0; c < colors.size(); c++)
	{
		float hue = (float) colors[c][0];
		//Add to the correct bin the correct percentage
		_bin[floor(hue / _bin_size)] += 1.0 / totalColors;

		float val = (float) colors[c][2];
		//Add to the correct bin the correct percentage
		_v_bin[floor(val / _v_bin_size)] += 1.0 / totalColors;
	}
}

ColorHistogram::~ColorHistogram(void)
{
}

//Returns the dissimilarity of an HSV color to this model
float ColorHistogram::distanceTo(Scalar color)
{
	//Corresponding bin height, = % of the sample colors in that went in the same bin
	float height = 0.0;

	//Dark or faded colors are not easy to distinguish by hue,
	//  so in these cases the value histogram will be used instead
	//The values 13 and 16 were empirically determined using a color picker
	if (color[1] > 13 && color [2] > 16)
	{
		//Compensate for the bin size
		height = _bin[floor(color[0] / _bin_size)] * _bin_size;
	}
	else
	{
		//Compensate for the bin size
		height = _v_bin[floor(color[2] / _v_bin_size)] * _v_bin_size;
	}
	
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

	//Make a black image with colored bars for the hue histogram
	//The hue of each bin is the 'middle hue' of those that fit in the bin.
	Mat preview = Mat(height * 2, width, CV_8UC3, Scalar(0, 0, 0));
	for (int b = 0; b < _bin.size(); b++)
	{
		int left = floor(width * (_bin_size * b) / 180.0);
		int right = floor(width * (_bin_size * (b+1)) / 180.0);
		int top = floor(height - _bin[b] * height - 1);
		Scalar color = Scalar(_bin_size * (((float) b) + 0.5), 255, 255);

		rectangle(preview, Point(left, top), Point(right, height), color, CV_FILLED);
	}

	//Draw a red image beneath the hue histogram, with black/white bars, for the value histogram
	rectangle(preview, Point(0, height), Point(width-1, 2*height-1), Scalar(0, 255, 255), CV_FILLED);
	for (int b = 0; b < _v_bin.size(); b++)
	{
		int left = floor(width * (_v_bin_size * b) / 180.0);
		int right = floor(width * (_v_bin_size * (b+1)) / 180.0);
		int top = floor(2*height - _v_bin[b] * height - 1);
		int bottom = 2*height - 1;
		Scalar color = Scalar(0, 0, _v_bin_size * (((float) b) + 0.5));

		rectangle(preview, Point(left, top), Point(right, bottom), color, CV_FILLED);
	}

	//The bins were drawn with HSV in mind, so convert it to the standard BGR before showing
	cvtColor(preview, preview, CV_HSV2BGR);
	imshow("Color Histogram #" + to_string(_model_nr), preview);

	return preview;
}