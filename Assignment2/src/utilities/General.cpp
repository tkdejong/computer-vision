/*
 * General.cpp
 *
 *  Created on: Nov 13, 2013
 *      Author: coert
 */

#include "General.h"

using namespace std;

namespace nl_uu_science_gmt
{

const string General::CBConfigFile			= "checkerboard.xml";
const string General::CalibrationVideo		= "calibration.avi";
const string General::CheckerboadVideo		= "checkerboard.avi";
const string General::BackgroundVideoFile	= "background.avi";
const string General::BackgroundImageFile	= "background.png";
const string General::VideoFile				= "video.avi";
const string General::IntrinsicsFile		= "config.xml";
const string General::CheckerboadCorners	= "boardcorners.xml";
const string General::ConfigFile			= "config.xml";

/**
 * Linux/Windows friendly way to check if a file exists
 */
bool General::fexists(const std::string &filename)
{
	ifstream ifile(filename.c_str());
	return ifile.is_open();
}

} /* namespace nl_uu_science_gmt */
