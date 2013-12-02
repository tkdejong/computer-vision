/*
 * General.h
 *
 *  Created on: Nov 13, 2013
 *      Author: coert
 */

#ifndef GENERAL_H_
#define GENERAL_H_

#include <fstream>
#include "opencv2/opencv.hpp"

// Windows path separators are rather ugly
#ifdef __linux__
#define PATH_SEP "/"
#elif defined __APPLE__
#define PATH_SEP "/"
#elif defined _WIN32
#define PATH_SEP "\\"
#endif

namespace nl_uu_science_gmt
{

// Version and Main OpenCV window name
const static std::string VERSION = "2.0";
const static std::string VIDEO_WINDOW = "Video";
const static std::string SCENE_WINDOW = "OpenGL 3D scene";

// Some OpenCV colors
const static cv::Scalar Color_BLUE = cv::Scalar(255, 0, 0);
const static cv::Scalar Color_GREEN = cv::Scalar(0, 200, 0);
const static cv::Scalar Color_RED = cv::Scalar(0, 0, 255);
const static cv::Scalar Color_YELLOW = cv::Scalar(0, 255, 255);
const static cv::Scalar Color_MAGENTA = cv::Scalar(255, 0, 255);
const static cv::Scalar Color_CYAN = cv::Scalar(255, 255, 0);
const static cv::Scalar Color_WHITE = cv::Scalar(255, 255, 255);
const static cv::Scalar Color_BLACK = cv::Scalar(0, 0, 0);

class General
{
public:
	static const std::string CBConfigFile;
	static const std::string IntrinsicsFile;
	static const std::string CalibrationVideo;
	static const std::string CheckerboadVideo;
	static const std::string CheckerboadCorners;
	static const std::string VideoFile;
	static const std::string BackgroundImageFile;
	static const std::string BackgroundVideoFile;
	static const std::string ConfigFile;

	static bool fexists(const std::string &);
};

} /* namespace nl_uu_science_gmt */

#endif /* GENERAL_H_ */
