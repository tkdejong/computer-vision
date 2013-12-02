/*
 * Camera.h
 *
 *  Created on: Nov 13, 2013
 *      Author: coert
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include "opencv2/opencv.hpp"

#include "General.h"

namespace nl_uu_science_gmt
{

#define MAIN_WINDOW "Checkerboard Marking"

class Camera
{
	static std::vector<cv::Point>* _BoardCorners;  // marked checkerboard corners

	bool _initialized;

	const std::string _data_path;
	const std::string _cam_prop;
	const int _id;

	std::vector<cv::Mat> _bg_hsv_channels;
	cv::Mat _foreground_image;

	cv::VideoCapture _video;

	cv::Size _plane_size;
	long _frames;

	cv::Mat _camera_matrix, _distortion_coeffs;
	cv::Mat _rotation_values, _translation_values;

	float _fx, _fy, _px, _py;

	cv::Mat _rt;
	cv::Mat _inverse_rt;

	cv::Point3f _camera_location; // the camera location in the 3D space
	std::vector<cv::Point3f> _camera_plane; // the camera plane of view
	std::vector<cv::Point3f> _camera_floor; // three points that are the projection of the camera itself to the ground floor view

	cv::Mat _frame;

	static void onMouse(int, int, int, int, void*);
	void initCamLoc();
	inline void camPtInWorld();

	cv::Point3f ptToW3D(const cv::Point &);
	cv::Point3f cam3DtoW3D(const cv::Point3f &);

public:
	Camera(const std::string &, const std::string &, int);
	virtual ~Camera();

	bool initialize();

	cv::Mat& advanceVideoFrame();
	cv::Mat& getVideoFrame(int);
	void setVideoFrame(int);

	static bool detExtrinsics(const std::string &, const std::string &, const std::string &, const std::string &);

	static cv::Point projectOnView(const cv::Point3f &, const cv::Mat &, const cv::Mat &, const cv::Mat &, const cv::Mat &);
	cv::Point projectOnView(const cv::Point3f &);

	const std::string& getCamPropertiesFile() const
	{
		return _cam_prop;
	}

	const std::string& getDataPath() const
	{
		return _data_path;
	}

	const int getId() const
	{
		return _id;
	}

	const cv::VideoCapture& getVideo() const
	{
		return _video;
	}

	void setVideo(const cv::VideoCapture& video)
	{
		_video = video;
	}

	long getFramesAmount() const
	{
		return _frames;
	}

	const std::vector<cv::Mat>& getBgHsvChannels() const
	{
		return _bg_hsv_channels;
	}

	bool isInitialized() const
	{
		return _initialized;
	}

	const cv::Size& getSize() const
	{
		return _plane_size;
	}

	const cv::Mat& getForegroundImage() const
	{
		return _foreground_image;
	}

	void setForegroundImage(const cv::Mat& foregroundImage)
	{
		_foreground_image = foregroundImage;
	}

	const cv::Mat& getFrame() const
	{
		return _frame;
	}

	const std::vector<cv::Point3f>& getCameraFloor() const
	{
		return _camera_floor;
	}

	const cv::Point3f& getCameraLocation() const
	{
		return _camera_location;
	}

	const std::vector<cv::Point3f>& getCameraPlane() const
	{
		return _camera_plane;
	}
};

} /* namespace nl_uu_science_gmt */

#endif /* CAMERA_H_ */
