/*
 * Scene3DRenderer.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#include "Scene3DRenderer.h"

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

/**
 * Scene properties class (mostly called by Glut)
 */
Scene3DRenderer::Scene3DRenderer(Reconstructor &r, const vector<Camera*> &cs) :
		_reconstructor(r), _cameras(cs), _num(4), _sphere_radius(1850)
{
	_width = 640;
	_height = 480;
	_quit = false;
	_paused = false;
	_rotate = false;
	_camera_view = true;
	_show_volume = true;
	_show_grd_flr = true;
	_show_cam = true;
	_show_org = true;
	_show_arcball = false;
	_show_info = true;
	_fullscreen = false;

	// Read the checkerboard properties (XML)
	FileStorage fs;
	fs.open(_cameras.front()->getDataPath() + ".." + string(PATH_SEP) + General::CBConfigFile, FileStorage::READ);
	if (fs.isOpened())
	{
		fs["CheckerBoardWidth"] >> _board_size.width;
		fs["CheckerBoardHeight"] >> _board_size.height;
		fs["CheckerBoardSquareSize"] >> _square_side_len;
	}
	fs.release();

	_current_camera = 0;
	_previous_camera = 0;

	_number_of_frames = _cameras.front()->getFramesAmount();
	_current_frame = 0;
	_previous_frame = -1;

	const int H = 0;
	const int S = 15;
	const int V = 25;
	_h_threshold = H;
	_ph_threshold = H;
	_s_threshold = S;
	_ps_threshold = S;
	_v_threshold = V;
	_pv_threshold = V;

	createTrackbar("Frame", VIDEO_WINDOW, &_current_frame, _number_of_frames - 2);
	createTrackbar("H", VIDEO_WINDOW, &_h_threshold, 255);
	createTrackbar("S", VIDEO_WINDOW, &_s_threshold, 255);
	createTrackbar("V", VIDEO_WINDOW, &_v_threshold, 255);

	createFloorGrid();
	setTopView();
}

/**
 * Free the memory of the floor_grid pointer vector
 */
Scene3DRenderer::~Scene3DRenderer()
{
	for (size_t f = 0; f < _floor_grid.size(); ++f)
		for (size_t g = 0; g < _floor_grid[f].size(); ++g)
			delete _floor_grid[f][g];
}

/**
 * Process the current frame on each camera
 */
bool Scene3DRenderer::processFrame()
{
	for (size_t c = 0; c < _cameras.size(); ++c)
	{
		if (_current_frame == _previous_frame + 1)
		{
			_cameras[c]->advanceVideoFrame();
		}
		else if (_current_frame != _previous_frame)
		{
			_cameras[c]->getVideoFrame(_current_frame);
		}
		processForeground(_cameras[c]);
	}
	return true;
}

/**
 * Separate the background from the foreground
 * ie.: Create an 8 bit image where only the foreground of the scene is white
 */
void Scene3DRenderer::processForeground(Camera* camera)
{
	Mat hsv_image;
	cvtColor(camera->getFrame(), hsv_image, CV_BGR2HSV);  // from BGR to HSV color space

	vector<Mat> channels;
	split(hsv_image, channels);  // Split the HSV-channels for further analysis

	// Background subtraction H
	Mat tmp, foreground, background;
	absdiff(channels[0], camera->getBgHsvChannels().at(0), tmp);
	threshold(tmp, foreground, _h_threshold, 255, CV_THRESH_BINARY);

	// Background subtraction S
	absdiff(channels[1], camera->getBgHsvChannels().at(1), tmp);
	threshold(tmp, background, _s_threshold, 255, CV_THRESH_BINARY);
	bitwise_and(foreground, background, foreground);

	// Background subtraction V
	absdiff(channels[2], camera->getBgHsvChannels().at(2), tmp);
	threshold(tmp, background, _v_threshold, 255, CV_THRESH_BINARY);
	bitwise_or(foreground, background, foreground);

	// Remove noise
#ifndef USE_GRAPHCUTS
	// Using Erosion and/or Dilation of the foreground image
#else
	// Using Graph cuts on the foreground image
#endif

	camera->setForegroundImage(foreground);
}

/**
 * Set currently visible camera to the given camera id
 */
void Scene3DRenderer::setCamera(int camera)
{
	_camera_view = true;

	if (_current_camera != camera)
	{
		_previous_camera = _current_camera;
		_current_camera = camera;
		_arcball_eye.x = _cameras[camera]->getCameraPlane()[0].x;
		_arcball_eye.y = _cameras[camera]->getCameraPlane()[0].y;
		_arcball_eye.z = _cameras[camera]->getCameraPlane()[0].z;
		_arcball_up.x = 0.0f;
		_arcball_up.y = 0.0f;
		_arcball_up.z = 1.0f;
	}
}

/**
 * Set the 3D scene to bird's eye view
 */
void Scene3DRenderer::setTopView()
{
	_camera_view = false;
	if (_current_camera != -1) _previous_camera = _current_camera;
	_current_camera = -1;

	_arcball_eye = vec(0.0f, 0.0f, 10000.0f);
	_arcball_centre = vec(0.0f, 0.0f, 0.0f);
	_arcball_up = vec(0.0f, 1.0f, 0.0f);
}

/**
 * Create a LUT for the floor grid
 */
void Scene3DRenderer::createFloorGrid()
{
	const int size = _reconstructor.getSize();
	const int z_offset = 3;

	// edge 1
	vector<Point3i*> edge1;
	for (int y = -size * _num; y <= size * _num; y += size)
		edge1.push_back(new Point3i(-size * _num, y, z_offset));

	// edge 2
	vector<Point3i*> edge2;
	for (int x = -size * _num; x <= size * _num; x += size)
		edge2.push_back(new Point3i(x, size * _num, z_offset));

	// edge 3
	vector<Point3i*> edge3;
	for (int y = -size * _num; y <= size * _num; y += size)
		edge3.push_back(new Point3i(size * _num, y, z_offset));

	// edge 4
	vector<Point3i*> edge4;
	for (int x = -size * _num; x <= size * _num; x += size)
		edge4.push_back(new Point3i(x, -size * _num, z_offset));

	_floor_grid.push_back(edge1);
	_floor_grid.push_back(edge2);
	_floor_grid.push_back(edge3);
	_floor_grid.push_back(edge4);
}

} /* namespace nl_uu_science_gmt */
