/*
 * VoxelReconstruction.h
 *
 *  Created on: Nov 13, 2013
 *      Author: coert
 */

#ifndef VOXELRECONSTRUCTION_H_
#define VOXELRECONSTRUCTION_H_

#include "opencv2/opencv.hpp"

#include "arcball.h"

#include "General.h"
#include "Camera.h"
#include "Reconstructor.h"
#include "Scene3DRenderer.h"
#include "Glut.h"

namespace nl_uu_science_gmt
{

class VoxelReconstruction
{
	const std::string _data_path;
	const int _cam_views_amount;

	std::vector<Camera*> _cam_views;

public:
	VoxelReconstruction(const std::string &, const int);
	virtual ~VoxelReconstruction();

	static void showKeys();

	void run(int, char**);
};

} /* namespace nl_uu_science_gmt */

#endif /* VOXELRECONSTRUCTION_H_ */
