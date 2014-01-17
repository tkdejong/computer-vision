#include "Clustering.h"
#include <iostream>
#include <fstream>

namespace nl_uu_science_gmt
{

/*
*  Make a voxel clustering with _K clusters.
*/
Clustering::Clustering(Scene3DRenderer& scene3d, int K, bool initialOnly) :
	_scene3d(scene3d), _K(K), _show_initial(initialOnly)
{
	//Only voxels of (approximately) torso height are used for the color models,
	// and for the inital labeling in each tracking step.
	//This is to remove the noise (shadows) around the feet, and the face that most jeans have similar colors.
	//These voxels drawn shown as black by Glut.cpp
	_min_z = 750;
	_max_z = 1450;

	//Estimated radius of a person. Used for occlusion handling, where people are approximated by cylinders.
	_person_radius = 350.0f;
	//The frame that is used to initialize the color models
	_init_frame = 0;

	//Set the colors corresponding to the labels, using float color values (which Glut uses)
	drawcolors.push_back(Scalar(1.0, 0.0, 0.0));
	drawcolors.push_back(Scalar(0.0, 1.0, 0.0));
	drawcolors.push_back(Scalar(0.0, 0.0, 1.0));
	drawcolors.push_back(Scalar(0.0, 1.0, 1.0));
	unlabeledColor = Scalar(.5, .5, .5);

	//Set the (color)models vector to the amount of tracked clusters

	//Keep the camera's stored for future use
	_cams = _scene3d.getCameras();
	initializeColorModel();

	//To write the cluster center position to a text file, for the entire video,
	//  simply uncomment the next line  --v
	//makeTextFile();
}


Clustering::~Clustering(void)
{
}


/*
*  Initialize the color models that will be used to track the voxels.
*  This is done by processing the first frame (where everything should be well-visible)
*  and clustering the resulting voxels using K-means.
*  A color model is then built for each initial cluster obtained this way.
*/
void Clustering::initializeColorModel()
{
	//Process the first frame
	_scene3d.setCurrentFrame(_init_frame);
	_scene3d.processFrame();
	_scene3d.getReconstructor().update();
	//Get the active voxels for the first frame
	vector<Reconstructor::Voxel*> voxels = _scene3d.getReconstructor().getVisibleVoxels();



	//Initialize the matrices.
	//As k-means input we need a [#voxels x #dimensions] matrix
	Mat voxelPoints = Mat::zeros(voxels.size(), 2, CV_32F),
		labels,
		centers;

	//Put the (x,y) positions of the active voxels in the matrix,
	// as floats (to allow a cluster center to be inbetween voxels).
	for (int i = 0; i < voxels.size(); i++)
	{
		//Make a row for each voxel
		float data[2] = { (float) voxels[i]->x, (float) voxels[i]->y };
		Mat point(1, 2, CV_32F, &data);

		//Add the row values to the (zero-initialized) matrix, simply using '=' won't work
		voxelPoints.row(i) += point;
	}

	//Attempt to get the clustering right (no local minimum),
	//  after 10 unsuccessful attemps just continue with the result.
	for (int attempt = 1; attempt <= 10; attempt++)
	{
		cout << endl << endl << "Starting clustering attempt " << attempt << endl;

		//Use k-means, stopping after 10 iterations or center movements of smaller than 1.0, with 4 attempts. 
		kmeans(voxelPoints, _K, labels, TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
			10, 1.0), 4, KMEANS_PP_CENTERS, centers);


		//Write the centers as debug information
		for (int i = 0; i < centers.rows; i++)
		{
			cout << "Center " << i << " at " << centers.at<float>(i,0) << ", " << centers.at<float>(i,1)
				<< endl;
		}


		//     ------------------------     Cluster Visualisation     -----------------------------

		//Draw a top-down view of the voxel clusters,
		// to get an idea of how successful the clustering was
		Mat clusterImage = Mat(600, 600, CV_8UC3); //Mat::zeros(600, 600, CV_8UC3);
		clusterImage = Scalar::all(0);

		Scalar colors[] = {
			Scalar(255, 0, 0),
			Scalar(0, 255, 0),
			Scalar(0, 0, 255),
			Scalar(0, 255, 255)
		};
		//Scaling and offset from voxel (x,y) space to image space (to make it fit nicely in the image)
		Point2f offset = Point2f(300.0, 300.0);
		//The voxel x and y range from -512*4 to 512*4
		float scale = 600.0 / (512 * 8);

		//Draw each voxel, projected on the ground plane (height was not used for clustering),
		//  with the color of it's cluster
		for (int i = 0; i < voxelPoints.rows; i++)
		{
			//Calculate the position in the image (image y axis is reversed!)
			Point2f position = offset + (scale * Point2f(voxelPoints.at<float>(i, 0),
														 -voxelPoints.at<float>(i, 1)));
			int cluster = labels.at<int>(i);
			voxels[i]->cluster = cluster;
			
			//Draw the voxel as a circle
			circle(clusterImage, position, 2, colors[cluster], CV_FILLED, 8);			
		}

		//Draw the cluster centers
		_prev_centers = vector<Point2f>(_K);
		for (int i = 0; i < centers.rows; i++)
		{
			//Calculate the position in the image (image y axis is reversed!)
			Point2f position = offset + (scale * Point2f(centers.at<float>(i, 0),
														 -centers.at<float>(i, 1)));

			//Give each center a ring of white, for more visibility
			circle(clusterImage, position, 5, Scalar(255, 255, 255), CV_FILLED, 8);
			circle(clusterImage, position, 4, colors[i], CV_FILLED, 8);

			//Also save them for the first round of occlusion handling
			_prev_centers[i] = Point2f(centers.at<float>(i, 0), centers.at<float>(i, 1));
			circle(clusterImage, position, _person_radius * scale, Scalar(255, 255, 255), 3, 8);
		}

		//Check if the clustering attempt succeeded
		if (isLocalMinimum(centers))
		{
			imshow("Attempt resulted in local minimum" + attempt, clusterImage);
		}
		else
		{
			imshow("Successful initial clustering", clusterImage);
			break;
		}
	}


	//     ------------------------     Building the color models     ----------------------------

	//Separate the voxels into one voxel-vector per cluster
	vector<vector<Reconstructor::Voxel*>> clusters (_K);

	for (int v = 0; v < voxels.size(); v++)
	{
		//Add voxel #v to the cluster for which it was labeled,
		// but only if it is of torso height, because these voxels will give good distinctive colors.
		if ((voxels[v]->z >= _min_z) && (voxels[v]->z <= _max_z))
		{
			clusters[labels.at<int>(v)].push_back(voxels[v]);
			voxels[v]->cluster = labels.at<int>(v);
		}
		else
		{
			voxels[v]->cluster = -1;
		}
	}

	//Before building color models, process occlusions.
	processOcclusions(voxels);

	//Now make a color model for each cluster
	for (int m = 0; m < _K; m++)
	{
		//Use a Hue Histogram as color model
		_models.push_back(ColorHistogram(getVoxelColorsBunchedHSV(clusters[m]), 30, m));	
		//Visualize the color models, so we know what we are working with
		_models[m].visualisationImage();
	}

}


//Does the full voxel clustering / tracking for one frame of the video
//The result is that each voxel will have received the color attribute of its corresponding cluster.
//Voxels colored gray have not been classified (e.g. when doing only the initial labeling round).
//Returns the cluster centers.
vector<Point2f> Clustering::processFrame()
{
	//Take the visible voxels for this frame
	vector<Reconstructor::Voxel*> voxels = _scene3d.getReconstructor().getVisibleVoxels();
	processOcclusions(voxels);
		
	//Get the current frames of the cameras, in HSV, to determine voxel colors
	vector<Mat> BGRframes (_cams.size());
	vector<Mat> frames (_cams.size());
	for (int c = 0; c < _cams.size(); c++)
	{
		BGRframes[c] = _cams[c]->getFrame();
		//Convert each frame from BGR to HSV
		frames[c] = BGRframes[c].clone();
		cvtColor(frames[c], frames[c], CV_BGR2HSV);
	}
	
	//Start putting the 2d voxel position in a matrix, this can be used for kmeans (if desired)
	Mat voxeldata = Mat::zeros(voxels.size(), 2, CV_32F);
	Mat centers;
	vector<int> labels = vector<int>(voxels.size());

	//The third dimension of average is used for the amount of samples over which the average is taken
	vector<Point3i> average;
	//Start calculating the center (avg position in 2d) of each of the clusters
	for (int i=0; i<_K; i++)
	{
		average.push_back(Point3i(0,0,0));
	}

	//Compare voxels to color models	
	for(int v = 0; v < voxels.size(); v++)
	{
		//Take voxel 2d position
		Point2f position = Point2f((float) voxels[v]->x, (float) voxels[v]->y);
		float data[2] = { (float) position.x, (float) position.y };
		Mat point(1, 2, CV_32F, &data);
		//Add the row values to the (zero-initialized) matrix, simply using '=' won't work
		voxeldata.row(v) += point;
		vector<float> divergence;

		//Get the (HSV) colors belonging to the voxel
		vector<Scalar> voxelColors = getVoxelColors(voxels[v], v, frames);

		//If for some reason no colors are returned (completely occluded or wrong height),
		// don't use the voxel for initial labeling and make it gray.
		if (voxelColors.empty())
		{
			voxels[v]->color = unlabeledColor;
			voxels[v]->cluster = -1;
		}
		else
		{
			//Compare the voxel colos to each color model
			for (int k = 0; k< _K; k++)
			{
				int size = voxelColors.size();
				float count = 0;
				for (int c =0; c<size; c++)
				{
					count += _models[k].distanceTo(voxelColors[c]); 
				}

				float averageDivergence = count/size;
				divergence.push_back(averageDivergence);
			}
			//Find the index of the color model that fits best
			int index = 0;
			float lowestdivergence = 9999999999.9f; //should be infinity, arbitrary large number should work though.
			
			for (int i=0; i<_K; i++)
			{	
				if (divergence[i] < lowestdivergence){
					index = i;
					lowestdivergence = divergence[i];
				}
			}
			labels[v] = index;

			//Set the voxel color
			voxels[v]->color = drawcolors[index];
			voxels[v]->cluster = index;

			average[index] += Point3i(voxels[v]->x, voxels[v]->y, 1);
		}
			
	}

	//Calculate the initial cluster centers
	vector<Point2f> initialCenters;
	for (int i = 0; i < _K; i++)
	{
		float avgX = (float) average[i].x / (float) average[i].z;
		float avgY = (float) average[i].y / (float) average[i].z;
		initialCenters.push_back( Point2f(avgX, avgY) );
	}


	//   ---   Recluster and relabel the voxels

	if (_show_initial)
	{
		_prev_centers = vector<Point2f>(initialCenters);
		return initialCenters;
	}


	//Get ready to calculate new cluster centers
	//The third dimension of newAverage is used for the amount of samples over which the average is taken
	vector<Point3i> newAverage;	
	for (int i=0; i<_K; i++)
	{
		newAverage.push_back(Point3i(0,0,0));
	}

	//Do a round of (re-)labeling the voxels to the closest initial cluster center
	for (int v = 0; v < voxels.size(); v++)
	{
		double shortestDistance = 999999.9;
		int closestCenter = 0;
		Point2f position = Point2f((float) voxels[v]->x, (float) voxels[v]->y);

		for (int i=0; i<_K; i++)
		{
			//Check if this center is closer, using Euclidean distance in the x-y plane
			if (norm(initialCenters[i] - position) < shortestDistance)
			{
				closestCenter = i;
				shortestDistance = norm(initialCenters[i] - position);
			}
		}

		//Re-label
		//labels[v] = closestCenter;
		//Re-set the voxel color
		voxels[v]->color = drawcolors[closestCenter];
		voxels[v]->cluster = closestCenter;
		//Start calculating the new cluster centers
		newAverage[closestCenter] += Point3i(voxels[v]->x, voxels[v]->y, 1);
	}

	//Now recalculate the cluster centers
	for (int i=0; i<_K; i++)
	{
		float avgX = (float) newAverage[i].x / (float) newAverage[i].z;
		float avgY = (float) newAverage[i].y / (float) newAverage[i].z;
		centers.push_back( Point2f(avgX, avgY) );
	}

	///Finally, it is possible to add a few extra iteration of kmeans (simply uncomment the next line)
	//kmeans(voxeldata,_K,labels,TermCriteria(CV_TERMCRIT_ITER, 3, 1.0), 2, KMEANS_USE_INITIAL_LABELS, centers);
	
	_prev_centers = vector<Point2f>(centers);
	return centers;
}



//Create a text file which shows the movement of the cluster centers.
void Clustering::makeTextFile()
{

	ofstream textOutput;
	textOutput.open("textoutput.txt");
	//The moving position of each center is written to one line
	vector<stringstream> lines = vector<stringstream>(_K);

	//iterate through all of the frames
	for (int f = 0 ; f < _scene3d.getNumberOfFrames(); f++)
	{
		_scene3d.setCurrentFrame(f);
		//Process frame
		_scene3d.processFrame();
		_scene3d.getReconstructor().update();
		//Get the centers
		vector<Point2f> centers = processFrame();

		//Add the centers to each of the lines
		for (int i = 0; i < _K; i++)
		{
			lines[i] << to_string((int) floor(centers[i].x)) << " " << to_string((int) floor(centers[i].y));
			if (f != _scene3d.getNumberOfFrames() - 1)
			{
				lines[i] << ",";
			}
		}
	}

	//Combine the lines
	for (int i = 0; i < _K; i++)
	{
		textOutput << lines[i] << endl;
	}
	textOutput.close();
}


//Reports if the clustering ended up in a local minimum,
//  this is an educated guess based on the locations of the cluster centers
bool Clustering::isLocalMinimum(Mat& centers)
{
	//Check the distances between each pair of cluster centers
	for (int i = 0; i < centers.rows; i++)
	{
		for (int j = 0; j < centers.rows; j++)
		{
			//Don't compare a cluster center with itself
			if (i==j)
				continue;

			Point2f center1 = Point2f(centers.at<float>(i, 0), centers.at<float>(i, 1));
			Point2f center2 = Point2f(centers.at<float>(j, 0), centers.at<float>(j, 1));
			float distance = (float) norm(center1 - center2);

			//If the distance is unusually low, it is probably a local minimum
			if (distance < 450.0)
			{
				return true;
			}
		}
	}
	//If no two cluster centers were too close, the clustering was probably OK
	return false;
}


//This function does the occlusion handling, checking for each voxel for each camera if it is occluded (something is in front).
//A person is approximated by a cylinder with radius _person_radius and center at the cluster center of previous frame.
//If a line from a voxel to the camera intersects with ANOTHER person, it is occluded for that camera.
//The result is a rough estimation, where 'inner body voxels' are not occluded by their own cylinder, but this is not a big problem,
//  because they belong to the same person as the 'skin-level voxels' anyway.
void Clustering::processOcclusions(vector<Reconstructor::Voxel*> voxels)
{
	//Determine occlusions for each camera
	for (int c = 0; c < _cams.size(); c++)
	{
		//Calculate the (2d) vector from camera to each cluster center.
		Point2f camPosition = Point2f(_cams[c]->getCameraLocation().x, _cams[c]->getCameraLocation().y);
		vector<Point2f> camToCenter = vector<Point2f>(_K);
		for (int i = 0; i < _K; i++)
		{
			camToCenter[i] = _prev_centers[i] - camPosition;
		}

		//For each voxel..
		for (int v = 0; v < voxels.size(); v++)
		{
			//Initially set occlusion to false
			voxels[v]->occluded_from_camera[c] = false;

			//Check if the voxel is actually in the camera's FoV
			if (voxels[v]->valid_camera_projection[c])
			{
				//Calculate camera to voxel vector (normalized to unit length)
				Point2f camToVoxel = Point2f((float) voxels[v]->x, (float) voxels[v]->y) - camPosition;
				float length = norm(camToVoxel);
				Point2f camToVoxelNormalized = camToVoxel * (1/length);

				for (int i = 0; i < _K; i++)
				{
					//A voxel can not be occluded by it's own cluster, or a cluster further from the camera
					if (i == voxels[v]->cluster || length < norm(camToCenter[i]))
					{
						continue;
					}

					//Calculate the projection of vector cam->center onto vector cam->voxel,
					//  this is the point on the ray from cam to voxel that is closest to the center
					Point2f parallelProjection = camToCenter[i].dot(camToVoxelNormalized) * camToVoxelNormalized;
					float perpDistance = (float) norm(parallelProjection - camToCenter[i]);

					//If this point is within the radius of a person, the voxel is occluded
					if (perpDistance < _person_radius)
					{
						voxels[v]->occluded_from_camera[c] = true;
					}
				}
			}
		}
		
	}
	
}


//Determines the voxel's corresponding pixel colors,
//  found on the given frames (one for each camera).
vector<Scalar> Clustering::getVoxelColors(Reconstructor::Voxel* voxel, int voxelNr, vector<Mat> frames)
{
	//Only the colors of torso-height voxels are used, because those have distinctive colors
	if ((voxel->z < _min_z) || (voxel->z > _max_z))
	{
		return vector<Scalar>();
	}

	//Pixel colors for this voxel
	vector<Scalar> colors;

	for (int c = 0; c < frames.size(); c++)
	{
		//Check if the voxel is within the camera angle and not occluded
		if (voxel->valid_camera_projection[c] && !voxel->occluded_from_camera[c])
		{
			Point pixel = voxel->camera_projection[c];
			Vec3b values = frames[c].at<Vec3b>(pixel);
			colors.push_back(Scalar(values[0], values[1], values[2]));
		}
	}

	return colors;
}


//Determines the voxels' corresponding pixel colors (just like getVoxelColorsBGR),
//  but returns all BGR color values bunched together in a single vector.
//This is more useful for building a color model, instead of tracking single voxels.
//Contains an extra conversion step to go from BGR to HSV
vector<Scalar> Clustering::getVoxelColorsBunchedHSV(vector<Reconstructor::Voxel*> voxels)
{
	//First, get the current frames of the cameras, in HSV
	vector<Mat> BGRframes (_cams.size());
	vector<Mat> frames (_cams.size());
	for (int c = 0; c < _cams.size(); c++)
	{
		BGRframes[c] = _cams[c]->getFrame();
		//Convert each frame from BGR to HSV, to get HSV values for the voxels
		frames[c] = BGRframes[c].clone();
		cvtColor(frames[c], frames[c], CV_BGR2HSV);
	}


	//Then, gather all pixel colors corresponding to any of the given voxels
	vector<Scalar> colors;
	for (int v = 0; v < voxels.size(); v++)
	{
		for (int c = 0; c < _cams.size(); c++)
		{
			//Check if the voxel is within the camera angle and not occluded
			if (voxels[v]->valid_camera_projection[c] && !voxels[v]->occluded_from_camera[c])
			{
				Point pixel = voxels[v]->camera_projection[c];
				Vec3b values = frames[c].at<Vec3b>(pixel);
				colors.push_back(Scalar(values[0], values[1], values[2]));
			}
		}
	}

	return colors;
}

} // end namespace nl_uu_science_gmt