#include "Clustering.h"
#include <iostream>
#include <fstream>

namespace nl_uu_science_gmt
{

/*
*  Make a voxel clustering with _K clusters.
*/
Clustering::Clustering(Scene3DRenderer& scene3d, int K) :
	_scene3d(scene3d), _K(K)
{
	//Only voxels of (approximately) torso height are used for the color models,
	// and for the inital labeling in each tracking step.
	//This is to remove the noise (shadows) around the feet, and the face that most jeans have similar colors.
	//These voxels drawn shown as black by Glut.cpp
	_min_z = 750;
	_max_z = 1450;

	//Set the (color)models vector to the amount of tracked clusters
	//_models.resize(_K);
	//Keep the camera's stored for future use
	_cams = _scene3d.getCameras();
	initializeColorModel();
	RunTracking();
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
	_scene3d.setCurrentFrame(0);
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
			
			//Draw the voxel as a circle
			circle(clusterImage, position, 2, colors[cluster], CV_FILLED, 8);
		}

		//Draw the cluster centers
		for (int i = 0; i < centers.rows; i++)
		{
			//Calculate the position in the image (image y axis is reversed!)
			Point2f position = offset + (scale * Point2f(centers.at<float>(i, 0),
														 -centers.at<float>(i, 1)));

			//Give each center a ring of white, for more visibility
			circle(clusterImage, position, 5, Scalar(255, 255, 255), CV_FILLED, 8);
			circle(clusterImage, position, 4, colors[i], CV_FILLED, 8);
		}

		//Check if the clustering attempt succeeded
		if (isLocalMinimum(centers))
		{
			imshow("Attempt resulted in local minimum" + attempt, clusterImage);
		}
		else
		{
			imshow("Successful clustering attempt", clusterImage);
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
		}
	}

	//Now make a color model for each cluster
	for (int m = 0; m < _K; m++)
	{
		//Use a Hue Histogram as color model
		_models.push_back(ColorHistogram(getVoxelColorsBunchedHSV(clusters[m]), 30, m));	
		//Visualize the color models, so we know what we are working with
		_models[m].visualisationImage();
	}

}

void Clustering::RunTracking()
{
	//create video writer
	VideoWriter outputVideo;
	outputVideo.open("initial.avi",-1,2, Size(600,600));
	VideoWriter outputVideoPostKmeans;
	outputVideoPostKmeans.open("relabeled.avi",-1,2, Size(600,600));

	if (!outputVideo.isOpened() || !outputVideoPostKmeans.isOpened())
    {
        cout  << "Could not open the output video for write: " << endl;
        return;
    }

	Mat clusterImage = Mat(600, 600, CV_8UC3); //Mat::zeros(600, 600, CV_8UC3);
	Scalar drawcolors[] = {
			Scalar(255, 0, 0),
			Scalar(0, 255, 0),
			Scalar(0, 0, 255),
			Scalar(0, 255, 255)
		};
	//Scaling and offset from voxel (x,y) space to image space (to make it fit nicely in the image)
	Point2f offset = Point2f(300.0, 300.0);
	//The voxel x and y range from -512*4 to 512*4
	float scale = 600.0 / (512 * 8);

	ofstream textOutput;
	textOutput.open("textoutput.txt");


	//iterate through all of the frames
	for (int f = 0 ; f < _scene3d.getNumberOfFrames(); f++)
	{

		clusterImage = Scalar::all(0);
		_scene3d.setCurrentFrame(f);
		//Process frame
		_scene3d.processFrame();
		_scene3d.getReconstructor().update();
		//Get the active voxels 
		vector<Reconstructor::Voxel*> voxels = _scene3d.getReconstructor().getVisibleVoxels();

		_cams = _scene3d.getCameras();
		_cams[0]->setVideoFrame(f);
		_cams[1]->setVideoFrame(f);
		_cams[2]->setVideoFrame(f);
		_cams[3]->setVideoFrame(f);
		
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
		
		Mat voxeldata = Mat::zeros(voxels.size(), 2, CV_32F);
		Mat centers;

		vector<int> labels = vector<int>(voxels.size());
		//The third dimension of average is used for the amount of samples over which the average is taken
		vector<Point3i> average;
		
		for (int i=0; i<_K; i++)
		{
			average.push_back(Point3i(0,0,0));
		}

		//compare voxels to color models	
		for(int v = 0; v<voxels.size(); v++)
		{

			Point2f position = offset + (scale * Point2f(voxels[v]->x,voxels[v]->y));
			float data[2] = { (float) position.x, (float) position.y };
			Mat point(1, 2, CV_32F, &data);
			//Add the row values to the (zero-initialized) matrix, simply using '=' won't work
			voxeldata.row(v) += point;

			//For the initial labeling, only use voxels of torso-height, because they have distinctive colors
			if ((voxels[v]->z >= _min_z) && (voxels[v]->z <= _max_z))
			{
				vector<float> divergence;

				//Get the (HSV) colors belonging to the voxel
				vector<Scalar> VoxelColors = getVoxelColors(voxels[v], frames);

				for (int k = 0; k< _K; k++)
				{
					int size = VoxelColors.size();
					float count = 0;
					for (int c =0; c<size; c++)
					{
						count+= _models[k].distanceTo(VoxelColors[c]); 
					}

					float averageDivergence = count/size;
					divergence.push_back(averageDivergence);
				}
				int index =0;
				float lowestdivergence = 9999999999; //should be infinity, arbitrary large number should work though.
			
				for (int i=0; i<_K; i++)
				{	
					if (divergence[i] < lowestdivergence){
						index = i;
						lowestdivergence = divergence[i];
					}
				}
				labels[v] = index;
				average[index] = Point3i(voxels[v]->x+ average[index].x,voxels[v]->y + average[index].y, average[index].z +1);
			
				circle(clusterImage, position, 2, drawcolors[index], CV_FILLED, 8);
				//add point to image.
			}
			//For the voxels that aren't used for the initial clustering, just draw them as grey circles.
			else
			{	
				circle(clusterImage, position, 2, Scalar(125, 125, 125), CV_FILLED, 8);
			}
			
		}


		//Calculate and draw the initial cluster centers
		vector<Point2f> initialCenters;
		for (int i=0; i<_K; i++)
		{
			average[i].x = average[i].x / average[i].z;
			average[i].y = average[i].y / average[i].z;

			Point2f position = offset + (scale * Point2f(average[i].x, average[i].y));
			initialCenters.push_back(position);
				
			//Draw the center of each core (center of label/group/cluster) as a circle	
			circle(clusterImage, position, 5, Scalar(255, 255, 255), CV_FILLED, 8);
			circle(clusterImage, position, 2, drawcolors[i], CV_FILLED, 8);

		}


		//imshow("test", clusterImage);
		outputVideo.write(clusterImage);
		cout<< "writing video frame: "<< f+1 << "/" << _scene3d.getNumberOfFrames() <<endl;


		//   ---   Recluster and relabel the voxels


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
			Point2f position = offset + (scale * Point2f(voxels[v]->x,voxels[v]->y));

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
			labels[v] = closestCenter;
			//Start calculating the new cluster centers
			newAverage[closestCenter] += Point3i(voxels[v]->x, voxels[v]->y, 1);
		}

		textOutput<< "frame:"<< f << " -- ";

		//Now recalculate the cluster centers
		for (int i=0; i<_K; i++)
		{
			newAverage[i].x = newAverage[i].x / newAverage[i].z;
			newAverage[i].y = newAverage[i].y / newAverage[i].z;

			Point2f position = offset + (scale * Point2f(newAverage[i].x, newAverage[i].y));
			centers.push_back(position);
			//Also write the core (cluster center) positions to the text file
			textOutput<<"Core:"<< i+1<< "  X:"<< position.x << " Y:" << position.y << "  ||  ";
		}
		textOutput<<endl;

		//kmeans(voxeldata,_K,labels,TermCriteria(CV_TERMCRIT_ITER, 3, 1.0), 1, KMEANS_USE_INITIAL_LABELS, centers);
		clusterImage = Scalar::all(0);

		//Scaling and offset from voxel (x,y) space to image space (to make it fit nicely in the image)
		Point2f offset = Point2f(300.0, 300.0);
		//The voxel x and y range from -512*4 to 512*4
		float scale = 600.0 / (512 * 8);

		//Draw each voxel, projected on the ground plane (height was not used for clustering),
		//  with the color of its cluster
		for (int i = 0; i < voxeldata.rows; i++)
		{
			//Calculate the position in the image (image y axis is reversed!)
			Point2f position = Point2f(voxeldata.at<float>(i, 0),voxeldata.at<float>(i, 1));
			int cluster = labels[i];
			
			//Draw the voxel as a circle
			circle(clusterImage, position, 2, drawcolors[cluster], CV_FILLED, 8);
		}

		//Draw the cluster centers
		for (int i = 0; i < centers.rows; i++)
		{
			//Calculate the position in the image (image y axis is reversed!)
			Point2f position =  Point2f(centers.at<float>(i, 0),centers.at<float>(i, 1));

			//Give each center a ring of white, for more visibility
			circle(clusterImage, position, 5, Scalar(255, 255, 255), CV_FILLED, 8);
			circle(clusterImage, position, 4, drawcolors[i], CV_FILLED, 8);
		}

		outputVideoPostKmeans.write(clusterImage);
		cout<< "writing 2nd video frame: "<< f+1 << "/" << _scene3d.getNumberOfFrames() <<endl;

	
	}
	outputVideo.release();
	outputVideoPostKmeans.release();
	textOutput.close();

	
	//compare color of voxel to all colormodels
	//draw result to video
	
	//kmeans the voxels
	//
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
			float distance = norm(center1 - center2);
			cout << "d( c" << i << ", c" << j << " ) = " << distance << endl;

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

//Determines the voxel's corresponding pixel colors,
//  found on the given frames (one for each camera).
vector<Scalar> Clustering::getVoxelColors(Reconstructor::Voxel* voxel, vector<Mat> frames)
{
	//Pixel colors for this voxel
	vector<Scalar> colors;

	for (int c = 0; c < frames.size(); c++)
	{
		//Check if the voxel is within the camera angle and not occluded
		//TODO: add occlusion detection
		if (voxel->valid_camera_projection[c])
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
			//TODO: add occlusion detection
			if (voxels[v]->valid_camera_projection[c])
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