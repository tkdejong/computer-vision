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
		//Add voxel #v to the cluster for which it was labeled
		clusters[labels.at<int>(v)].push_back(voxels[v]);
	}

	//Now make a color model for each cluster
	for (int m = 0; m < _K; m++)
	{
		//Use a simple 'mean color' model for each cluster
		//TODO implement more advanced color model (like GMM)
		_models.push_back(MeanColorModel(getVoxelColorsBunchedBGR(clusters[m]), m));
		
		//Visualize the color models
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
		
		Mat voxeldata = Mat::zeros(voxels.size(), 2, CV_32F);
		//Mat labels;
		Mat centers;

		vector<Scalar> colors =getVoxelColorsBunchedBGR(voxels);
		vector<int> templabels = vector<int>();
		vector<Point3i> average;
		
		for (int i=0; i<_K; i++)
			{
				average.push_back(Point3i(0,0,0));
		}
		//compare voxels to 
		
		for(int v = 0; v<voxels.size(); v++)
		{
			vector<float> divergence;
			for (int k = 0; k< _K; k++)
			{
//				float a = _models[k].distanceTo(colors[(v*4)]);
//				float b = _models[k].distanceTo(colors[(v*4)+1]);
				float c = _models[k].distanceTo(colors[(v*4)+2]);
//				float d = _models[k].distanceTo(colors[(v*4)+3]);

				float averageDivergence = c;//(a+b+c+d)/4;
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
			templabels.push_back(index);
			
			average[index] = Point3i(voxels[v]->x+ average[index].x,voxels[v]->y + average[index].y, average[index].z +1);
			
			
			Point2f position = offset + (scale * Point2f(voxels[v]->x,voxels[v]->y));
			
			circle(clusterImage, position, 2, drawcolors[index], CV_FILLED, 8);
			//add point to image.

			float data[2] = { (float) position.x, (float) position.y };
			Mat point(1, 2, CV_32F, &data);

			//Add the row values to the (zero-initialized) matrix, simply using '=' won't work
			voxeldata.row(v) += point;
		}

		

		textOutput<< "frame:"<< f<< " -- ";
		for (int i=0; i<_K; i++)
		{
			average[i].x = average[i].x / average[i].z;
			average[i].y = average[i].y / average[i].z;

			Point2f position = offset + (scale * Point2f(average[i].x, average[i].y));
		
			
			//Draw the core as a circle
		
		circle(clusterImage, position, 5, Scalar(255, 255, 255), CV_FILLED, 8);
		circle(clusterImage, position, 2, drawcolors[i], CV_FILLED, 8);
		textOutput<<"Core:"<< i+1<< "  X:"<< position.x << " Y:" << position.y << "  ||  ";
		}
		textOutput<<endl;
		int x  = 0;


		//imshow("test", clusterImage);
		outputVideo.write(clusterImage);
		cout<< "writing video frame: "<< f+1 << "/" << _scene3d.getNumberOfFrames() <<endl;

		//recluster and relabel voxels
		kmeans(voxeldata,_K,templabels,TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,10, 1.0), 4, KMEANS_USE_INITIAL_LABELS,centers);
	
		clusterImage = Scalar::all(0);

		
		//Scaling and offset from voxel (x,y) space to image space (to make it fit nicely in the image)
		Point2f offset = Point2f(300.0, 300.0);
		//The voxel x and y range from -512*4 to 512*4
		float scale = 600.0 / (512 * 8);

		//Draw each voxel, projected on the ground plane (height was not used for clustering),
		//  with the color of it's cluster
		for (int i = 0; i < voxeldata.rows; i++)
		{
			//Calculate the position in the image (image y axis is reversed!)
			Point2f position = Point2f(voxeldata.at<float>(i, 0),voxeldata.at<float>(i, 1));
			int cluster = templabels[i];
			
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

//Reprojects the voxels to each camera, to get the corresponding pixels.
//The BGR values of the visible pixels (not occluded, in vield of vision) are returned.
vector<vector<Scalar>> Clustering::getVoxelColorsBGR(vector<Reconstructor::Voxel*> voxels)
{
	//First, get the current frames of the cameras
	vector<Mat> frames (_cams.size());
	for (int c = 0; c < _cams.size(); c++)
	{
		frames[c] = _cams[c]->getFrame();
	}


	//Then, check the pixels for each voxel
	vector<vector<Scalar>> voxelColors;
	for (int v = 0; v < voxels.size(); v++)
	{
		//Pixel colors for this voxel
		vector<Scalar> colors;

		for (int c = 0; c < _cams.size(); c++)
		{
			//Check if the voxel is within the camera angle and not occluded
			//TODO: actually add occlusion detection
			if (voxels[v]->valid_camera_projection[c])
			{
				Point pixel = voxels[v]->camera_projection[c];
				Vec3b values = frames[c].at<Vec3b>(pixel);
				colors.push_back(Scalar(values[0], values[1], values[2]));
				cout << "color " << colors[0] << endl;
			}
		}

		voxelColors.push_back(colors);
	}
	
	return voxelColors;
}

//Determines the voxels' corresponding pixel colors (just like getVoxelColorsBGR),
//  but returns all BGR color values bunched together in a single vector.
//This is more useful for building a color model, instead of tracking single voxels.
vector<Scalar> Clustering::getVoxelColorsBunchedBGR(vector<Reconstructor::Voxel*> voxels)
{
	//First, get the current frames of the cameras
	vector<Mat> frames (_cams.size());
	for (int c = 0; c < _cams.size(); c++)
	{
		frames[c] = _cams[c]->getFrame();
	}


	//Then, gather all pixel colors corresponding to any of the given voxels
	vector<Scalar> colors;
	for (int v = 0; v < voxels.size(); v++)
	{
		for (int c = 0; c < _cams.size(); c++)
		{
			//Check if the voxel is within the camera angle and not occluded
			//TODO: actually add occlusion detection
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