#include "Clustering.h"

namespace nl_uu_science_gmt
{

/*
*  Make a voxel clustering with _K clusters.
*/
Clustering::Clustering(Scene3DRenderer& scene3d, int K) :
	_scene3d(scene3d), _K(K)
{
	initializeColorModel();
}


Clustering::~Clustering(void)
{
}

/*
*  Initialize the color model that will be used to separate the voxels
*  into K different clusters. This is done by processing the frist frame
*  (where everything should be well-visible) and clustering the resulting voxels
*  using K-means.
*  A color model is then built for each initial cluster obtained this way.
*/
void Clustering::initializeColorModel()
{
	//Process the first frame
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

	//Use k-means, stopping after 10 iterations or center movements of smaller than 1.0, with 4 attempts. 
	kmeans(voxelPoints, _K, labels, TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
		10, 1.0), 4, KMEANS_PP_CENTERS, centers);

	//Write the centers as debug information
	for (int i = 0; i < centers.rows; i++)
	{
		cout << "Center " << i << " at " << centers.at<float>(i,0) << ", " << centers.at<float>(i,1)
			<< endl << endl;
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
		Point2f position = offset + (scale * Point2f(voxelPoints.at<float>(i, 0),
													 voxelPoints.at<float>(i, 1)));
        int cluster = labels.at<int>(i);
        
		//Draw the voxel as a circle
        circle(clusterImage, position, 2, colors[cluster], CV_FILLED, 8);
    }

	//Draw the cluster centers
	for (int i = 0; i < centers.rows; i++)
	{
		Point2f position = offset + (scale * Point2f(centers.at<float>(i, 0),
													 centers.at<float>(i, 1)));

		//Give each center a ring of white, for more visibility
		circle(clusterImage, position, 5, Scalar(255, 255, 255), CV_FILLED, 8);
		circle(clusterImage, position, 4, colors[i], CV_FILLED, 8);
	}

	imshow("Initial clusters", clusterImage);

}

} // end namespace nl_uu_science_gmt