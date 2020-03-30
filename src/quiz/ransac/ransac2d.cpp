/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	// Randomly sample subset and fit line
  for (int i=0; i < maxIterations; i++) {
    std::unordered_set<int> inliers; // makes sure to have unique elements, contain 2 indices
    for (int j=0; j < 3; j++) {
      inliers.insert(rand() % (cloud->points.size()));
    }

    // general line equation is Ax + By + C = 0
    // Given two points (x1, y1), (x2, y2), the line through those two points is
    // (y1-y2)*x | (x1-x2)*y + (x1*y2 - x2*y1) = 0
    
    // general plane equation is Ax + By + Cz + D = 0
    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    auto itr = inliers.begin();
    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    z1 = cloud->points[*itr].z;
    itr++;
    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;
    z2 = cloud->points[*itr].z;
    itr++;
    x3 = cloud->points[*itr].x;
    y3 = cloud->points[*itr].y;
    z3 = cloud->points[*itr].z;
    
    float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
    float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
    float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
    float d = -(a*x1 + b*y1 + c*z1);

    // interate through all point clouds
    for (int index=0; index < cloud->points.size(); index++) {
      if (inliers.count(index)>0) {
        continue;
      }

      // Measure distance between every point and fitted line
      // distance formula d = |Ax + By + C| / sqrt(A^2 + B^2)
      // distance point to plane: |Ax+By+Cz+D| / sqrt(A^2 + B^2 + C^2)
      pcl::PointXYZ point = cloud->points[index];
      float d = fabs(a*point.x + b*point.y + c*point.z + d) / sqrt(a*a + b*b + c*c);

      // If distance is within threshold, count it as inlier
      if (d <= distanceTol) {
        inliers.insert(index);
      }
    }

    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }

  }

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  else
  {
  	renderPointCloud(viewer,cloud,"data");
  }
	
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
  }
  	
}
