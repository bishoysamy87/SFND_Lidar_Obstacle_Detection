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
	std::unordered_set<int> tempInliersResult;
	srand(time(NULL));
	int num_inliners = 0;
	int max_inliners = 0;
	
	// TODO: Fill in this function

	// For max iterations 
	for (int itr = 0; itr < maxIterations;itr++)
	{
		 num_inliners = 0;
		tempInliersResult.clear();
	// Randomly sample subset and fit line
		int idx1 = rand() % cloud->points.size();
		int idx2 = rand() % cloud->points.size();
		int idx3 = rand() % cloud->points.size();
		
		double y1 = cloud->points[idx1].y;
		double x1 = cloud->points[idx1].x;
		double z1 = cloud->points[idx1].z;
		
		double y2 = cloud->points[idx2].y;
		double x2 = cloud->points[idx2].x;
		double z2 = cloud->points[idx2].z;
		
		double y3 = cloud->points[idx3].y;
		double x3 = cloud->points[idx3].x;
		double z3 = cloud->points[idx3].z;
		
		
		double A = ((y2-y1)*(z3-z1))- ((z2-z1)*(y3-y1));
		double B = ((z2-z1)*(x3-x1))- ((x2-x1)*(z3-z1));
		
		double C = ((x2-x1)*(y3-y1))- ((y2-y1)*(x3-x1));
		double D = -(A*x1+B*y1+C*z1);
		
		//cout<<"A = "<<A<<" B = " << B<<" C= " << C << std::endl;
		double den = sqrt(A*A + B*B + C*C);
	// Measure distance between every .point and fitted line
		for (int pts = 0; pts < cloud->points.size(); pts++)
		{
			
			double d = abs(A * cloud->points[pts].x + B*cloud->points[pts].y + C * cloud->points[pts].z + D) / den;
			if(d < distanceTol)
			{
				num_inliners++;
				tempInliersResult.insert(pts);
				
			}
		}
		if(num_inliners > max_inliners)
		{
			max_inliners = num_inliners;
			cout<< "max_inliners  = " << max_inliners << std::endl;
			inliersResult = tempInliersResult;
		}
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
		
	}
	

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
