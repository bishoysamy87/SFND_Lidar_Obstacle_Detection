// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <Eigen/Dense>
using namespace Eigen;


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
	
	// Create the filtering object
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (filterRes, filterRes,filterRes);
	sor.filter (*cloud_filtered);
	
	pcl::CropBox<PointT> region_interest(true);
	
	region_interest.setInputCloud (cloud_filtered);
	region_interest.setMin(minPoint);
	region_interest.setMax(maxPoint);
	region_interest.filter(*cloud_filtered);

	std::vector<int> indices;
	pcl::PointIndices::Ptr inliners (new pcl::PointIndices);
	
	region_interest.setInputCloud (cloud_filtered);
	region_interest.setMin( Eigen::Vector4f(-1.5,-1.7,-1,1));
	region_interest.setMax( Eigen::Vector4f(3,1.7,0,1));
	region_interest.filter(indices);
	
	for(int itr :indices)
	{
		inliners->indices.push_back(itr);
	}
	
	pcl::ExtractIndices<PointT> extract;
	
	extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliners);
    extract.setNegative (true);
    extract.filter (*cloud_filtered);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(typename pcl::PointCloud<PointT>::Ptr cloud,
			   std::vector<int>& is_processed,
			   KdTree<PointT> * tree,
			   float distanceTol,
			   int id,
			   std::vector<int>& cluster,int minSize, int maxSize)
{
	if(cluster.size()<maxSize && (is_processed[id] == 0))
	{
		is_processed[id] = 1;
		/**add this point to the cluster*/
		cluster.push_back(id);
	}
	else
	{
		std::cout<< "exceedt the maximum size"<<std::endl;
	}
	std::vector<int> nearby = tree->search(cloud->points[id],distanceTol);
	//std::cout<< "size of nearby = "<< nearby.size() << std::endl;
	
	for(int index : nearby)
	{
		//std::cout << index << ",";
		if(is_processed[index] == 0)
		{
			//std::cout<<"point = "<<index<<std::endl;
			Proximity(cloud, is_processed,tree ,distanceTol,index,cluster,minSize,maxSize);
		}
	}
	//std::cout << std::endl;
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol,int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<int> is_processed;
	
	is_processed.assign(cloud->points.size(),0);
	//std::cout<<"cloud size =  "<<cloud->points.size()<<std::endl;
	int num_clusters = 0;
	for (int itr = 0; itr < cloud->points.size(); itr++)
	{
		if(is_processed[itr] == 0)
		{
			//std::cout<<"point = "<<itr<<std::endl;
			std::vector<int> cluster;
			Proximity(cloud,is_processed,tree,distanceTol,itr,cluster,minSize,maxSize);
			num_clusters++;
			clusters.push_back(cluster);
			//std::cout<<"cluster added = "<<num_clusters<<std::endl;
		}
	}
 
	return clusters;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	KdTree<PointT>* tree= new KdTree<PointT>  ;
	tree->point_dim = 3;
	
	for(int id= 0; id< cloud->points.size();id++)
	{
		tree->insert (cloud->points[id],id);
	}
	
	
	std::vector<std::vector<int>> clusters_index = euclideanCluster(cloud, tree, clusterTolerance,minSize, maxSize);
	
	for(std::vector<int> cluster : clusters_index)
  	{
		if(cluster.size() < minSize || cluster.size() > maxSize)
			continue;
  		typename pcl::PointCloud<PointT>::Ptr cloud_cluster ( new pcl::PointCloud<PointT>);
		//std::cout<<"cluster num points = "<< cluster.size()<<std::endl;
  		for(int indice: cluster)
  			cloud_cluster->points.push_back(cloud->points[indice]);
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		clusters.push_back(cloud_cluster);
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);


    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBox2(typename pcl::PointCloud<PointT>::Ptr cluster)
{
	PointT minPoint, maxPoint;
    // Find bounding box for one of the clusters
	// Compute principal directions
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cluster, pcaCentroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
																					///    the signs are different and the box doesn't get correctly oriented in some cases.
	BoxQ box;
	/* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
	// In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
	*/
	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
	typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
	pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
	const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	box.bboxTransform = bboxTransform;
	box.bboxQuaternion = bboxQuaternion;
	
	box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width =  maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}



template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
		Vector3d v((x2-x1),(y2-y1),(z2-z1));
        Vector3d w((x3-x1),(y3-y1),(z3-z1));
		
		Vector3d res = v.cross(w);
	
		
		double A = res[0];
		double B = res[1];
		
		double C = res[2];
		double D = -(A*x1+B*y1+C*z1);
		
		//cout<<"A = "<<A<<" B = " << B<<" C= " << C << std::endl;
		double den = sqrt(A*A + B*B + C*C);
	// Measure distance between every .point and fitted line
		for (int pts = 0; pts < cloud->points.size(); pts++)
		{
			
			double d = fabs(A * cloud->points[pts].x + B*cloud->points[pts].y + C * cloud->points[pts].z + D) / den;
			if(d < distanceTol)
			{
				num_inliners++;
				tempInliersResult.insert(pts);
				
			}
		}
		if(num_inliners > max_inliners)
		{
			max_inliners = num_inliners;
			//cout<< "max_inliners  = " << max_inliners << std::endl;
			inliersResult = tempInliersResult;
		}
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
		
	}
	

	return inliersResult;

}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	std::unordered_set<int> inliers;
	
	inliers = Ransac(cloud,maxIterations,distanceThreshold);
	//std::cout<< "segmented the cloud using ransac "<< std::endl;
	if (inliers.size () == 0)
	{
	std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}

    auto endTime = std::chrono::steady_clock::now();   
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>);
	
	
	
	for(int index = 0; index < cloud->points.size(); index++)
	{
		if(inliers.count(index))
			road->points.push_back(cloud->points[index]);
		else
			obstacles->points.push_back(cloud->points[index]);
	}
	

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(road, obstacles);
    return segResult;
}