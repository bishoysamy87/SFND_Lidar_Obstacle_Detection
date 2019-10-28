// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<double> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<double> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};
template<typename PointT>
struct KdTree
{
	Node* root;
	int depth;
	int point_dim = 2;
	int left_current_depth = 0;
	int right_current_depth = 0;

	KdTree()
	: root(NULL)
	{}
	
    void getNode(Node ** node_ptr, std::vector<double> point, int id)
	{
		if((*node_ptr) == NULL)
		{
			(*node_ptr) = new Node(point,id);

		}
		else
		{
			if((*node_ptr)->point[(depth % point_dim)]< point[(depth % point_dim)])
			{
				depth++;
				getNode(&((*node_ptr)->right),point,id);
			}
			else
			{
				depth++;
				getNode(&((*node_ptr)->left),point,id);
				
			}
		}
			
	}

	void insert(PointT po, int id)
	{
		std::vector<double> point = {po.x,po.y,po.z};
		depth = 0;
		if(root == NULL)
		{
			root= new Node(point,id);
		}
		else
		{	
			getNode(&root,point,id);
		}

	}
	
	double getDistance(std::vector<double> target, std::vector<double> point)
	{
		double diffx = target[0] - point[0];
		double diffy = target[1] - point[1];
		double diffz = target[2] - point[2];
		return(sqrt(diffx*diffx + diffy*diffy + diffz*diffz));
	}
	
	Node * getNextNode(Node * node,std::vector<float> point)
	{
		Node * note_ptr;

		if(node->point[(depth % point_dim)]< point[(depth % point_dim)])
		{
			depth++;
			note_ptr = node->right;
		}
		else
		{
			depth++;
			note_ptr = node->left;
		}
		//note_ptr = node->right;
		return note_ptr;
	}
	
	void searchTree(Node * node,std::vector<double> point,float distanceTol,std::vector<int>& ids,int dep)
	{
		if(node != NULL)
		{
			double dist = 100;
			
			dist = getDistance(point,node->point) ;
			if(dist<= distanceTol)
			{
				ids.push_back(node->id);			   
			}

			if(node->point[(dep % point_dim)]> (point[(dep % point_dim)]- distanceTol))
			{
				searchTree(node->left,point,distanceTol,ids,dep+1);
			}
			if(node->point[(dep % point_dim)] < (point[(dep % point_dim)] + distanceTol))
			{
				searchTree(node->right,point,distanceTol,ids,dep+1);
			}
			
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT po, float distanceTol)
	{
		std::vector<double> target = {po.x,po.y,po.z};
		depth = 0;
		Node * next_node; 
		Node * in_box_node;
		bool isSecItr = false;
		int current_depth;
		std::vector<int> ids;
		if(root == NULL)
		{
			ids.push_back(0);
		}
		else
		{
			searchTree(root,target,distanceTol,ids,0);
		}
		return ids;
	}
	

};


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
	
	BoxQ BoundingBox2(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
	std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
	std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT> * tree, float distanceTol,int minSize, int maxSize);
	void Proximity(typename pcl::PointCloud<PointT>::Ptr cloud,
			   std::vector<int>& is_processed,
			   KdTree<PointT> * tree,
			   float distanceTol,
			   int id,
			   std::vector<int>& cluster,int minSize, int maxSize);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */