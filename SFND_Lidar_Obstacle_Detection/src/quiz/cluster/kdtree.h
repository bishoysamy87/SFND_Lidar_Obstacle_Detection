/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"



// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;
	int depth;
	int point_dim = 2;
	int current_depth = 0;

	KdTree()
	: root(NULL)
	{}
	
    void getNode(Node ** node_ptr, std::vector<float> point, int id)
	{
		if((*node_ptr) == NULL)
		{
			(*node_ptr) = new Node(point,id);
			//std::cout<< "The point = "<<point[0]<<","<< point[1]<<std::endl;
			//std::cout<< "The depth = "<<depth<<std::endl;
			//std::cout<< "The depth = "<<id<<std::endl;
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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
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
	
	float getDistance(std::vector<float> target, std::vector<float> point)
	{
		float diffx = target[0] - point[0];
		float diffy = target[1] - point[1];
		float diffz = target[2] - point[2];
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
		return note_ptr;
	}
	
	void searchTree(Node * node,std::vector<float> point,float distanceTol,std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if(getDistance(point,node->point) <= distanceTol)
			{
				ids.push_back(node->id);			   
			}
			if(node->point[(depth % point_dim)]>= (point[(depth % point_dim)]- distanceTol))
			{
				depth++;
				searchTree(node->left,point,distanceTol,ids);
				//ids.insert(ids.end(),std::make_move_iterator(temp_ids.begin()),std::make_move_iterator(temp_ids.end()));
			}
			if(node->point[(depth % point_dim)] <= (point[(depth % point_dim)] + distanceTol))
			{
				depth++;
				searchTree(node->right,point,distanceTol,ids);
				//ids.insert(ids.end(),std::make_move_iterator(temp_ids.begin()),std::make_move_iterator(temp_ids.end()));
			}
			
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
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
			//next_node = root;
			//while(next_node != NULL || isSecItr == true)
			//{
			//	if(isSecItr == true && next_node == NULL)
			//	{
			//		next_node = in_box_node;
			//		std::cout<<" next_node_id " << next_node->id<<std::endl;
			//		isSecItr = false;
			//		depth = current_depth + 1;
			//		next_node = getNextNode(next_node,target);
			//		//std::cout<<" next_node_id " << next_node->id<<std::endl;
			//	}
			//	if(getDistance(target,next_node->point) <= distanceTol)
			//	{
			//		isSecItr = true;
			//		current_depth = depth;
			//		in_box_node = next_node;
			//		ids.push_back(next_node->id);
			//		std::cout<<" next_node_id " << next_node->id<<std::endl;
			//		next_node = getNextNode(next_node,target);
			//	}
			//	else
			//	{
			//		std::cout<<" next_node_id " << next_node->id<<std::endl;
			//		next_node = getNextNode(next_node,target);
					
			//	}
			//}
			searchTree(root,target,distanceTol,ids);
		}
		return ids;
	}
	

};




