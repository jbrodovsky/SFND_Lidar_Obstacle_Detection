/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
//#incldue <chrono>

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
	int dimension;
	KdTree()
	: root(NULL), dimension(0)
	{}

	void insert(std::vector<float> point, int id) 
    {
      if(root == NULL){ dimension = point.size(); }
      inserter(&root, 0, point, id); 
    }
  	void inserter(Node** node, uint depth, std::vector<float> point, int id)
    {
      if (*node==NULL){ *node = new Node(point, id); }
      else
      {  
        int dim = depth % dimension;
      	if (point[dim] < ((*node)->point[dim]))
        	inserter(&((*node)->left), depth+1, point, id);
      	else
        	inserter(&((*node)->right), depth+1, point, id);
      }               
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
      
		std::vector<int> ids;
    	searcher(target, root, 0, distanceTol, ids);
		return ids;
	}
  
  	void searcher(std::vector<float>& target, Node* node, int depth, float& distanceTol, std::vector<int>& ids)
    {
      if(node!=NULL)
      {
        
        if( (node->point[0]>=(target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol)) && 
            (node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol)) &&
            (node->point[2]>=(target[2]-distanceTol) && node->point[2]<=(target[2]+distanceTol))
          )
        {
          float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + 
                                (node->point[1] - target[1])*(node->point[1] - target[1]) +
                                (node->point[2] - target[2])*(node->point[2] - target[2])
                               );
          if (distance <= distanceTol)
            ids.push_back(node->id);
        }
        /*
        float distance = 0;
        for(int i=0; i<target.size(); i++)
        {
          distance += (node->point[i] - target[i])*(node->point[i] - target[i]);
        }
        if(sqrt(distance) <= distanceTol){
          ids.push_back(node->id);
        }
        */
        if((target[depth%dimension] - distanceTol)<node->point[depth%dimension])
          searcher(target, node->left, depth+1, distanceTol, ids);
        if((target[depth%dimension] + distanceTol)>node->point[depth%dimension])
          searcher(target, node->right, depth+1, distanceTol, ids);
      }
    }
	

};




