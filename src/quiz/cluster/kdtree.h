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

	KdTree()
	: root(NULL)
	{}

  void recursive_helper(Node** node, unsigned int depth, std::vector<float> point, int id) {
    // If the tree is empty
    if (*node == NULL) {
      *node = new Node(point, id);
    }
    else {
      unsigned int cd = depth % 2; // find the current dimension
      // if the x, y, or z is < node's x,y,or z?
      if (point[cd] < ((*node)->point[cd])) {
        recursive_helper(&((*node)->left), depth+1, point, id);
      }
      else {
        recursive_helper(&((*node)->right), depth+1, point, id);
      }
    }

  }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
    recursive_helper(&root, 0, point, id);
	}

  void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids) {
    if (node!=NULL){
      if( (node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)) && (node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol)))
      {
        float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
        if (distance <= distanceTol)
          ids.push_back(node->id);
      }

      // check across split
      if((target[depth%2]-distanceTol) < node->point[depth%2])
        searchHelper(target, node->left, depth+1, distanceTol, ids);
      if((target[depth%2]+distanceTol) > node->point[depth%2])
        searchHelper(target, node->right, depth+1, distanceTol, ids);
    }
  }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
    // 1. look at box around target + distanceTol to split regions
    // 2. if points within the box, use radius
    //  if points not in box, compare the target(depth) value and go left or right
		std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};
