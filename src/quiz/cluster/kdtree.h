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

class KdTree
{
	private:
    	
    public: 
    std::vector<std::vector<float>> medianAlternativingVector;
    Node* root;
		KdTree()
		: root(NULL)
		{}
    
    void insertHelper(Node **node, uint depth, std::vector<float> point, int id)
    {
    	
    	if(*node == NULL)
      	{
        	*node = new Node(point, id);
      	}
        else
        {
        // Calculate current depth
        uint cd = depth % 2;
      		if(point[cd] < ((*node)->point[cd]))
      		{
        		insertHelper(&((*node)->left), depth+1, point, id);
      		}
      		else
      		{
        		insertHelper(&((*node)->right), depth+1, point, id);
      		}
        }
    return;
    }
    
    
	void insert(std::vector<float> point, int id)
	{
    	// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
 		
        std::cout << "Id :" << id << std::endl;
        std::cout << "point x,y :" << point[0] << "," << point[1] << std::endl;
		insertHelper(&root,0, point, id);
		
	}
/*
	void searchHelper(std::vector<float> target, Node * node, int depth, float distanceTol, std::vector<int> & ids)
	{
    	if(*node != NULL){
        	if( (node->point[0] >= (target[0] - distanceTol) && node->point[0] - (target[0] + distanceTol)) &&
            (node->point[1] >= (target[1] - distanceTol) && node->point[1] - (target[1] + distanceTol)) {
            	float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                                       (node->point[1] - target[1]) * (node->point[1] - target[1]));
                if (distance <= distanceTol){
                	ids.push_back(node->id);
                }                            
            }
            if((target[depth%2]-distanceTol) < node->point[depth%2]){
            	searchHelper(target, node->left, depth+1, distanceTol, ids);
            }
            if((target[depth%2] + distanceTol) > node->point[depth%2]){
            	searchHelper(target, node->right, depth+1, distanceTol, ids);
            }
        }
    }
*/    
    void searchHelper(std::vector<float> target, Node * node, int depth, float distanceTol, std::vector<int> & ids)
{
    if(node != NULL) { // Corrected *node to node
        if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) &&
            (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol))) {
            float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                                   (node->point[1] - target[1]) * (node->point[1] - target[1]));
            if (distance <= distanceTol) {
                ids.push_back(node->id);
            }
        }
        if ((target[depth % 2] - distanceTol) < node->point[depth % 2]) {
            searchHelper(target, node->left, depth + 1, distanceTol, ids);
        }
        if ((target[depth % 2] + distanceTol) > node->point[depth % 2]) {
            searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
    }
}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	
    
    // This function runs through vectors, one sorted on x, one sorted on y. It then takes one point from x, then
    // takes the median, oscilating between x and y until there are no more points. This presorts the vector to prep a 
    // more optimal binary search when we make the tree.
    std::vector<std::vector<float>> findMedianPoints(std::vector<std::vector<float>> & x, std::vector<std::vector<float>> & y)
    {
    	//check if either vector is empty before performing operations
    	if(x.empty() || y.empty()){
        	std::cerr << "one of the vectors is empty";
        }
        
        // allows us to alternate between x and y
    	bool isX = true;
        // sorted vector to return at the end
        std::vector<std::vector<float>> finishedVec;

		// access the median of x, pop it, pop the same value in the y array
        while(!x.empty()){
        	if(isX && x.size() != 1){
            int access = (x.size() -1)/2;
            std::vector<float> temp = x.at(access);
            x.erase(x.begin() + access);
            
            //grab the index so we can pop the same value on the y array (x, y, index)
          	int index = static_cast<int>(temp.back());
            temp.pop_back();
            finishedVec.push_back(temp);
            
            // Find and erase the corresponding point in y
       		auto it = std::find_if(y.begin(), y.end(), [index](const std::vector<float> &point) {
            	return static_cast<int>(point.back()) == index;
        	});
            //delete the y point
        	if (it != y.end()) {
            	y.erase(it);
        	}
            // Toggle between X and Y
        	isX = !isX;            
            }
            
            // grab a y median point and pop the x value the corresponds to it.
            else if(!isX && x.size() != 1){
            	int access = (y.size() -1)/2;
            	std::vector<float> temp = y.at(access);
                y.erase(y.begin() + access);
            //	// Assuming the last element is an ID
          		int index = static_cast<int>(temp.back());
            	temp.pop_back();
            	finishedVec.push_back(temp);
            
            	// Find and erase the corresponding point in y
       			auto it = std::find_if(x.begin(), x.end(), [index](const std::vector<float> &point) {
            		return static_cast<int>(point.back()) == index;
        		});
                //delete the x value
        		if (it != x.end()) {
            		x.erase(it);
        		}
            //	// Toggle between X and Y
        		isX = !isX;
            }
            // finish getting rid of the last point, then delete the final point in the x and y arrays
            else {
            	std::vector<float> temp = x.at(0);
                int index = static_cast<int>(temp.back());
                finishedVec.push_back(temp);
                x.pop_back();
                y.pop_back();
            }
        }
        // check if the points were evaluated correctly
        for (const auto i :finishedVec){
        	std::cout << "point ";
        	for (const auto j: i){
                std::cout << j << ",";
            }
            std::cout << std::endl;
        }
        return finishedVec;
    }
    
    //sort vector on x and y values so we can get a vector of alternating x and y median values in order
    void sortVectors(std::vector<std::vector<float>> unsorted){
    	std::vector<std::vector<float>> temp;
    	int count = 0;
    	for(auto index : unsorted){
        	std::vector<float> pushVec;
        	for(auto depth : index){
            	pushVec.push_back(depth);
            }
            pushVec.push_back(count);
            count += 1;
        	temp.push_back(pushVec);
        }

       std::sort(temp.begin(),temp.end(), [](std::vector<float> &a, std::vector<float> &b){ return a[0]<b[0]; });
       std::vector<std::vector<float>> xSort = temp;
        
        std::sort(temp.begin(),temp.end(), [](std::vector<float> &a, std::vector<float> &b){ return a[1]<b[1]; });
        std::vector<std::vector<float>> ySort = temp;
        medianAlternativingVector = findMedianPoints(xSort, ySort);
        std::cout << "vector SIZZZZZZZZZZZZZZZZZe" << medianAlternativingVector.size() << std::endl;

    }

};




