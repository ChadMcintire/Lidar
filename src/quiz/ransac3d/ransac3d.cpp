/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>

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

// Take two points and make one a vector from points p2 - p1 and the other from p3 - p1
std::pair<std::vector<float>, std::vector<float>> makeVectorsFromPoints(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3){
    std::pair<std::vector<float>, std::vector<float>> vectorPair;
    std::vector<float> v1 = {0,0,0};
    std::vector<float> v2 = {0,0,0};
    v1[0] = p2.x - p1.x; 
    v1[1] = p2.y - p1.y;
    v1[2] = p2.z - p1.z; 

    v2[0] = p3.x - p1.x; 
    v2[1] = p3.y - p1.y; 
    v2[2] = p3.z - p1.z; 
    vectorPair.first = v1;
    vectorPair.second = v2;
    return vectorPair;
}

// Finds the distance between a point and the plane created by the two vectors
std::array<float, 4> findNorm(std::pair<std::vector<float>, std::vector<float>> vectorPair, pcl::PointXYZ p1)
{
      float A = vectorPair.first[1] * vectorPair.second[2] - vectorPair.first[2] * vectorPair.second[1];
      float B = vectorPair.first[2] * vectorPair.second[0] - vectorPair.first[0] * vectorPair.second[2];
      float C = vectorPair.first[0] * vectorPair.second[1] - vectorPair.first[1] * vectorPair.second[0];
      float D = -(A * p1.x + B * p1.y + C * p1.z);

      //std::cout << "Points :" << x1 << "," << x2 << "," << y1 << "," << y2 << "," << std::endl; 

      std::array<float, 4> returnVals = {A, B, C, D};   
      //std::cout << "a, b, c :" << a << " ," << b  << " ," << c << " ," << std::endl;
      return returnVals;
}

// 3D distance formula
float find3dDistance(std::array<float, 4> normVec, pcl::PointXYZ p1){
    float distance = fabs(normVec[0] * p1.x + normVec[1] * p1.y + normVec[2] * p1.z + normVec[3]) / sqrt(normVec[0] * normVec[0] + normVec[1] * normVec[1] + normVec[2] * normVec[2]);
  return distance;
}

// obtain a random index 
int selectRandom(int vecSize){
    // Initialize random number generator
    std::random_device rd;  // Seed
    std::mt19937 gen(rd()); // Mersenne Twister RNG
    std::uniform_int_distribution<> dis(0, vecSize - 1); // Range of indices

    // Select a random index
    int random_index = dis(gen);
    return random_index;
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

/*
Set up Ransac for a for 3d instead of the cartesian plane.
*/ 
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::vector<int> finalInliersResult;
	srand(time(NULL));
	
    std::vector<int> inliersResult;
  
  // The max iter is the number of times we try to form a line of best fit (more like a plane in 3d) then return
  // the points that include the most points because it will be a better fit the more points included
    while(maxIterations--) {
        //We get rid of the elements we capture if any to check two new points for best fit
        inliersResult.clear();
        std::cout << "Size of Storage vectors" << "FinalInlierSize: " << finalInliersResult.size() << ", InlierSize: " << inliersResult.size() << std::endl;
  
        //push every element that has not been checked (all of them) into a vector to be tested
        std::vector<int> remainingElements;
        for (int i = 0; i < cloud->points.size(); i++){
        	remainingElements.push_back(i);
        }
        std::cout << "remaining elements" << remainingElements.size() << std::endl;
        std::cout << "After pushing all remaining elements" << std::endl;

        //grab 2 random points and put them into the index vector, and pop them off the remaining vector
        // the if-else statement is to make sure that we do not grab the same point twice, doing so will give a 
        // segfault with no useful error.
      	std::vector<pcl::PointXYZ> original_points; 
        for (int i = 0; i < 3; i++) {
            if (inliersResult.empty()) {
                int index = selectRandom(cloud->points.size());
                original_points.push_back(cloud->points[index]);
                remainingElements.erase(std::find(remainingElements.begin(),remainingElements.end(),index));
                inliersResult.push_back(index);
          } else {
            
            int index2 = selectRandom(cloud->points.size());
            while (index2 == inliersResult[0]) {
              index2 = selectRandom(cloud->points.size());
            }
            original_points.push_back(cloud->points[index2]);
            remainingElements.erase(std::find(remainingElements.begin(),remainingElements.end(),index2));
            inliersResult.push_back(index2);
          }
        }
        // make the vectors from p1, p2, p3, then take their vector norms to give us the values A, B, C, D
        std::pair<std::vector<float>, std::vector<float>> vecPair = makeVectorsFromPoints(original_points[0], original_points[1], original_points[2]);
      std::array<float, 4> abcd = findNorm(vecPair, original_points[0]);
      
    // go through every point and check that the point is not part of the best fit  
    while(!remainingElements.empty()){
       // for some reason c++ does not let you popback and assign on the same line, says 
      // it will result in a null pointer, which it will not
       int remaining_index = remainingElements.back();
       remainingElements.pop_back();
       pcl::PointXYZ p1 = cloud->points[remaining_index];
  
       // Check the distance from current point to the plane we are trying to fit
       float d = find3dDistance(abcd, p1);
       if (d <= distanceTol){
       //std::cout << "a " << abc[0] << std::endl;
       //std::cout << "b " << abc[1] << std::endl;
       //std::cout << "c " << abc[2] << std::endl;
       //std::cout << "X " << x3 << std::endl;
       //std::cout << "Y " << y3 << std::endl;
       //std::cout << "Distance formula result " << d << std::endl;
       //std::cout << "check index" << remaining_index << std::endl;
         inliersResult.push_back(remaining_index);
         //std::cout << "inliers at the end of each iter" << inliersResult.size() << std::endl;
         
       }
     }  

      // Take the result of the search and update the final set to include the current points if we included
      // more points than the previous max iter passes
      if (finalInliersResult.size() < inliersResult.size()){
          finalInliersResult = inliersResult;
      }
      

    }
    // the program needs a set returned, so we convert the vector to a set and return the set
    std::unordered_set<int> my_set(finalInliersResult.begin(), finalInliersResult.end());
	return my_set;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 3, 0.24);

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