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

std::array<float,3> findLine(pcl::PointXYZ pointOne, pcl::PointXYZ  pointTwo)
{
	  float x1 = pointOne.x;
      float y1 = pointOne.y;
  
      float x2 = pointTwo.x;
      float y2 = pointTwo.y;
  	  std::cout << "Points :" << x1 << "," << x2 << "," << y1 << "," << y2 << "," << std::endl;
  
      float a = y1 - y2;
      float b = x2 - x1;
      float c = x1 *y2 - x2 * y1;
      std::array<float, 3> returnVals = {a, b, c};   
      std::cout << "a, b, c :" << a << " ," << b  << " ," << c << " ," << std::endl;
      return returnVals;
}

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::cout << "Section 1" << std::endl;
	std::vector<int> finalInliersResult;
	srand(time(NULL));
	
    std::vector<int> inliersResult;
	while(maxIterations--) {
        inliersResult.clear();
        std::cout << "FinalInlierSize: " << finalInliersResult.size() << ", InlierSize: " << inliersResult.size() << std::endl;
  
        //push every element that has not been checked (all of them) into a vector to be tested
        std::vector<int> remainingElements;
        for (int i = 0; i < cloud->points.size(); i++){
        	remainingElements.push_back(i);
        }
        std::cout << "After pushing all remaining elements" << std::endl;
        //std::cout << "vec size" << remainingElements.size() << std::endl;
        //grab 2 random points and put them into the index vector, and pop them off the remaining vector
    	std::vector<pcl::PointXYZ> original_points; 
        for (int i = 0; i < 2; i++) {
          if (inliersResult.empty()) {
            int index = selectRandom(cloud->points.size());
          
            original_points.push_back(cloud->points[index]);
            remainingElements.erase(std::find(remainingElements.begin(),remainingElements.end(),index));
            inliersResult.push_back(index);
        //std::cout << "original index " << index << endl; 
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
        std::array<float, 3> abc = findLine(original_points[0], original_points[1]);
    
     std::cout << "before while loop" << std::endl;
     while(!remainingElements.empty()){
       int remaining_index = remainingElements.back();
       remainingElements.pop_back();
       float x3 = cloud->points[remaining_index].x;
       float y3 = cloud->points[remaining_index].y;

       
       float d = (fabs(abc[0] * x3 + abc[1] * y3 + abc[2])) / (sqrt(abc[0]*abc[0] + abc[1]*abc[1]));
       
       if (d <= distanceTol){
       //std::cout << "a " << abc[0] << std::endl;
       //std::cout << "b " << abc[1] << std::endl;
       //std::cout << "c " << abc[2] << std::endl;
       //std::cout << "X " << x3 << std::endl;
       //std::cout << "Y " << y3 << std::endl;
       //std::cout << "Distance formula result " << d << std::endl;
       //std::cout << "check index" << remaining_index << std::endl;
         inliersResult.push_back(remaining_index);
       }
     }  
      std::cout << "Before check: " << finalInliersResult.size() << ", " << inliersResult.size() << std::flush;
      if (finalInliersResult.size() < inliersResult.size()){
          finalInliersResult = inliersResult;
      }
      std::cout << "after check" << finalInliersResult.size() << "," << inliersResult.size() << std::endl;
      std::cout << "The end0" << std::endl;
    }
    std::cout << "The end1" << std::endl;
    std::unordered_set<int> my_set(finalInliersResult.begin(), finalInliersResult.end());
    for (int i: my_set)
    	std::cout << i << ' ';

    std::cout << "The end2" << std::endl;
	return my_set;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 3, 1.0);

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
