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
	std::unordered_set<int> inlierResults;
	srand(time(NULL));
	

	while(maxIterations--) {

            //push every element that has not been checked (all of them) into a vector to be tested
            std::unordered_set<int> inliers;

            while (inliers.size() < 2){
            	inliers.insert(rand()%(cloud->points.size()));
            }
      
            float x1, y1, x2, y2;
      
            auto itr = inliers.begin();
            x1 = cloud->points[*itr].x;
            y1 = cloud->points[*itr].y;
            itr++;
            x2 = cloud->points[*itr].x;
            y2 = cloud->points[*itr].y;
      
            float a = (y1-y2);
            float b = (x2-x1);
            float c = (x1*y2-x2*y1);
            
            for(int index = 0; index < cloud->points.size(); index++)
            {
                if(inliers.count(index)>0){
                    continue;
                }
        
                pcl::PointXYZ point = cloud->points[index];
                
                float d = fabs(a*point.x+b*point.y+c)/sqrt(a*a+b*b);
              
                if(d <= distanceTol) {
            	    inliers.insert(index); 
                }
            }
      
      
      		if(inliers.size() > inlierResults.size())
      		{
      			inlierResults = inliers; 
      		}  
    }
      return inlierResults;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inlierResults;
	srand(time(NULL));

	while(maxIterations--) {

            //push every element that has not been checked (all of them) into a vector to be tested
            std::unordered_set<int> inliers;

            while (inliers.size() < 3){
            	inliers.insert(rand()%(cloud->points.size()));
            }
      
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
            float d = (a*x1 +b*y1 + c*z1);
            float sqrt_abc = sqrt(a*a + b*b + c*c);
      
            for(int index = 0; index < cloud->points.size(); index++)
            {
                if(inliers.count(index)>0){
                    continue;
                }
        
                pcl::PointXYZ point = cloud->points[index];
                
                float d = fabs(a * point.x + b * point.y + c * point.z + d) / sqrt_abc;
              
                if(d <= distanceTol) {
            	    inliers.insert(index); 
                }
            }
      
      
      		if(inliers.size() > inlierResults.size())
      		{
      			inlierResults = inliers; 
      		}  
    }
      return inlierResults;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 20, 0.5);
    std::unordered_set<int> inliers = RansacPlane(cloud, 20, 0.7);
	
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
