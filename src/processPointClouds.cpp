// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
              ProcessPointClouds<PointT>::RansacPlaneSegment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  auto startTime = std::chrono::steady_clock::now();
	srand(time(NULL));

  int num_points = cloud->points.size();
  auto all_points = cloud->points;

  Ransac<PointT> segRansac(maxIterations, distanceTol, num_points);

  // get inliers from local-RANSAC implementation rather than PCL implementation
  std::unordered_set<int> inliersResult = segRansac.Ransac3d(cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "local-RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  typename pcl::PointCloud<PointT>::Ptr out_plane(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr in_plane(new pcl::PointCloud<PointT>());

  for (int i=0; i<num_points; i++) {
    PointT pt = all_points[i];
    if (inliersResult.count(i)) {
      out_plane->points.push_back(pt);
    }
    else {
      in_plane->points.push_back(pt);
    }
  }

  return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(in_plane, out_plane);
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Apply voxel filter
    //downsampling the dataset using a leaf size of .2m
    pcl::VoxelGrid<PointT> vg;
  	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (cloud);
  
  	// setLeafSize( len x, len y, len z), setting the voxel size
    vg.setLeafSize (filterRes, filterRes, filterRes);
    //apply voxel grid filtering and return the  stores the result in cloudFilter
    vg.filter (*cloudFiltered);
    
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
  
    // Set to true if you want to be able to extract the indices of points being removed
    pcl::CropBox<PointT> region(true);
    
    // Lower corner of the crop box
    region.setMin(minPoint);
    // Upper corner of the crop box
  	region.setMax(maxPoint);
    // set the input cloud
    region.setInputCloud(cloudFiltered);
    // the filter points are stored in the cloudRegion
    region.filter(*cloudRegion);
    
    // Store the points that fall in the cropping box
  	std::vector<int> indices;

  	// crop the roof out, it says the true allows for rotating and translating the box
    pcl::CropBox<PointT> roof(true);
    // lower corner of crop box
	roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    // upper corner of crop box
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    // input point cloud from which we draw points
    roof.setInputCloud(cloudRegion);
    // instead of a point cloud, we return the indices of points inside the crop box
    roof.filter(indices);

    // make a pointer to remove the points we want to remove
  	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    //fills inliers with the points to remove
  	for (int point : indices){
    	inliers->indices.push_back(point); 
    }
  
    // allows us to extract point given indices
    pcl::ExtractIndices<PointT> extract;
    // input to be looked at
    extract.setInputCloud(cloudRegion);
    //give the indices to be removed
    extract.setIndices(inliers);
    // keep points that are not in the inliers
  	extract.setNegative(true);
    // applies the cloud filter, removing the region with the unwanted points.
    extract.filter(*cloudRegion);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
     
    // fill the vector with points from the plane
    for (int index : inliers -> indices)
      planeCloud->points.push_back(cloud->points[index]);
      
    // allows us to extract point given indices
    pcl::ExtractIndices<PointT> extract;
    // input to be looked at
    extract.setInputCloud (cloud);
    //give the indices to be removed
    extract.setIndices (inliers);
    // keep points that are not in the inliers
    extract.setNegative (true);
    // applies the cloud filter, removing the region with the unwanted points.
    extract.filter (*obstCloud);
  
    //return a pair that has the point cloud of the obstacles and the plane
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    //Creates a segmentation object that will use RANSAAC to fine the plane
    pcl::SACSegmentation<PointT> seg;
    // store the indices of points belonging to the detected plane
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // stores the plane equation coefficient ( Ax + By + Cz + D =0)
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // Optional
    //minimize errors in the model to optimize the plane model
	seg.setOptimizeCoefficients (true);
	// Specifies that we are looking for a plane model.
	seg.setModelType (pcl::SACMODEL_PLANE);
    // select the best fitting plane while ignoring outliers
	seg.setMethodType (pcl::SAC_RANSAC);
    // set the number of iterations to find the best plane
	seg.setMaxIterations (maxIterations);
    // maximum distance that a point can still be considered a part of the plane
	seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    // update the inliers and the values of the coefficients
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // Extract the inliers

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    //store the clusters as separate point clouds
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    // space partitioning data structure  optimized for nearest-neighbor searches
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    // 
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    
  	// set the max distance between two points to be considered in the same cluster
  	ec.setClusterTolerance (clusterTolerance); // 2cm
    // sets the minimum number of points required to form a cluster
    ec.setMinClusterSize (minSize);
    // sets the maximum allowed cluster size
    ec.setMaxClusterSize (maxSize);
    // uses the kd-tree for efficient searching
    ec.setSearchMethod (tree);
    // cloud input
    ec.setInputCloud (cloud);
    //cluster indices will contain the returned clustered regions
    ec.extract (cluster_indices);
  
    
    for (const pcl::PointIndices getIndices : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        //pre-alloacte memory, reducing memory reallocation overhead. 
        cloud_cluster->points.reserve(getIndices.indices.size());
          
        // extract the points from the original cluster to this one
        for (int index : getIndices.indices) {
            cloud_cluster->points.push_back(cloud->points[index]);
        } //*
      
        // number of points in the cluster (stored as a 1d array)
        cloud_cluster->width = cloud_cluster->points.size();
        // height = 1 means that the cloud is treated as an unordered 1D point cloud
        cloud_cluster->height = 1;
        // Assumes no NAN points
        cloud_cluster->is_dense = true;
      
        // add the extracted points to the clusters vector
        clusters.push_back(cloud_cluster);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " 
              << clusters.size() << " clusters" << std::endl;

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
