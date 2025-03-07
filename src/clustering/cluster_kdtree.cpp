#include <chrono>
#include <string>
#include <memory>
#include "kdtree_pcl.implementation.h"

template<typename PointT>
class ClusterPts {
private:
  int num_points;
  float clusterTol;
  int minClusterSize;
  int maxClusterSize;
  std::vector<bool> processed;
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // std::vector<std::vector<int>> clusters;

public:
  ClusterPts(int nPts, float clustTol, int minSize, int maxSize) :
      num_points(nPts), clusterTol(clustTol), minClusterSize(minSize), maxClusterSize(maxSize) {
        processed.assign(num_points, false);
      }
  ~ClusterPts() = default;

  void proximity(const typename pcl::PointCloud<PointT>::Ptr& cloud, KdTree& tree, std::vector<int>& cluster, int idx);
  std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclidCluster(const typename pcl::PointCloud<PointT>::Ptr& cloud);
};


/**********************************************************************/
template<typename PointT>
void ClusterPts<PointT>::proximity(
    const typename pcl::PointCloud<PointT>::Ptr& cloud, 
    KdTree& tree, 
    std::vector<int>& cluster, 
    int idx) 
{
    // Start with a queue and add the first point (idx)
    std::queue<int> q;
    q.push(idx);
    processed[idx] = true;

    // While the que isn't empty, take the first point from the queue
    while (!q.empty()) {
        int point_idx = q.front();
        q.pop();
        cluster.push_back(point_idx);

        // Find the neighbors using tree.search(), mark neighbor as processed, push it into the queue. Repeat until the queue is empty. 
        std::vector<int> neighbors = tree.search(cloud->points[point_idx], clusterTol);
        for (int neighbor : neighbors) {
            if (!processed[neighbor]) {
                processed[neighbor] = true;
                q.push(neighbor);
            }
        }
    }
}

/**********************************************************************/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusterPts<PointT>::EuclidCluster(const typename pcl::PointCloud<PointT>::Ptr& cloud) {
  // The tree is only used during clustering so we don't really need to have it available elsewhere
  std::unique_ptr<KdTree> tree(new KdTree());

  for (int i=0; i<num_points; i++)
    tree->insert(cloud->points[i],i);

  // Iterate over all points to form clusters
  for (int i=0; i<num_points; i++) {
    if (processed[i]) {
      continue;
    }

    std::vector<int> cluster_idx;
    typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
    proximity(cloud, *tree, cluster_idx, i);

    int cluster_size = cluster_idx.size();


        if (cluster_size >= minClusterSize && cluster_size <= maxClusterSize) {
            cloudCluster->points.reserve(cluster_size); // Optimize memory allocation

            for (int j : cluster_idx) {
                cloudCluster->points.push_back(cloud->points[j]);
            }

      cloudCluster->width = cloudCluster->points.size();
	  cloudCluster->height = 1;
      cloudCluster->is_dense = true;


      clusters.push_back(cloudCluster);
    }

  }

  return clusters;
}
