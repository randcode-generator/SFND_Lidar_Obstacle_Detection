#include "cluster.h"

template<typename PointT>
EuclideanCluster<PointT>::EuclideanCluster(KdTree<PointT>* tree) {
  this->tree = tree;
}

template<typename PointT>
void EuclideanCluster<PointT>::clusterHelper(int indice, const typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool> &processed, float distanceTol) {
	processed[indice] = true;
	cluster.push_back(indice);

  auto point = {
    cloud->points[indice].x,
    cloud->points[indice].y
  };
	std::vector<int> nearest = this->tree->search(point, distanceTol);

	for(int id : nearest) {
		if(!processed[id])
			this->clusterHelper(id, cloud, cluster, processed, distanceTol);
	}
}

template<typename PointT>
std::vector<std::vector<int>> EuclideanCluster<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->points.size(), false);
	
	int i = 0;
	while(i < cloud->points.size()) {
		if(processed[i]) {
			i++;
			continue;
		}

		std::vector<int> cluster;
		this->clusterHelper(i, cloud, cluster, processed, distanceTol);
		if(cluster.size() >= this->minSize && cluster.size() <= this->maxSize)
      clusters.push_back(cluster);
		i++;
	}
	return clusters;
}

template<typename PointT>
void EuclideanCluster<PointT>::setMinClusterSize(int size) {
  this->minSize = size;
}

template<typename PointT>
void EuclideanCluster<PointT>::setMaxClusterSize(int size) {
  this->maxSize = size;
}