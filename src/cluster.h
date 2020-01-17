#ifndef CLUSTER_H
#define CLUSTER_H
#include <pcl/io/pcd_io.h>
#include "kdtree.h"

template<typename PointT>
class EuclideanCluster {
  KdTree<PointT>* tree;
  int minSize = 10;
  int maxSize = 200;

  void clusterHelper(int indice, const typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool> &processed, float distanceTol);

  public:
    EuclideanCluster(KdTree<PointT>* tree);
    std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol);

    void setMinClusterSize(int size);
    void setMaxClusterSize(int size);
};
#endif /* CLUSTER_H */