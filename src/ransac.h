#ifndef RANSAC_H
#define RANSAC_H
#include <pcl/io/pcd_io.h>
#include <vector>

class Ransac {
  public:
    void RansacCalculate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol, pcl::PointIndices::Ptr inliers);
};
#endif