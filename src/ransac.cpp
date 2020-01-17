#include "ransac.h"
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>

void Ransac::RansacCalculate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol, pcl::PointIndices::Ptr inliers)
{
	std::vector<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--) {
		std::vector<int>inliers;
    int indexValue1 = rand()%(cloud->points.size());
    int indexValue2 = rand()%(cloud->points.size());
    int indexValue3 = rand()%(cloud->points.size());

    inliers.push_back(indexValue1);
    inliers.push_back(indexValue2);
    inliers.push_back(indexValue3);

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto iter = inliers.begin();
		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		z1 = cloud->points[*iter].z;
		iter++;
		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;
		z2 = cloud->points[*iter].z;
		iter++;
		x3 = cloud->points[*iter].x;
		y3 = cloud->points[*iter].y;
		z3 = cloud->points[*iter].z;
    
		float a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float d = -(a*x1+b*y1+c*z1);

		for(int index = 0; index < cloud->points.size(); index++) {
      if(index == indexValue1 || index == indexValue2 || index == indexValue3) {
        continue;
      }
      			
			pcl::PointXYZI point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float dist = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);

			if(dist < distanceTol)
				inliers.push_back(index);
		}

		if(inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}
	
	Eigen::Vector4f plane_parameters;

	EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
	Eigen::Vector4f xyz_centroid;
	
	computeMeanAndCovarianceMatrix (*cloud, inliersResult, covariance_matrix, xyz_centroid);

	EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
	EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
	pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);
	
	float a = eigen_vector[0];
	float b = eigen_vector[1];
	float c = eigen_vector[2];
	float d = -(a*xyz_centroid[0] + b*xyz_centroid[1] + c*xyz_centroid[2]);
	
	for(int index = 0; index < cloud->points.size(); index++) {
		pcl::PointXYZI point = cloud->points[index];
		float x4 = point.x;
		float y4 = point.y;
		float z4 = point.z;

		float dist = fabs((a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c));

		if(dist < distanceTol)
      inliers->indices.push_back(index);
	}
}