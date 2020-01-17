#ifndef KDTREE_H
#define KDTREE_H
#include <pcl/io/pcd_io.h>
#include <vector>

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

template<typename PointT>
class KdTree
{
  void insertHelper(Node** node, uint depth, std::vector<float> point, int id);
  void insert(std::vector<float> point, int id);
  void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids);

	public:
    Node* root;

    KdTree()
    : root(NULL)
    {}
    std::vector<int> search(std::vector<float> target, float distanceTol);
    void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);
};
#endif