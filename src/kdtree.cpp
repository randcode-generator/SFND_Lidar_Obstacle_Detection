#include "kdtree.h"

template<typename PointT>
void KdTree<PointT>::insertHelper(Node** node, uint depth, std::vector<float> point, int id) {
  if(*node == NULL) {
    *node = new Node(point, id);
  } else {
    uint cd = depth % 2;

    if(point[cd] < ((*node)->point[cd]))
      insertHelper(&((*node)->left), depth+1, point, id);
    else
      insertHelper(&((*node)->right), depth+1, point, id);
  }
}

template<typename PointT>
void KdTree<PointT>::insert(std::vector<float> point, int id)
{
  insertHelper(&this->root, 0, point, id);
}

template<typename PointT>
void KdTree<PointT>::searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
  if(node!=NULL) {
    if((node->point[0]>=(target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol) && (node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol)))) {
      float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
      if(distance <= distanceTol)
        ids.push_back(node->id);
    }

    if((target[depth%2]-distanceTol)<node->point[depth%2])
      searchHelper(target, node->left, depth+1, distanceTol, ids);
    if((target[depth%2]+distanceTol)>node->point[depth%2])
      searchHelper(target, node->right, depth+1, distanceTol, ids);
  }
}

// return a list of point ids in the tree that are within distance of target
template<typename PointT>
std::vector<int> KdTree<PointT>::search(std::vector<float> target, float distanceTol)
{
  std::vector<int> ids;
  searchHelper(target, root, 0, distanceTol, ids);

  return ids;
}

template<typename PointT>
void KdTree<PointT>::setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud) {
  for (int i = 0; i < cloud->points.size(); i++) {
    this->insert({
      cloud->points[i].x,
      cloud->points[i].y
    }, i);
  }
}
