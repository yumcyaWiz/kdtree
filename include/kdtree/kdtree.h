#ifndef _KDTREE_H
#define _KDTREE_H
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

namespace kdtree {

// PointT: point type
// PointT must have following property
// unsigned int PointT::dim                   dimmension
// T PointT::operator[](unsigned int) const   element access
// TODO: use concept in C++20
template <typename PointT>
class KdTree {
 private:
  struct Node {
    int axis;          // separation axis
    int idx;           // index of median point
    Node* leftChild;   // left child node
    Node* rightChild;  // right child node

    Node() : axis(-1), idx(-1), leftChild(nullptr), rightChild(nullptr) {}
  };

  std::vector<PointT> points;  // array of points
  Node* root;                  // root node

  // build kd-tree recursively
  // indices: indices of points
  // n_points: number of points
  // depth : current tree depth
  // NOTE: using indices rather than points directly helps to reduce memory
  // allocation and copy operations
  Node* buildNode(int* indices, int n_points, int depth) {
    // if points is empty
    if (n_points <= 0) return nullptr;

    // separation axis
    const int axis = depth % PointT::dim;

    // sort indices by point coordinate in separation axis
    std::sort(indices, indices + n_points, [&](const int idx1, const int idx2) {
      return points[idx1][axis] < points[idx2][axis];
    });

    // index of middle element of indices
    const int mid = (n_points - 1) / 2;

    // create node recursively
    Node* node = new Node();
    node->axis = axis;
    node->idx = indices[mid];
    node->leftChild = buildNode(indices, mid, depth + 1);
    node->rightChild =
        buildNode(indices + mid + 1, n_points - mid - 1, depth + 1);

    return node;
  }

  // delete node recursively
  void destructNode(Node* node) {
    if (!node) return;

    // delete left child
    if (node->leftChild) {
      destructNode(node->leftChild);
    }
    // delete right child
    if (node->rightChild) {
      destructNode(node->rightChild);
    }

    // delete current node
    delete node;
  }

  void printTreeNode(const Node* node) const {
    std::cout << node->idx << std::endl;
    if (node->leftChild) {
      printTreeNode(node->leftChild);
    }
    if (node->rightChild) {
      printTreeNode(node->rightChild);
    }
  }

  // compute squared distance between given points
  static float distance2(const PointT& p1, const PointT& p2) {
    float dist2 = 0;
    for (int i = 0; i < PointT::dim; ++i) {
      dist2 += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    }
    return dist2;
  }

  // search nearest neighbor node recursively
  void searchNearestNode(const Node* node, const PointT& queryPoint,
                         int& idx_nearest, float& minDist2) const {
    if (!node) return;

    // median point
    const PointT& median = points[node->idx];

    // update minimum squared distance and index of nearest point
    const float dist2 = distance2(queryPoint, median);
    if (dist2 < minDist2) {
      idx_nearest = node->idx;
      minDist2 = dist2;
    }

    // if query point is lower than median, search left child
    // else, search right child
    const bool isLower = queryPoint[node->axis] < median[node->axis];
    if (isLower) {
      searchNearestNode(node->leftChild, queryPoint, idx_nearest, minDist2);
    } else {
      searchNearestNode(node->rightChild, queryPoint, idx_nearest, minDist2);
    }

    // at leaf node, test minDist overlaps siblings region
    // if so, search siblings
    const float dist_to_siblings = median[node->axis] - queryPoint[node->axis];
    if (minDist2 > dist_to_siblings * dist_to_siblings) {
      if (isLower) {
        searchNearestNode(node->rightChild, queryPoint, idx_nearest, minDist2);
      } else {
        searchNearestNode(node->leftChild, queryPoint, idx_nearest, minDist2);
      }
    }
  }

  // search k-nearest neighbor nodes recursively
  using KNNQueue = std::priority_queue<std::pair<float, int>>;
  void searchKNearestNode(const Node* node, const PointT& queryPoint, int k,
                          KNNQueue& queue) const {
    if (!node) return;

    // median point
    const PointT& median = points[node->idx];

    // push to queue
    const float dist2 = distance2(queryPoint, median);
    queue.emplace(dist2, node->idx);

    // if size of queue is larger than k, pop queue
    if (queue.size() > k) {
      queue.pop();
    }

    // if query point is lower than median, search left child
    // else, search right child
    const bool isLower = queryPoint[node->axis] < median[node->axis];
    if (isLower) {
      searchKNearestNode(node->leftChild, queryPoint, k, queue);
    } else {
      searchKNearestNode(node->rightChild, queryPoint, k, queue);
    }

    // at leaf node, if size of queue is smaller than k, or queue's largest
    // minimum distance overlaps sibblings region, then search siblings
    const float dist_to_siblings = median[node->axis] - queryPoint[node->axis];
    if (queue.top().first > dist_to_siblings * dist_to_siblings) {
      if (isLower) {
        searchKNearestNode(node->rightChild, queryPoint, k, queue);
      } else {
        searchKNearestNode(node->leftChild, queryPoint, k, queue);
      }
    }
  }

 public:
  KdTree() : root(nullptr) {}
  KdTree(std::initializer_list<PointT> init) : root(nullptr), points(init) {}
  KdTree(const std::vector<PointT>& points) : root(nullptr), points(points) {}

  ~KdTree() { destructNode(root); }

  // build kd-tree
  void buildTree() {
    // setup indices of points
    std::vector<int> indices(points.size());
    std::iota(indices.begin(), indices.end(), 0);

    // build tree recursively
    root = buildNode(indices.data(), points.size(), 0);
  }

  // print kd-tree on terminal
  void printTree() const { printTreeNode(root); }

  // nearest neighbor search
  // return index of nearest neighbor point
  int searchNearest(const PointT& queryPoint) const {
    int idx_nearest;
    // NOTE: initialize minimum squared distance to infinity
    float minDist2 = std::numeric_limits<float>::max();
    searchNearestNode(root, queryPoint, idx_nearest, minDist2);
    return idx_nearest;
  }

  // k-nearest neighbor search
  // return indices of k-nearest neighbor points
  std::vector<int> searchKNearest(const PointT& queryPoint, int k) const {
    KNNQueue queue;
    searchKNearestNode(root, queryPoint, k, queue);

    std::vector<int> ret(queue.size());
    for (int i = 0; i < ret.size(); ++i) {
      ret[i] = queue.top().second;
      queue.pop();
    }
    return ret;
  }
};

}  // namespace kdtree

#endif