#ifndef _KDTREE_H
#define _KDTREE_H
#include <algorithm>
#include <iostream>
#include <vector>

#include "point.h"

namespace kdtree {

struct Node {
  int axis;          // separation axis
  int idx_median;    // index of median point
  Node* leftChild;   // left child node
  Node* rightChild;  // right child node
};

// PointT: point type
// PointT must have following property
// unsigned int PointT::dim                   dimmension
// T PointT::operator[](unsigned int) const   element access
// TODO: use concept in C++20
template <typename PointT>
class KdTree {
 private:
  std::vector<PointT> points;  // array of points
  Node* root;                  // root node

  // build kd-tree recursively
  // indices: indices of points
  // idx_start: start index of points
  // idx_end: end index of points
  // depth : current tree depth
  // NOTE: using indices rather than points directly helps to reduce memory
  // allocation and copy operations
  Node* buildNode(int* indices, int idx_start, int idx_end, int depth) {
    // if points is empty
    if (idx_start > idx_end) return nullptr;

    // separation axis
    int axis = depth % PointT::dim;

    // sort indices
    std::sort(indices + idx_start, indices + idx_end,
              [&](const int idx1, const int idx2) {
                return points[idx1][axis] < points[idx2][axis];
              });

    // choose median
    const int idx_median = (idx_end - idx_start) / 2 + idx_start;

    // create node recursively
    Node* node = new Node;
    node->axis = axis;
    node->idx_median = idx_median;
    node->leftChild = buildNode(indices, idx_start, idx_median - 1, depth + 1);
    node->rightChild = buildNode(indices, idx_median + 1, idx_end, depth + 1);

    return node;
  }

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
    root = buildNode(indices.data(), 0, points.size() - 1, 0);
  }
};

}  // namespace kdtree

#endif