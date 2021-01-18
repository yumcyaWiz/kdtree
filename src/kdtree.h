#ifndef _KDTREE_H
#define _KDTREE_H
#include <algorithm>
#include <iostream>
#include <vector>

#include "point.h"

namespace kdtree {

template <typename T>
struct Node {
  int axis;
  T median;
  Node* leftChild;
  Node* rightChild;
};

template <typename T, unsigned int N>
class KdTree {
 private:
  std::vector<Point<T, N>> points;
  Node<T>* root;

  // build kd-tree recursively
  // idx_start: start index of points
  // idx_end: end index of points
  // NOTE: we can not pass points directly because we need to handle partial
  // elements of points
  // depth : current tree depth
  Node<T>* buildNode(int idx_start, int idx_end, int depth) {
    // if points is empty
    if (idx_start <= idx_end) return nullptr;

    // separation axis
    int axis = depth % N;

    // sort points
    std::sort(points.begin() + idx_start, points.begin() + idx_end,
              [&](const Point<T, N>& p1, const Point<T, N>& p2) {
                return p1[axis] < p2[axis];
              });

    // choose median
    const int idx_median = (idx_end - idx_start) / 2;

    // create node recursively
    Node<T>* node = new Node<T>;
    node->axis = axis;
    node->median = points[idx_median][axis];
    node->leftChild = buildNode(idx_start, idx_median, depth + 1);
    node->rightChild = buildNode(idx_median + 1, idx_end, depth + 1);

    return node;
  }

  void destructNode(Node<T>* node) {
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
  KdTree() {}
  KdTree(std::initializer_list<Point<T, N>> init) : points(init) {}
  KdTree(const std::vector<Point<T, N>>& points) : points(points) {}

  ~KdTree() { destructNode(root); }

  // build kd-tree
  void buildTree() { root = buildNode(0, points.size(), 0); }
};

}  // namespace kdtree

#endif