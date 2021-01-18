#ifndef _KDTREE_H
#define _KDTREE_H
#include <algorithm>
#include <array>
#include <initializer_list>
#include <iostream>
#include <vector>

namespace kdtree {

template <typename T, unsigned int N>
class Point {
 private:
  std::array<T, N> elements;

 public:
  Point() : Point(0) {}
  Point(T x) {
    for (int i = 0; i < N; ++i) {
      elements[i] = x;
    }
  }
  Point(std::initializer_list<T> init) {
    if (init.size() > N) {
      std::cerr << "invalid dimension" << std::endl;
      std::exit(EXIT_FAILURE);
    }
    std::copy(init.begin(), init.end(), elements.begin());
  }

  T operator[](unsigned int i) const { return elements.at(i); }
};

using Point2u = Point<unsigned int, 2>;
using Point2i = Point<int, 2>;
using Point2f = Point<float, 2>;
using Point2d = Point<double, 2>;

using Point3u = Point<unsigned int, 3>;
using Point3i = Point<int, 3>;
using Point3f = Point<float, 3>;
using Point3d = Point<double, 3>;

template <typename T, unsigned int N>
class KdTree {
 private:
  std::vector<Point<T, N>> points;

  struct Node {
    int axis;  // needed to visualize tree
    Point<T, N> median;
    Node* leftChild;
    Node* rightChild;
  };

  Node* root;

  // build kd-tree recursively
  // idx_start: start index of points
  // idx_end: end index of points
  // NOTE: we can not pass points directly because we need to handle partial
  // elements of points
  // depth : current tree depth
  Node* buildNode(int idx_start, int idx_end, int depth) {
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
    Node* node = new Node;
    node->axis = axis;
    node->median = points[idx_median];
    node->leftChild = buildNode(idx_start, idx_median, depth + 1);
    node->rightChild = buildNode(idx_median + 1, idx_end, depth + 1);

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
  KdTree() {}
  KdTree(std::initializer_list<Point<T, N>> init) : points(init) {}
  KdTree(const std::vector<Point<T, N>>& points) : points(points) {}

  ~KdTree() { destructNode(root); }

  // build kd-tree
  void buildTree() { root = buildNode(0, points.size(), 0); }
};

}  // namespace kdtree

#endif