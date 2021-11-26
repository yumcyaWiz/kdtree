#ifndef _KDTREE_LINEAR_H
#define _KDTREE_LINEAR_H
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <vector>

namespace kdtree {

// Point concept
template <typename T>
concept Point = requires(T& x, int i) {
  { T::dim } -> std::convertible_to<int>;  // dimension
  { x[i] } -> std::convertible_to<float>;  // element access
};

// compute squared distance between given points
// NOTE: assume PointT and PointU has the same dimension
template <typename PointT, typename PointU>
requires Point<PointT> && Point<PointU>
inline float distance2(const PointT& p1, const PointU& p2) {
  float dist2 = 0;
  for (int i = 0; i < PointT::dim; ++i) {
    dist2 += (p1[i] - p2[i]) * (p1[i] - p2[i]);
  }
  return dist2;
}

// PointT: user defined point type
// PointT must have following property
// unsigned int PointT::dim                   dimmension
// T PointT::operator[](unsigned int) const   element access
// TODO: use concept in C++20
template <typename PointT>
requires Point<PointT>
class KdTree {
 private:
  struct Node {
    int axis;           // separation axis
    int idx;            // index of median point
    int leftChildIdx;   // index of left child
    int rightChildIdx;  // index of right child

    Node() : axis(-1), idx(-1), leftChildIdx(-1), rightChildIdx(-1) {}
  };

  std::vector<PointT> points;  // array of points
  std::vector<Node> nodes;     // array of tree nodes

  // build kd-tree recursively
  // indices: indices of points
  // n_points: number of points
  // depth : current tree depth
  // NOTE: using indices rather than points directly helps to reduce memory
  // allocation and copy operations
  void buildNode(int* indices, int n_points, int depth) {
    // if points is empty
    if (n_points <= 0) return;

    // separation axis
    const int axis = depth % PointT::dim;

    // sort indices by point coordinate in separation axis
    std::sort(indices, indices + n_points, [&](const int idx1, const int idx2) {
      return points[idx1][axis] < points[idx2][axis];
    });

    // index of middle element of indices
    const int mid = (n_points - 1) / 2;

    // add node to node array, remember index of current node(parent node)
    const int parentIdx = nodes.size();
    Node node;
    node.axis = axis;
    node.idx = indices[mid];
    nodes.push_back(node);

    // add left children on node array
    const int leftChildIdx = nodes.size();
    buildNode(indices, mid, depth + 1);

    // set index of left child on parent node
    // if size of nodes doesn't change, it means there is no left children
    if (leftChildIdx == nodes.size()) {
      nodes[parentIdx].leftChildIdx = -1;
    } else {
      nodes[parentIdx].leftChildIdx = leftChildIdx;
    }

    // add right children on node array
    const int rightChildIdx = nodes.size();
    buildNode(indices + mid + 1, n_points - mid - 1, depth + 1);

    // set index of right child on parent node
    // if size of nodes doesn't change, it means there is no right children
    if (rightChildIdx == nodes.size()) {
      nodes[parentIdx].rightChildIdx = -1;
    } else {
      nodes[parentIdx].rightChildIdx = rightChildIdx;
    }
  }

  bool hasLeftChild(int nodeIdx) const {
    const Node& node = nodes[nodeIdx];
    return node.leftChildIdx != -1;
  }

  bool hasRightChild(int nodeIdx) const {
    const Node& node = nodes[nodeIdx];
    return node.rightChildIdx != -1;
  }

  void toGraphVizNode(int nodeIdx, std::ofstream& stream) const {
    const Node& node = nodes[nodeIdx];

    if (hasLeftChild(nodeIdx)) {
      const Node& leftChild = nodes[node.leftChildIdx];
      // draw edge
      stream << node.idx << "->" << leftChild.idx << " [label=" << node.axis
             << "]" << std::endl;

      // draw child recursively
      toGraphVizNode(node.leftChildIdx, stream);
    } else {
      // create null node
      const std::string nullNode = "null" + std::to_string(node.idx) + "0";
      stream << nullNode << " [label=\"\", shape=\"none\"]" << std::endl;

      // draw edge
      stream << node.idx << "->" << nullNode << " [label=" << node.axis << "]"
             << std::endl;
    }

    if (hasRightChild(nodeIdx)) {
      const Node& rightChild = nodes[node.rightChildIdx];

      // draw edge
      stream << node.idx << "->" << rightChild.idx << " [label=" << node.axis
             << "]" << std::endl;

      // draw child recursively
      toGraphVizNode(node.rightChildIdx, stream);
    } else {
      // create null node
      const std::string nullNode = "null" + std::to_string(node.idx) + "1";
      stream << nullNode << " [label=\"\", shape=\"none\"]" << std::endl;

      // draw edge
      stream << node.idx << "->" << nullNode << " [label=" << node.axis << "]"
             << std::endl;
    }
  }

  // search nearest neighbor node recursively
  template <typename PointU>
  requires Point<PointU>
  void searchNearestNode(int nodeIdx, const PointU& queryPoint,
                         int& idx_nearest, float& minDist2) const {
    if (nodeIdx >= nodes.size()) return;

    const Node& node = nodes[nodeIdx];

    // median point
    const PointT& median = points[node.idx];

    // update minimum squared distance and index of nearest point
    const float dist2 = distance2(queryPoint, median);
    if (dist2 < minDist2) {
      idx_nearest = node.idx;
      minDist2 = dist2;
    }

    // if query point is lower than median, search left child
    // else, search right child
    const bool isLower = queryPoint[node.axis] < median[node.axis];
    if (isLower) {
      searchNearestNode(node.leftChildIdx, queryPoint, idx_nearest, minDist2);
    } else {
      searchNearestNode(node.rightChildIdx, queryPoint, idx_nearest, minDist2);
    }

    // if query point is lower than median, search left child
    // else, search right child
    const float dist_to_siblings = median[node.axis] - queryPoint[node.axis];
    if (minDist2 > dist_to_siblings * dist_to_siblings) {
      if (isLower) {
        searchNearestNode(node.rightChildIdx, queryPoint, idx_nearest,
                          minDist2);
      } else {
        searchNearestNode(node.leftChildIdx, queryPoint, idx_nearest, minDist2);
      }
    }
  }

  // search k-nearest neighbor nodes recursively
  using KNNQueue = std::priority_queue<std::pair<float, int>>;
  template <typename PointU>
  requires Point<PointU>
  void searchKNearestNode(int nodeIdx, const PointU& queryPoint, int k,
                          KNNQueue& queue) const {
    if (nodeIdx >= nodes.size()) return;

    const Node& node = nodes[nodeIdx];

    // median point
    const PointT& median = points[node.idx];

    // push to queue
    const float dist2 = distance2(queryPoint, median);
    queue.emplace(dist2, node.idx);

    // if size of queue is larger than k, pop queue
    if (queue.size() > k) {
      queue.pop();
    }

    // if query point is lower than median, search left child
    // else, search right child
    const bool isLower = queryPoint[node.axis] < median[node.axis];
    if (isLower) {
      searchKNearestNode(node.leftChildIdx, queryPoint, k, queue);
    } else {
      searchKNearestNode(node.rightChildIdx, queryPoint, k, queue);
    }

    // at leaf node, if size of queue is smaller than k, or queue's largest
    // minimum distance overlaps sibblings region, then search siblings
    const float dist_to_siblings = median[node.axis] - queryPoint[node.axis];
    if (queue.top().first > dist_to_siblings * dist_to_siblings) {
      if (isLower) {
        searchKNearestNode(node.rightChildIdx, queryPoint, k, queue);
      } else {
        searchKNearestNode(node.leftChildIdx, queryPoint, k, queue);
      }
    }
  }

  // range search with radius r sphere
  template <typename PointU>
  requires Point<PointU>
  void sphericalRangeSearchNode(int nodeIdx, const PointU& queryPoint, float r,
                                std::vector<int>& list) const {
    if (nodeIdx >= nodes.size()) return;

    const Node& node = nodes[nodeIdx];

    // median point
    const PointT& median = points[node.idx];

    // if distance from query point to median point is smaller than radius, add
    // median point to list
    const float dist2 = distance2(queryPoint, median);
    if (dist2 < r * r) {
      list.push_back(node.idx);
    }

    // if query point is lower than median, search left child
    // else, search right child
    const bool isLower = queryPoint[node.axis] < median[node.axis];
    if (isLower) {
      sphericalRangeSearchNode(node.leftChildIdx, queryPoint, r, list);
    } else {
      sphericalRangeSearchNode(node.rightChildIdx, queryPoint, r, list);
    }

    // at leaf node, if sphere overlaps sibblings region, search sibbligs
    const float dist_to_siblings = median[node.axis] - queryPoint[node.axis];
    if (r * r > dist_to_siblings * dist_to_siblings) {
      if (isLower) {
        sphericalRangeSearchNode(node.rightChildIdx, queryPoint, r, list);
      } else {
        sphericalRangeSearchNode(node.leftChildIdx, queryPoint, r, list);
      }
    }
  }

 public:
  KdTree() {}
  KdTree(std::initializer_list<PointT> init) : points(init) {}
  KdTree(const std::vector<PointT>& points) : points(points) {}

  // build kd-tree
  void buildTree() {
    // setup indices of points
    std::vector<int> indices(points.size());
    std::iota(indices.begin(), indices.end(), 0);

    // build tree recursively
    buildNode(indices.data(), points.size(), 0);
  }

  // output tree as graphviz dot file
  void toGraphviz(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file) {
      std::cerr << "failed to create " << filename << std::endl;
    }

    // write dot
    file << "digraph {" << std::endl;
    toGraphVizNode(0, file);
    file << "}" << std::endl;

    file.close();
  }

  // nearest neighbor search
  // return index of nearest neighbor point
  template <typename PointU>
  requires Point<PointU>
  int searchNearest(const PointU& queryPoint) const {
    int idx_nearest;
    // NOTE: initialize minimum squared distance to infinity
    float minDist2 = std::numeric_limits<float>::max();
    searchNearestNode(0, queryPoint, idx_nearest, minDist2);
    return idx_nearest;
  }

  // k-nearest neighbor search
  // return indices of k-nearest neighbor points
  template <typename PointU>
  requires Point<PointU> std::vector<int> searchKNearest(
      const PointU& queryPoint, int k)
  const {
    KNNQueue queue;
    searchKNearestNode(0, queryPoint, k, queue);

    std::vector<int> ret(queue.size());
    for (int i = 0; i < ret.size(); ++i) {
      ret[i] = queue.top().second;
      queue.pop();
    }
    return ret;
  }

  // range search with radius r sphere centered at query point
  template <typename PointU>
  requires Point<PointU> std::vector<int> sphericalRangeSearch(
      const PointU& queryPoint, float r)
  const {
    std::vector<int> ret;
    sphericalRangeSearchNode(0, queryPoint, r, ret);
    return ret;
  }
};

}  // namespace kdtree

#endif