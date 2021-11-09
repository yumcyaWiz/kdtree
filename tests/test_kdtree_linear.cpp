#include <algorithm>
#include <queue>
#include <random>
#include <vector>

#include "kdtree/kdtree_linear.h"

// user defined point class
struct Point {
  float v[3];
  static constexpr int dim = 3;

  Point(float x, float y, float z) {
    v[0] = x;
    v[1] = y;
    v[2] = z;
  }

  float operator[](int i) const { return v[i]; }
};

// compute squared distance between points
float distance2(const Point& p1, const Point& p2) {
  float ret = 0;
  for (int i = 0; i < 3; ++i) {
    ret += (p1[i] - p2[i]) * (p1[i] - p2[i]);
  }
  return ret;
}

bool test_search_nearest(const std::vector<Point>& points,
                         const Point& query_point) {
  // prepare reference nearest point
  std::vector<float> dists(points.size());
  for (int i = 0; i < points.size(); ++i) {
    dists[i] = distance2(points[i], query_point);
  }
  const int nearest_idx_ref =
      std::min_element(dists.begin(), dists.end()) - dists.begin();

  // build kdtree
  kdtree::KdTree<Point> kdtree(points);
  kdtree.buildTree();

  // find nearest point
  const int nearest_idx = kdtree.searchNearest(query_point);

  printf("===reference===\n");
  printf("(%f, %f, %f)\n", points[nearest_idx_ref][0],
         points[nearest_idx_ref][1], points[nearest_idx_ref][2]);
  printf("===kdtree===\n");
  printf("(%f, %f, %f)\n", points[nearest_idx][0], points[nearest_idx][1],
         points[nearest_idx][2]);

  return nearest_idx == nearest_idx_ref;
}

bool test_search_k_nearest(const std::vector<Point>& points,
                           const Point& query_point, int k) {
  // prepare reference k-narest points
  // find k-nearest point with priority queue
  std::vector<int> k_nearest_idx_ref;
  std::priority_queue<std::pair<float, int>> queue;
  for (int i = 0; i < points.size(); ++i) {
    queue.emplace(distance2(points[i], query_point), i);
    if (queue.size() > k) {
      queue.pop();
    }
  }
  for (int i = 0; i < k; ++i) {
    k_nearest_idx_ref.push_back(queue.top().second);
    queue.pop();
  }

  // build kdtree
  kdtree::KdTree<Point> kdtree(points);
  kdtree.buildTree();

  // find k-nearest point
  std::vector<int> k_nearest_idx = kdtree.searchKNearest(query_point, k);

  bool success = true;
  for (int i = 0; i < k; ++i) {
    if (k_nearest_idx[i] != k_nearest_idx_ref[i]) {
      success = false;
      break;
    }
  }

  printf("===reference===\n");
  for (int i = 0; i < k; ++i) {
    const int idx = k_nearest_idx_ref[i];
    printf("(%f, %f, %f)\n", points[idx][0], points[idx][1], points[idx][2]);
  }
  printf("===kd-tree===\n");
  for (int i = 0; i < k; ++i) {
    const int idx = k_nearest_idx[i];
    printf("(%f, %f, %f)\n", points[idx][0], points[idx][1], points[idx][2]);
  }

  return success;
}

// [0, 1] uniform random
std::random_device rnd_dev;
std::mt19937 rng(rnd_dev());
std::uniform_real_distribution<float> dist(0, 1);
float rnd() { return dist(rng); }

int main() {
  const int N = 100;
  const int k = 3;

  // generate random points
  std::vector<Point> points;
  for (int i = 0; i < N; ++i) {
    points.emplace_back(rnd(), rnd(), rnd());
  }

  Point query_point(0.5f, 0.5f, 0.5f);

  printf("===points===\n");
  for (int i = 0; i < N; ++i) {
    printf("points[%d] = (%f, %f, %f), distance2=%f\n", i, points[i][0],
           points[i][1], points[i][2], distance2(points[i], query_point));
  }

  // test search nearest point
  if (test_search_nearest(points, query_point)) {
    printf("test_search_nearest_point: OK\n");
  } else {
    printf("test_search_nearest_point: Failed\n");
  }

  printf("\n");

  // test search k nearest point
  if (test_search_k_nearest(points, query_point, k)) {
    printf("test_search_k_nearest_point: OK\n");
  } else {
    printf("test_search_k_nearest_point: Failed\n");
  }

  return 0;
}