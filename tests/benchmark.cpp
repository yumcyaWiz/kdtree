#include <chrono>
#include <random>

#include "kdtree/kdtree.h"

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

// [0, 1] uniform random
std::random_device rnd_dev;
std::mt19937 rng(rnd_dev());
std::uniform_real_distribution<float> dist(0, 1);
float rnd() { return dist(rng); }

int main() {
  const int N = 1000000;
  const int k = 100;

  // generate random points
  std::vector<Point> points;
  for (int i = 0; i < N; ++i) {
    points.emplace_back(rnd(), rnd(), rnd());
  }

  Point query_point(0.5f, 0.5f, 0.5f);

  // build kdtree
  kdtree::KdTree<Point> kdtree(points);
  auto start_time = std::chrono::system_clock::now();
  kdtree.buildTree();
  const auto build_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - start_time);
  printf("build: %d ms\n", build_time);

  // search nearest neighbor
  start_time = std::chrono::system_clock::now();
  kdtree.searchNearest(query_point);
  const auto search_nearest_time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now() - start_time);
  printf("searchNearest: %d ns\n", search_nearest_time);

  // search k-nearest neighbor
  start_time = std::chrono::system_clock::now();
  kdtree.searchKNearest(query_point, k);
  const auto search_k_nearest_time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now() - start_time);
  printf("searchKNearest: %d ns\n", search_k_nearest_time);

  return 0;
}