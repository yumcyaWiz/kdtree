#ifndef _KDTREE_H
#define _KDTREE_H
#include <algorithm>
#include <array>
#include <initializer_list>
#include <iostream>

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

}  // namespace kdtree

#endif