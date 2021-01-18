#ifndef _POINT_H
#define _POINT_H
#include <array>
#include <initializer_list>

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

}  // namespace kdtree

#endif