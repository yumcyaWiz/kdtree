#include <iostream>
#include <random>

// SFML
#include <SFML/Graphics.hpp>

// uniform random number in [0, 1]
inline float rnd() {
  static std::mt19937 mt(0);
  static std::uniform_real_distribution<float> dist(0, 1);
  return dist(mt);
}

// override sf::Vector2f to satisfy PointT requirements
class Point2f : public sf::Vector2f {
 public:
  static constexpr unsigned int dim = 2;

  Point2f(float x, float y) : sf::Vector2f(x, y) {}
  template <typename T>
  Point2f(const sf::Vector2<T>& point) : sf::Vector2f(point) {}

  float operator[](unsigned int i) const {
    if (i == 0) {
      return x;
    } else if (i == 1) {
      return y;
    } else {
      std::cerr << "invalid dimension" << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }
};