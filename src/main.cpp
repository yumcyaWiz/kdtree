#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>

#include "kdtree.h"

const int width = 512;
const int height = 512;
const int n_balls = 10;

// uniform random number in [0, 1]
float rnd() {
  static std::mt19937 mt(0);
  static std::uniform_real_distribution<float> dist(0, 1);
  return dist(mt);
}

// override sf::Vector2f to satisfy PointT requirements
class Point2f : public sf::Vector2f {
 public:
  static constexpr unsigned int dim = 2;

  Point2f(float x, float y) : sf::Vector2f(x, y) {}

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

int main() {
  sf::RenderWindow window(sf::VideoMode(width, height), "SFML works!");

  std::vector<Point2f> points;
  for (int i = 0; i < n_balls; ++i) {
    points.push_back(Point2f{width * rnd(), height * rnd()});
  }

  kdtree::KdTree<Point2f> tree(points);
  tree.buildTree();

  std::vector<sf::CircleShape> circles;
  for (int i = 0; i < n_balls; ++i) {
    sf::CircleShape circle;
    circle.setFillColor(sf::Color::Black);
    circle.setRadius(10.0f);
    circle.setPosition(points[i]);
    circles.push_back(circle);
  }

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    window.clear(sf::Color::White);

    // draw balls
    for (const auto& c : circles) {
      window.draw(c);
    }

    // draw kd-tree

    window.display();
  }

  return 0;
}