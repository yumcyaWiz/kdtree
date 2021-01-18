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

int main() {
  sf::RenderWindow window(sf::VideoMode(width, height), "SFML works!");

  std::vector<kdtree::Point2f> points;
  for (int i = 0; i < n_balls; ++i) {
    points.push_back(kdtree::Point2f{width * rnd(), height * rnd()});
  }

  kdtree::KdTree<float, 2> tree(points);
  tree.buildTree();

  std::vector<sf::CircleShape> circles;
  for (int i = 0; i < n_balls; ++i) {
    sf::CircleShape circle;
    circle.setFillColor(sf::Color::Black);
    circle.setRadius(10.0f);
    circle.setPosition(sf::Vector2f(points[i][0], points[i][1]));

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