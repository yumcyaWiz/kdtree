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

// ball entity
class Ball : public sf::Drawable {
 protected:
  sf::Vector2f position;
  float radius;
  sf::CircleShape circle;

  virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
    target.draw(circle);
  }

 public:
  static constexpr unsigned int dim = 2;

  Ball(const sf::Vector2f& position, float radius) {
    setRadius(radius);
    setPosition(position);
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineThickness(1.0);
    circle.setOutlineColor(sf::Color::Black);
  }

  float operator[](unsigned int i) const {
    if (i == 0) {
      return position.x;
    } else if (i == 1) {
      return position.y;
    } else {
      std::cerr << "invalid dimension" << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

  sf::Vector2f getPosition() const { return position; }
  float getRadius() const { return radius; }

  void setPosition(const sf::Vector2f& position) {
    this->position = position;
    circle.setPosition(position - sf::Vector2f(radius, radius));
  }

  void setRadius(float r) {
    radius = r;
    circle.setRadius(r);
  }

  void setColor(const sf::Color& color) { circle.setOutlineColor(color); }
};