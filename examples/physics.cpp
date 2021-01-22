#include <cmath>

// SFML
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>

// externals
#include "imgui-SFML.h"
#include "imgui.h"

// kdtree
#include "kdtree/kdtree.h"

//
#include "common.h"

float norm(const sf::Vector2f& v) { return std::sqrt(v.x * v.x + v.y * v.y); }
float norm2(const sf::Vector2f& v) { return v.x * v.x + v.y * v.y; }

sf::Vector2f normalize(const sf::Vector2f& v) { return v / norm(v); }

// custom sfml entity
class PhysicsBall : public sf::Drawable {
 private:
  sf::Vector2f position;
  sf::Vector2f velocity;
  sf::Vector2f acceleration;
  float mass;
  float radius;
  sf::CircleShape circle;

  virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
    target.draw(circle);
  }

 public:
  static constexpr unsigned int dim = 2;

  PhysicsBall(const sf::Vector2f& position, float radius)
      : position(position), radius(radius) {
    setPosition(position);
    setRadius(radius);
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

  void setRadius(float r) {
    radius = r;
    mass = 3.14 * r * r;
    circle.setRadius(r);
  }

  void setColor(const sf::Color& color) { circle.setOutlineColor(color); }

  sf::Vector2f getPosition() const { return position; }
  sf::Vector2f getVelocity() const { return velocity; }
  float getMass() const { return mass; }
  float getRadius() const { return radius; }

  void setPosition(const sf::Vector2f& position) {
    this->position = position;
    circle.setPosition(position - sf::Vector2f(radius, radius));
  }
  void setVelocity(const sf::Vector2f& velocity) { this->velocity = velocity; }

  void applyForce(const sf::Vector2f& force) { acceleration = force / mass; }

  void update(float dt) {
    position += velocity * dt;
    setPosition(position);
    velocity += acceleration * dt;
  }
};

// globals
constexpr int width = 512;
constexpr int height = 512;
int n_balls = 1000;
float G = 0.0f;   // gravitational acceleration
float E = 1.0f;   // coefficient of restitution
float K = 0.25f;  // coefficient of air drag
std::vector<PhysicsBall> balls;
kdtree::KdTree<PhysicsBall> tree;

void placeBalls() {
  // place balls randomly
  balls.clear();
  for (int i = 0; i < n_balls; ++i) {
    balls.emplace_back(sf::Vector2f(width * rnd(), height * rnd()), 5.0f);
    balls[i].setVelocity(
        100.0f * sf::Vector2f((2.0f * rnd() - 1.0f), (2.0f * rnd() - 1.0f)));
  }

  // build kd-tree
  tree = {balls};
  tree.buildTree();
}

int main() {
  // create window
  sf::RenderWindow window(sf::VideoMode(width, height), "kdtree - Physics");
  window.setFramerateLimit(60);
  ImGui::SFML::Init(window);

  // setup balls
  placeBalls();

  // app loop
  sf::Clock deltaClock;
  while (window.isOpen()) {
    // event handling
    sf::Event event;
    while (window.pollEvent(event)) {
      ImGui::SFML::ProcessEvent(event);

      if (event.type == sf::Event::Closed) window.close();
    }

    const sf::Time dt = deltaClock.restart();
    ImGui::SFML::Update(window, dt);

    window.clear(sf::Color::White);

    // rebuild kd-tree
    tree.destructTree();
    tree = {balls};
    tree.buildTree();

    for (int i = 0; i < n_balls; ++i) {
      const sf::Vector2f position = balls[i].getPosition();
      const sf::Vector2f velocity = balls[i].getVelocity();
      const float mass = balls[i].getMass();
      const float radius1 = balls[i].getRadius();

      // collision with wall
      // reflect x
      if (position.x < 0) {
        balls[i].setPosition(sf::Vector2f(0, position.y));
        balls[i].setVelocity(sf::Vector2f(-velocity.x, velocity.y) * E);
      }
      if (position.x > width) {
        balls[i].setPosition(sf::Vector2f(width, position.y));
        balls[i].setVelocity(sf::Vector2f(-velocity.x, velocity.y) * E);
      }
      // reflect y
      if (position.y < 0) {
        balls[i].setPosition(sf::Vector2f(position.x, 0));
        balls[i].setVelocity(sf::Vector2f(velocity.x, -velocity.y) * E);
      }
      if (position.y > height) {
        balls[i].setPosition(sf::Vector2f(position.x, height));
        balls[i].setVelocity(sf::Vector2f(velocity.x, -velocity.y) * E);
      }

      // collision check with kd-tree
      std::vector<int> indices_of_collision_candidate =
          tree.sphericalRangeSearch(balls[i], 2.0f * radius1);
      for (int k = 0; k < indices_of_collision_candidate.size(); ++k) {
        const int idx = indices_of_collision_candidate[k];
        // skip itself
        if (i == idx) continue;

        const sf::Vector2f position2 = balls[idx].getPosition();
        const sf::Vector2f velocity2 = balls[idx].getVelocity();
        const float mass2 = balls[idx].getMass();
        const float radius2 = balls[idx].getRadius();

        // check collision precisely
        const float dist = norm(position - position2);
        const float diff = (radius1 + radius2) - dist;
        if (diff > 0) {
          const sf::Vector2f v12 = normalize(position - position2);

          balls[i].setPosition(position + 0.5f * diff * v12);
          balls[idx].setPosition(position2 - 0.5f * diff * v12);

          const sf::Vector2f vel_diff = velocity - velocity2;
          const float lambda = -2.0f * E * (mass * mass2) / (mass + mass2) *
                               (vel_diff.x * v12.x + vel_diff.y * v12.y);
          balls[i].setVelocity(velocity + v12 * lambda / mass);
          balls[idx].setVelocity(velocity2 - v12 * lambda / mass2);
        }
      }

      // gravity
      sf::Vector2f force = sf::Vector2f(0, G * mass);

      // air drag
      force += -K * norm2(velocity) * normalize(velocity);

      // update
      balls[i].applyForce(force);
      balls[i].update(2.0f * dt.asSeconds());

      // draw
      window.draw(balls[i]);
    }

    // render
    ImGui::SFML::Render(window);
    window.display();
  }

  ImGui::SFML::Shutdown();

  return 0;
}