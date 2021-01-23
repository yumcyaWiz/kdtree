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

// vector utils
float norm(const sf::Vector2f& v) { return std::sqrt(v.x * v.x + v.y * v.y); }
float norm2(const sf::Vector2f& v) { return v.x * v.x + v.y * v.y; }

sf::Vector2f normalize(const sf::Vector2f& v) { return v / norm(v); }

// Ball entity with Physics
class PhysicsBall : public Ball {
 private:
  sf::Vector2f velocity;
  sf::Vector2f acceleration;
  sf::Vector2f force;
  float mass;

  virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
    target.draw(circle);
  }

 public:
  PhysicsBall(const sf::Vector2f& position, float radius)
      : Ball(position, radius) {
    mass = 3.14 * radius * radius;
  }

  sf::Vector2f getVelocity() const { return velocity; }
  float getMass() const { return mass; }

  void setVelocity(const sf::Vector2f& velocity) { this->velocity = velocity; }

  void applyForce(const sf::Vector2f& force) { this->force += force; }

  void update(float dt) {
    // a = F / m
    acceleration = force / mass;

    // euler method
    position += velocity * dt;
    setPosition(position);
    velocity += acceleration * dt;

    // clear force
    force = sf::Vector2f(0, 0);
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

  // mouse ball
  Ball mouseBall(sf::Vector2f(), 50.0f);
  mouseBall.setColor(sf::Color::Blue);

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

    // draw imgui
    ImGui::Begin("Parameters");

    if (ImGui::InputInt("Number of balls", &n_balls)) {
      // resetup balls
      placeBalls();
    }

    ImGui::InputFloat("G", &G);
    ImGui::InputFloat("Restitution", &E);
    ImGui::InputFloat("Air Drag", &K);

    ImGui::End();

    window.clear(sf::Color::White);

    // rebuild kd-tree
    tree.destructTree();
    tree = {balls};
    tree.buildTree();

    // collision check
    for (int i = 0; i < n_balls; ++i) {
      balls[i].setColor(sf::Color::Black);

      const sf::Vector2f position = balls[i].getPosition();
      const sf::Vector2f velocity = balls[i].getVelocity();
      const float mass = balls[i].getMass();
      const float radius = balls[i].getRadius();

      // collision with wall
      // reflect x
      if (position.x - radius < 0) {
        balls[i].setPosition(sf::Vector2f(radius, position.y));
        balls[i].setVelocity(E * sf::Vector2f(-velocity.x, velocity.y));
      }
      if (position.x + radius > width) {
        balls[i].setPosition(sf::Vector2f(width - radius, position.y));
        balls[i].setVelocity(E * sf::Vector2f(-velocity.x, velocity.y));
      }
      // reflect y
      if (position.y - radius < 0) {
        balls[i].setPosition(sf::Vector2f(position.x, radius));
        balls[i].setVelocity(E * sf::Vector2f(velocity.x, -velocity.y));
      }
      if (position.y + radius > height) {
        balls[i].setPosition(sf::Vector2f(position.x, height - radius));
        balls[i].setVelocity(E * sf::Vector2f(velocity.x, -velocity.y));
      }

      // collision check with kd-tree
      const std::vector<int> indices_of_collision_candidate =
          tree.sphericalRangeSearch(balls[i], 4.0f * radius);
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
        const float diff = (radius + radius2) - dist;
        if (diff > 0) {
          balls[i].setColor(sf::Color::Red);
          balls[idx].setColor(sf::Color::Red);

          const sf::Vector2f v12 = normalize(position - position2);

          balls[i].setPosition(position + 0.5f * diff * v12);
          balls[idx].setPosition(position2 - 0.5f * diff * v12);

          // update velocity
          const sf::Vector2f vel_diff = velocity - velocity2;
          const float lambda = -2.0f * E * (mass * mass2) / (mass + mass2) *
                               (vel_diff.x * v12.x + vel_diff.y * v12.y);
          balls[i].setVelocity(velocity + v12 * lambda / mass);
          balls[idx].setVelocity(velocity2 - v12 * lambda / mass2);
        }
      }
    }

    // mouse effect
    const sf::Vector2i mousePos = sf::Mouse::getPosition(window);
    mouseBall.setPosition(sf::Vector2f(mousePos));
    window.draw(mouseBall);
    if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
      const std::vector<int> indices_of_near_balls =
          tree.sphericalRangeSearch(Point2f(mousePos), mouseBall.getRadius());

      for (int i = 0; i < indices_of_near_balls.size(); ++i) {
        const int idx = indices_of_near_balls[i];
        const sf::Vector2f direction =
            normalize(balls[idx].getPosition() - sf::Vector2f(mousePos));

        balls[idx].applyForce(1000.0f * balls[idx].getMass() * direction);
      }
    }

    // compute force and update physical quantity
    for (int i = 0; i < n_balls; ++i) {
      const sf::Vector2f position = balls[i].getPosition();
      const sf::Vector2f velocity = balls[i].getVelocity();
      const float mass = balls[i].getMass();
      const float radius = balls[i].getRadius();

      // gravity
      balls[i].applyForce(sf::Vector2f(0, G * mass));

      // air drag
      balls[i].applyForce(-K * norm2(velocity) * normalize(velocity));

      // update
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