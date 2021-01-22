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

enum class SearchType { NN, KNN, SR };

// globals
constexpr int width = 512;
constexpr int height = 512;
kdtree::KdTree<Point2f> tree;
int n_balls = 100;
std::vector<Point2f> points;
int k = 5;
float r = 100;
std::vector<Ball> balls;

void placeBalls() {
  // place points randomly
  points.clear();
  for (int i = 0; i < n_balls; ++i) {
    points.push_back(Point2f{width * rnd(), height * rnd()});
  }

  // setup sf circle object
  balls.clear();
  for (int i = 0; i < n_balls; ++i) {
    Ball ball(5.0f);
    ball.setPosition(points[i]);
    balls.emplace_back(ball);
  }

  // rebuild kd-tree
  tree = {points};
  tree.buildTree();
}

int main() {
  // create window
  sf::RenderWindow window(sf::VideoMode(width, height), "kdtree - Search");
  window.setFramerateLimit(60);
  ImGui::SFML::Init(window);

  // setup points and balls
  placeBalls();

  // mouse ball
  Ball mouseBall(100);
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

    ImGui::SFML::Update(window, deltaClock.restart());

    // draw imgui
    ImGui::Begin("Parameters");

    if (ImGui::InputInt("number of balls", &n_balls)) {
      // resetup points and balls
      placeBalls();
    }

    static SearchType search_type = SearchType::NN;
    ImGui::Combo("Search Type", reinterpret_cast<int*>(&search_type),
                 "Nearest Neighbor\0k-Nearest Neighbor\0Spherical Range\0\0");

    switch (search_type) {
      case SearchType::KNN: {
        ImGui::InputInt("k", &k);
        break;
      }
      case SearchType::SR: {
        ImGui::InputFloat("r", &r);
        break;
      }
    }

    ImGui::End();

    window.clear(sf::Color::White);

    // draw sfml
    switch (search_type) {
      case SearchType::NN: {
        // query nearest point to mouse cursor
        const sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        const int idx_nearest =
            tree.searchNearest(Point2f(mousePos.x, mousePos.y));

        // draw balls
        for (int i = 0; i < balls.size(); ++i) {
          // make nearest point red
          if (i == idx_nearest) {
            balls[i].setColor(sf::Color::Red);
          } else {
            balls[i].setColor(sf::Color::Black);
          }

          window.draw(balls[i]);
        }

        // draw line between mouse cursor and nearest point
        sf::Vertex line[2];
        line[0].position = sf::Vector2f(mousePos);
        line[0].color = sf::Color::Blue;
        line[1].position = points[idx_nearest];
        line[1].color = sf::Color::Blue;
        window.draw(line, 2, sf::LineStrip);

        break;
      }

      case SearchType::KNN: {
        // query k-nearest point to mouse cursor
        const sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        const std::vector<int> idx_nearests =
            tree.searchKNearest(Point2f(mousePos.x, mousePos.y), k);

        // make nearest point red
        for (int i = 0; i < balls.size(); ++i) {
          balls[i].setColor(sf::Color::Black);
        }
        for (int i = 0; i < idx_nearests.size(); ++i) {
          const int idx = idx_nearests[i];
          balls[idx].setColor(sf::Color::Red);
        }

        // draw balls
        for (int i = 0; i < balls.size(); ++i) {
          window.draw(balls[i]);
        }

        // draw line between mouse cursor and nearest point
        for (int i = 0; i < idx_nearests.size(); ++i) {
          sf::Vertex line[2];
          line[0].position = sf::Vector2f(mousePos);
          line[0].color = sf::Color::Blue;
          line[1].position = points[idx_nearests[i]];
          line[1].color = sf::Color::Blue;
          window.draw(line, 2, sf::LineStrip);
        }

        break;
      }

      case SearchType::SR: {
        // draw mouse ball
        const sf::Vector2f mousePos =
            static_cast<sf::Vector2f>(sf::Mouse::getPosition(window));
        mouseBall.setRadius(r);
        mouseBall.setPosition(mousePos);
        window.draw(mouseBall);

        // spherical range search
        const std::vector<int> indices =
            tree.sphericalRangeSearch(Point2f(mousePos.x, mousePos.y), r);

        // make nearest point red
        for (int i = 0; i < balls.size(); ++i) {
          balls[i].setColor(sf::Color::Black);
        }
        for (int i = 0; i < indices.size(); ++i) {
          const int idx = indices[i];
          balls[idx].setColor(sf::Color::Red);
        }

        // draw balls
        for (int i = 0; i < balls.size(); ++i) {
          window.draw(balls[i]);
        }
        break;
      }
    }

    // render
    ImGui::SFML::Render(window);
    window.display();
  }

  ImGui::SFML::Shutdown();

  return 0;
}