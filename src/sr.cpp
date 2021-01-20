#include "common.h"
#include "kdtree/kdtree.h"

int main() {
  const int width = 512;
  const int height = 512;
  const int n_balls = 100;
  const float r = 100;

  // create window
  sf::RenderWindow window(sf::VideoMode(width, height),
                          "kdtree - Sphere Range Search");

  // place points randomly
  std::vector<Point2f> points;
  for (int i = 0; i < n_balls; ++i) {
    points.push_back(Point2f{width * rnd(), height * rnd()});
  }

  // build kd-tree
  kdtree::KdTree<Point2f> tree(points);
  tree.buildTree();

  // setup sf circle object
  std::vector<Ball> balls;
  for (int i = 0; i < n_balls; ++i) {
    Ball ball(10.0f);
    ball.setPosition(points[i]);
    balls.emplace_back(ball);
  }

  // mouse ball
  Ball mouseBall(r);
  mouseBall.setColor(sf::Color::Blue);

  // app loop
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    window.clear(sf::Color::White);

    // draw mouse ball
    const sf::Vector2f mousePos =
        static_cast<sf::Vector2f>(sf::Mouse::getPosition(window));
    mouseBall.setPosition(mousePos);
    window.draw(mouseBall);

    // spherical range search
    std::vector<int> indices =
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

    window.display();
  }

  return 0;
}