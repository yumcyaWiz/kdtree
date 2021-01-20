#include "common.h"
#include "kdtree.h"

int main() {
  const int width = 512;
  const int height = 512;
  const int n_balls = 100;

  // create window
  sf::RenderWindow window(sf::VideoMode(width, height),
                          "kdtree - Nearest Neighbor Search");

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
    Ball ball(i, 10.0f);
    ball.setPosition(points[i]);
    balls.emplace_back(ball);
  }

  // app loop
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    window.clear(sf::Color::White);

    // query nearest point to mouse cursor
    const sf::Vector2i mousePos = sf::Mouse::getPosition(window);
    int idx_nearest = tree.searchNearest(Point2f(mousePos.x, mousePos.y));

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

    window.display();
  }

  return 0;
}