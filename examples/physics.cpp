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

// globals
constexpr int width = 512;
constexpr int height = 512;
kdtree::KdTree<Point2f> tree;
int n_balls = 100;
std::vector<Point2f> points;
std::vector<Ball> balls;

int main() {}