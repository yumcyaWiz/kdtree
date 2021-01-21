# kdtree

header-only implementation of kdtree and visualization.

following features are implemented.

* Nearest neighbor search
* k-Nearest neighbor search
* Spherical range search
* Graphviz dot language output

|filename|description|
|:--|:--|
|`include/kdtree.h`|header-only implementation of kdtree|
|`examples/search.cpp`|visualization of neighbor search result|

## Requirements

* C++17
* CMake 3.12 or Higher

if you want to build visualization programs, you also need followings.

* OpenGL 3.1
* [SFML 2.5](https://github.com/SFML/SFML)

## How to integrate kdtree into your cmake project

in your CMakeLists.txt

```cmake
add_subdirectory(<path-to-kdtree>)
target_link_libraries(<your-target> kdtree)
```

## How to use kdtree in your source

```cpp
// include kdtree
#include "kdtree/kdtree.h"

// setup points 
std::vector<Point> points;
// populate points here...

// setup kdtree
kdtree::KdTree<Point> tree(points);
// build kdtree
tree.buildTree();

// nearest neighbor search
Point queryPoint = ...
int index_of_nearest = tree.searchNearest(queryPoint);

// k-nearest neighbor search
int k = 5;
std::vector<int> indices_of_k_nearest = tree.searchKNearest(queryPoint, k);

// spherical range search
float r = 1.5;
std::vector<int> indices_of_range = tree.sphericalRangeSearch(queryPoint, r);
```

**Note that `Point` must have following members.**

* `T Point::operator[](unsigned int) const`: element access
* `static unsigned int Point::dim`: number of dimension

## How to build visualization programs

Run following command to retrieve external libraries.

```bash
git submodule update --init
```

Then, set cmake option `KDTREE_VISUALIZATION` to `On` and build.

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DKDTREE_VISUALIZATION=On ..
make
```

## Gallery

### Graphviz dot language output

![](img/tree.png)

generated dot file
```dot
digraph {
0->9 [label=0]
9->6 [label=1]
null60 [label="", shape="none"]
6->null60 [label=0]
null61 [label="", shape="none"]
6->null61 [label=0]
9->4 [label=1]
null40 [label="", shape="none"]
4->null40 [label=0]
4->3 [label=0]
null30 [label="", shape="none"]
3->null30 [label=1]
null31 [label="", shape="none"]
3->null31 [label=1]
0->5 [label=0]
5->7 [label=1]
null70 [label="", shape="none"]
7->null70 [label=0]
7->8 [label=0]
null80 [label="", shape="none"]
8->null80 [label=1]
null81 [label="", shape="none"]
8->null81 [label=1]
5->2 [label=1]
null20 [label="", shape="none"]
2->null20 [label=0]
2->1 [label=0]
null10 [label="", shape="none"]
1->null10 [label=1]
null11 [label="", shape="none"]
1->null11 [label=1]
}
```

## Externals

* [SFML 2.5](https://github.com/SFML/SFML)
* [imgui](https://github.com/ocornut/imgui)
* [imgui-sfml](https://github.com/eliasdaler/imgui-sfml)

## References

* [C言語によるkd-treeの実装](https://qiita.com/fj-th/items/1bb2dc39f3088549ad6e)
* [kd-treeを実装してみた](https://atkg.hatenablog.com/entry/2016/12/18/002353)
* [kd-tree (2次元)](https://tjkendev.github.io/procon-library/cpp/range_query/kd-tree.html)
* [k-d treeによる最近傍探索](https://github.com/komi2/survey/blob/master/01/01/kdtree.md)