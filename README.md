# kdtree

header-only implementation of kdtree and visualization.

following features are implemented.

* Nearest neighbor search
* k-Nearest neighbor search
* Spherical range search

|filename|description|
|:--|:--|
|`include/kdtree.h`|header-only implementation of kdtree|
|`examples/common.h`|common components of visualization programs|
|`examples/nn.cpp`|visualization of nearest neighbor search|
|`examples/knn.cpp`|visualization of k-nearest neighbor search|
|`examples/sr.cpp`|visualization of spherical range search|

## Requirements

* C++17
* CMake 3.12 or Higher
* [SFML 2.5](https://github.com/SFML/SFML) (if you want to build visualization programs)

## how to integrate kdtree into your cmake project

in your CMakeLists.txt

```cmake
add_subdirectory(<path-to-kdtree>)
target_link_libraries(<your-target> kdtree)
```

## how to use kdtree in your source

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

Note that `Point` must have following members.

* `T Point::operator[](unsigned int) const`: element access
* `static unsigned int Point::dim`: number of dimension

## how to build visualization programs

set cmake option `KDTREE_VISUALIZATION` to `On`

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DKDTREE_VISUALIZATION=On ..
make
```

## Externals

* [SFML 2.5](https://github.com/SFML/SFML)

## References

* [C言語によるkd-treeの実装](https://qiita.com/fj-th/items/1bb2dc39f3088549ad6e)
* [kd-treeを実装してみた](https://atkg.hatenablog.com/entry/2016/12/18/002353)
* [kd-tree (2次元)](https://tjkendev.github.io/procon-library/cpp/range_query/kd-tree.html)
* [k-d treeによる最近傍探索](https://github.com/komi2/survey/blob/master/01/01/kdtree.md)