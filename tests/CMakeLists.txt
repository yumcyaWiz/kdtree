add_executable(test_kdtree "test_kdtree.cpp")
target_link_libraries(test_kdtree PRIVATE kdtree)

add_executable(test_kdtree_linear "test_kdtree_linear.cpp")
target_link_libraries(test_kdtree_linear PRIVATE kdtree)

add_executable(benchmark "benchmark.cpp")
target_link_libraries(benchmark PRIVATE kdtree)