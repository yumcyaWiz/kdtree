cmake_minimum_required(VERSION 3.12)

project(kdtree LANGUAGES CXX)

# kdtree
add_library(kdtree INTERFACE)
target_include_directories(kdtree INTERFACE include)

# SFML
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

# nn
add_executable(nn src/nn.cpp)
target_compile_features(nn PUBLIC cxx_std_17)
set_target_properties(nn PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(nn PRIVATE src)
target_link_libraries(nn kdtree)
target_link_libraries(nn sfml-graphics sfml-window sfml-system)

# knn
add_executable(knn src/knn.cpp)
target_compile_features(knn PUBLIC cxx_std_17)
set_target_properties(knn PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(knn PRIVATE src)
target_link_libraries(knn kdtree)
target_link_libraries(knn sfml-graphics sfml-window sfml-system)