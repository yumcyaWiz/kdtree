cmake_minimum_required(VERSION 3.12)

project(kdtree LANGUAGES CXX)

# nn
add_executable(nn src/nn.cpp)
target_compile_features(nn PUBLIC cxx_std_17)
set_target_properties(nn PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(nn PUBLIC src)

find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
target_link_libraries(nn sfml-graphics sfml-window sfml-system)

# knn
add_executable(knn src/knn.cpp)
target_compile_features(knn PUBLIC cxx_std_17)
set_target_properties(knn PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(knn PUBLIC src)

find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
target_link_libraries(knn sfml-graphics sfml-window sfml-system)