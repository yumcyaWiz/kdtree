cmake_minimum_required(VERSION 3.12)

project(kdtree LANGUAGES CXX)

# app
add_executable(nn src/nn.cpp)
target_compile_features(nn PUBLIC cxx_std_17)
set_target_properties(nn PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(nn PUBLIC src)

# SFML
find_package(SFML 2.5 COMPONENTS graphics audio REQUIRED)
target_link_libraries(nn sfml-graphics sfml-window sfml-system)