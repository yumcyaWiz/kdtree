cmake_minimum_required(VERSION 3.12)

project(kdtree LANGUAGES CXX)

# app
add_executable(nns src/nns.cpp)
target_compile_features(nns PUBLIC cxx_std_17)
set_target_properties(nns PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(nns PUBLIC src)

# SFML
find_package(SFML 2.5 COMPONENTS graphics audio REQUIRED)
target_link_libraries(nns sfml-graphics sfml-window sfml-system)