cmake_minimum_required(VERSION 3.12)

project(kdtree LANGUAGES CXX)

# app
add_executable(app src/main.cpp)
target_compile_features(app PUBLIC cxx_std_17)
set_target_properties(app PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(app PUBLIC src)

# SFML
find_package(SFML 2.5 COMPONENTS graphics audio REQUIRED)
target_link_libraries(app sfml-graphics sfml-window sfml-system)
