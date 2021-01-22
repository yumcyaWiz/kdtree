cmake_minimum_required(VERSION 3.12)

project(kdtree LANGUAGES CXX)

option(KDTREE_VISUALIZATION "build visualization programs or not" OFF)

# kdtree
add_library(kdtree INTERFACE)
target_compile_features(kdtree INTERFACE cxx_std_17)
target_include_directories(kdtree INTERFACE include)

if(KDTREE_VISUALIZATION)
  # SFML
  find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

  # OpenGL
  find_package(OpenGL REQUIRED)

  # externals
  add_subdirectory(externals)

  # examples
  add_library(examples INTERFACE)
  target_compile_features(examples INTERFACE cxx_std_17)
  target_include_directories(examples INTERFACE examples)
  target_link_libraries(examples INTERFACE kdtree)
  target_link_libraries(examples INTERFACE sfml-graphics sfml-window sfml-system)
  target_link_libraries(examples INTERFACE OpenGL::GL)
  target_link_libraries(examples INTERFACE imgui)
  target_link_libraries(examples INTERFACE imgui-sfml)

  add_executable(search examples/search.cpp)
  set_target_properties(search PROPERTIES CXX_EXTENSIONS OFF)
  target_link_libraries(search PRIVATE examples)
  add_executable(physics examples/physics.cpp)
  set_target_properties(physics PROPERTIES CXX_EXTENSIONS OFF)
  target_link_libraries(physics PRIVATE examples)

endif()