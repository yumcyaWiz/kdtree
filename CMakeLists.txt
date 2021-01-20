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

  # search
  add_executable(search examples/search.cpp)
  target_compile_features(search PUBLIC cxx_std_17)
  set_target_properties(search PROPERTIES CXX_EXTENSIONS OFF)
  target_include_directories(search PRIVATE examples)
  target_link_libraries(search kdtree)
  target_link_libraries(search sfml-graphics sfml-window sfml-system)
  target_link_libraries(search OpenGL::GL)
  target_link_libraries(search imgui)
  target_link_libraries(search imgui-sfml)

endif()