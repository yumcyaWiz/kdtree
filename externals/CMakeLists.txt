# imgui
add_library(imgui imgui/imgui.cpp imgui/imgui_widgets.cpp imgui/imgui_draw.cpp)
target_include_directories(imgui SYSTEM PUBLIC imgui)

# imgui-sfml
add_library(imgui-sfml imgui-sfml/imgui-SFML.cpp)
target_include_directories(imgui-sfml SYSTEM PUBLIC imgui-sfml)
target_link_libraries(imgui-sfml imgui)