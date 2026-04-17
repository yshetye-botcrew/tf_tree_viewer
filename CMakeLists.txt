cmake_minimum_required(VERSION 3.16)
project(tf_tree_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(message_manager REQUIRED)

add_executable(tf_tree_viewer src/tf_tree_viewer.cpp)
target_link_libraries(tf_tree_viewer PRIVATE message_manager::message_manager)

install(TARGETS tf_tree_viewer RUNTIME DESTINATION bin)
