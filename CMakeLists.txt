cmake_minimum_required(VERSION 3.20)
project(hypotenuse_campus)

set(CMAKE_CXX_STANDARD 14)
find_package(Boost 1.65.0 COMPONENTS system filesystem REQUIRED)
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

add_executable(hypotenuse_campus compare.cpp)
add_executable(smart_graph_test smart_test.cpp smart_graph.h)

target_link_libraries(hypotenuse_campus ${PNG_LIBRARY})
target_link_libraries(smart_graph_test ${PNG_LIBRARY})