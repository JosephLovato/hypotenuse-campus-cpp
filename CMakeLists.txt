cmake_minimum_required(VERSION 3.20)
project(hypotenuse_campus)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-pedantic -Wall -Wc++11-extensions -O3)
find_package(Boost 1.65.0 COMPONENTS system filesystem REQUIRED)
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

add_executable(hypotenuse_campus src/compare.cpp src/open_space_shortest_path.h src/strict_spider.h src/open_spider.h src/smart_corners.h src/types.h src/utils.h)
add_executable(smart_graph_test src/smart_test.cpp src/smart_corners.h src/types.h src/utils.h)

target_link_libraries(hypotenuse_campus ${PNG_LIBRARY})
target_link_libraries(smart_graph_test ${PNG_LIBRARY})