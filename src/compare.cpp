//
// Created by Joey Lovato on 11/26/21.
//
//
//=======================================================================
// Copyright (c) 2004 Kristopher Beevers
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//

#include "open_spider.h"
#include "smart_corners.h"
#include "strict_spider.h"
#include "types.h"
#include <__chrono/duration.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/betweenness_centrality.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <math.h> // for sqrt
#include <numeric>
#include <png++/png.hpp>
#include <unordered_set>
#include <vector>

#define NUM_TESTS 50

using namespace png;
using namespace std;
using namespace boost;

bool is_traversable_pixel(int y, int x, png::image<png::rgb_pixel> &image) {
  bool f = (y < image.get_height() && x < image.get_width() && y >= 0 &&
            x >= 0 && image[y][x].red < 245);
  return f;
}

bool is_traversable_pixel_green(int y, int x,
                                png::image<png::rgb_pixel> &image) {
  bool f = (y < image.get_height() && x < image.get_width() && y >= 0 &&
            x >= 0 && image[y][x].green > 245);
  return f;
}

double average(vector<double> v) {
  return accumulate(v.begin(), v.end(), 0.0) / v.size();
}

int main(int argc, char **argv) {
  // GENERATE STRICT GRAPH
  // open image and grab some constants
  png::image<png::rgb_pixel> image_s("campus_green_v1.png");
  png::image<png::rgb_pixel> image_o("campus_v3.png");
  unsigned int image_height = image_s.get_height();
  unsigned int image_width = image_s.get_width();

  // instantiate strict spider method
  cout << "Building Spider Graph (strict paths)" << endl;
  auto start_strict_spider_graph_build = chrono::high_resolution_clock::now();
  StrictSpider strict_spider("campus_green_v1.png", false);
  strict_spider.initialize();
  auto end_strict_spider_graph_build = chrono::high_resolution_clock::now();
  cout << " - num vertices: " << strict_spider.num_vertices() << endl;
  cout << " - num edges: " << strict_spider.num_edges() << endl;
  cout << " - time (ms): "
       << chrono::duration_cast<chrono::milliseconds>(
              end_strict_spider_graph_build - start_strict_spider_graph_build)
              .count()
       << endl;

  // instantiate open spider method
  cout << "Building Spider Graph (open)" << endl;
  auto start_open_spider_graph_build = chrono::high_resolution_clock::now();
  OpenSpider open_spider("campus_v3.png", false);
  open_spider.initialize();
  auto end_open_spider_graph_build = chrono::high_resolution_clock::now();
  cout << " - num vertices: " << open_spider.num_vertices() << endl;
  cout << " - num edges: " << open_spider.num_edges() << endl;
  cout << " - time (ms): "
       << chrono::duration_cast<chrono::milliseconds>(
              end_open_spider_graph_build - start_open_spider_graph_build)
              .count()
       << endl;

  cout << "Building Smart Graph" << endl;
  auto start_smart_corners_graph_build = chrono::high_resolution_clock::now();
  SmartCorners smart_corners("campus_v3.png");
  smart_corners.initialize();
  auto end_smart_corners_graph_build = chrono::high_resolution_clock::now();
  cout << " - num vertices: " << smart_corners.num_vertices() << endl;
  cout << " - num edges: " << smart_corners.num_edges() << endl;
  cout << " - time (ms): "
       << chrono::duration_cast<chrono::milliseconds>(
              end_smart_corners_graph_build - start_smart_corners_graph_build)
              .count()
       << endl;

  // build set of navigable nodes in both strict/open graphs
  // NOTE eventually we'll do something different but keeping
  // the same for consistency with previous data
  std::unordered_set<int> intersection;
  for (png::uint_32 y = 0; y < image_height; ++y) {
    for (png::uint_32 x = 0; x < image_width; ++x) {
      if (is_traversable_pixel_green(y, x, image_s) &&
          is_traversable_pixel(y, x, image_o))
        intersection.insert(y * image_width + x);
    }
  }

  // Test random start/end points on all three
  vector<double> open_distances;
  vector<double> strict_distances;
  vector<double> smart_distances;
  vector<double> open_times;
  vector<double> strict_times;
  vector<double> smart_times;
  for (int i = 0; i < NUM_TESTS; i++) {
    // Select random start/end points
    auto it = intersection.begin();
    std::advance(it, rand() % intersection.size());
    int start = *it;
    it = intersection.begin();
    std::advance(it, rand() % intersection.size());
    int goal = *it;

    cout << "Test " << i << endl;

    auto start_y = (int)(start / image_width);
    auto start_x = (int)(start % image_width);
    cout << "Start: (" << start_y << "," << start_x << ")"
         << "[#" << start << "]" << endl;
    auto goal_y = (int)(goal / image_width);
    auto goal_x = (int)(goal % image_width);
    cout << "Goal: (" << goal_y << "," << goal_x << ")"
         << "[#" << goal << "]" << endl;

    auto [strict_distance, strict_image, strict_time] =
        strict_spider.shortest_path({start_x, start_y}, {goal_x, goal_y});

    auto [open_distance, open_image, open_time] =
        open_spider.shortest_path({start_x, start_y}, {goal_x, goal_y});

    auto [smart_distance, smart_image, smart_time] =
        smart_corners.shortest_path({start_x, start_y}, {goal_x, goal_y});

    cout << "Strict Distance: " << strict_distance << endl;
    cout << "Open Distance: " << open_distance << endl;
    cout << "Smart Distance: " << smart_distance << endl;
    strict_distances.push_back(strict_distance);
    open_distances.push_back(open_distance);
    smart_distances.push_back(smart_distance);

    cout << "Strict Time (ms): " << strict_time << endl;
    cout << "Open Time (ms): " << open_time << endl;
    cout << "Smart Time (ms): " << smart_time << endl;
    strict_times.push_back(strict_time);
    open_times.push_back(open_time);
    smart_times.push_back(smart_time);

    cout << "--------------------------" << endl;

    strict_image.write(string("out_images_red/strict") + to_string(i) + ".png");
    open_image.write(string("out_images_red/open") + to_string(i) + ".png");
    smart_image.write(string("out_images_red/smart") + to_string(i) + ".png");
  }

  // print averages

  cout << "Open Average Distance (pixels): " << average(open_distances) << endl;
  cout << "Strict Average Distance (pixels): " << average(strict_distances)
       << endl;
  cout << "Smart Average Distance (pixels) " << average(smart_distances)
       << endl;

  cout << "Open Average Time (ms): " << average(open_times) << endl;
  cout << "Strict Average Time (ms): " << average(strict_times) << endl;
  cout << "Smart Average Time (ms) " << average(smart_times) << endl;
  return 0;
}
