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

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/betweenness_centrality.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <math.h> // for sqrt
#include <png++/png.hpp>
#include <unordered_set>
#include <vector>

#include "open_spider.h"
#include "smart_corners.h"
#include "strict_spider.h"
#include "types.h"

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

int main(int argc, char **argv) {
  // GENERATE STRICT GRAPH
  // open image and grab some constants
  png::image<png::rgb_pixel> image_s("campus_green_v1.png");
  png::image<png::rgb_pixel> image_o("campus_v3.png");
  unsigned int image_height = image_s.get_height();
  unsigned int image_width = image_s.get_width();

  // instantiate strict spider method
  cout << "Building Spider Graph (strict paths)" << endl;
  StrictSpider strict_spider("campus_green_v1.png", false);
  strict_spider.initialize();
  cout << "num vertices: " << strict_spider.num_vertices() << endl;
  cout << "num edges: " << strict_spider.num_edges() << endl;

  // instantiate open spider method
  cout << "Building Spider Graph (open)" << endl;
  OpenSpider open_spider("campus_v3.png", false);
  open_spider.initialize();
  cout << "num vertices: " << open_spider.num_vertices() << endl;
  cout << "num edges: " << open_spider.num_edges() << endl;

  cout << "Building Smart Graph" << endl;
  SmartCorners smart_corners("campus_v3.png");
  smart_corners.initialize();
  cout << "num vertices: " << smart_corners.num_vertices() << endl;
  cout << "num edges: " << smart_corners.num_edges() << endl;

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
  for (int i = 0; i < NUM_TESTS; i++) {
    // Select random start/end points
    auto it = intersection.begin();
    std::advance(it, rand() % intersection.size());
    int start = *it;
    it = intersection.begin();
    std::advance(it, rand() % intersection.size());
    int goal = *it;

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

    cout << "Strict: " << strict_distance << endl;
    cout << "Open: " << open_distance << endl;
    cout << "Smart: " << smart_distance << endl;
    strict_distances.push_back(strict_distance);
    open_distances.push_back(open_distance);
    smart_distances.push_back(smart_distance);

    strict_image.write(string("out_images/strict") + to_string(i) + ".png");
    open_image.write(string("out_images/open") + to_string(i) + ".png");
    smart_image.write(string("out_images/smart") + to_string(i) + ".png");
  }

  // AVERAGE
  double open_average = 0;
  for (double t : open_distances)
    open_average += t;
  open_average /= NUM_TESTS;

  double strict_average = 0;
  for (double t : strict_distances)
    strict_average += t;
  strict_average /= NUM_TESTS;

  double smart_average = 0;
  for (double t : smart_distances)
    smart_average += t;
  smart_average /= NUM_TESTS;

  cout << "Open Average: " << open_average << endl;
  cout << "Strict Average: " << strict_average << endl;
  cout << "Smart Average: " << smart_average << endl;
  return 0;
}
