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

#define NUM_TESTS 50

using namespace png;
using namespace std;
using namespace boost;

int count_nodes = 0;
int count_edges = 0;

// auxiliary types
struct location {
  float y, x; // lat, long
};
typedef float cost;

// euclidean distance heuristic
template <class Graph, class CostType>
class distance_heuristic : public astar_heuristic<Graph, CostType> {
public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  distance_heuristic(Vertex goal, int image_width, int image_height)
      : m_goal(goal), width(image_width), height(image_height) {}
  CostType operator()(Vertex u) {
    // calculate x and y positions for goal and u
    int u_x = u % width;
    int u_y = u / width;
    int goal_x = m_goal % width;
    int goal_y = m_goal / width;
    CostType dx = goal_x - u_x;
    CostType dy = goal_y - u_y;
    return ::sqrt(dx * dx + dy * dy);
  }

private:
  Vertex m_goal;
  int width;
  int height;
};

struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor {
public:
  astar_goal_visitor(Vertex goal) : m_goal(goal) {}
  template <class Graph> void examine_vertex(Vertex u, Graph &g) {
    if (u == m_goal)
      throw found_goal();
  }

private:
  Vertex m_goal;
};

typedef adjacency_list<setS, vecS, undirectedS, no_property,
                       property<edge_weight_t, cost>>
    mygraph_t;
typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
typedef mygraph_t::vertex_descriptor vertex;
typedef mygraph_t::edge_descriptor edge_descriptor;
typedef std::pair<int, int> edge;

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

bool add_edge_f(int current, int y, int x, float weight, WeightMap &wm,
                mygraph_t &g, int image_width) {
  count_edges++;
  edge_descriptor e;
  bool inserted;
  boost::tie(e, inserted) = add_edge(current, y * image_width + x, g);
  wm[e] = weight;
  return inserted;
}

int strict_a_star(int start_in, int goal_in, mygraph_t g_s, int image_width,
                  int image_height) {
  typedef mygraph_t::vertex_descriptor vertex;

  mygraph_t::vertex_descriptor start = start_in;
  mygraph_t::vertex_descriptor goal = goal_in;

  vector<mygraph_t::vertex_descriptor> p(num_vertices(g_s));
  vector<cost> d(num_vertices(g_s));

  try {
    // call astar named parameter interface
    astar_search_tree(
        g_s, start,
        distance_heuristic<mygraph_t, cost>(goal, image_width, image_height),
        predecessor_map(
            make_iterator_property_map(p.begin(), get(vertex_index, g_s)))
            .distance_map(
                make_iterator_property_map(d.begin(), get(vertex_index, g_s)))
            .visitor(astar_goal_visitor<vertex>(goal)));
  } catch (found_goal fg) { // found a path to the goal
    list<vertex> shortest_path;
    for (vertex v = goal;; v = p[v]) {
      shortest_path.push_front(v);
      if (p[v] == v)
        break;
    }

    list<vertex>::iterator spi = shortest_path.begin();

    png::image<png::rgb_pixel> out_image("campus_base_a.png");
    out_image[start / image_width][start % image_width].blue = 255;
    out_image[goal / image_width][goal % image_width].green = 255;
    for (++spi; spi != shortest_path.end(); ++spi) {
      out_image[*spi / image_width][*spi % image_width].green = 255;
    }

    out_image.write("out_strict.png");
    return d[goal];
  }
  return -1;
}

int open_a_star(int start_in, int goal_in, mygraph_t g, int image_width,
                int image_height) {

  typedef mygraph_t::vertex_descriptor vertex;

  mygraph_t::vertex_descriptor start = start_in;
  mygraph_t::vertex_descriptor goal = goal_in;

  vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
  vector<cost> d(num_vertices(g));

  try {
    // call astar named parameter interface
    astar_search_tree(
        g, start,
        distance_heuristic<mygraph_t, cost>(goal, image_width, image_height),
        predecessor_map(
            make_iterator_property_map(p.begin(), get(vertex_index, g)))
            .distance_map(
                make_iterator_property_map(d.begin(), get(vertex_index, g)))
            .visitor(astar_goal_visitor<vertex>(goal)));
  } catch (found_goal fg) { // found a path to the goal
    list<vertex> shortest_path;
    for (vertex v = goal;; v = p[v]) {
      shortest_path.push_front(v);
      if (p[v] == v)
        break;
    }

    list<vertex>::iterator spi = shortest_path.begin();

    png::image<png::rgb_pixel> out_image("campus_base_a.png");
    out_image[start / image_width][start % image_width].blue = 255;
    out_image[goal / image_width][goal % image_width].green = 255;
    for (++spi; spi != shortest_path.end(); ++spi) {
      out_image[*spi / image_width][*spi % image_width].green = 255;
      out_image[*spi / image_width][*spi % image_width].red = 255;
    }

    out_image.write("out_open.png");
    return d[goal];
  }
  return -1;
}

int main(int argc, char **argv) {
  // GENERATE STRICT GRAPH
  // open image and grab some constants
  png::image<png::rgb_pixel> image_s("campus_green_v1.png");
  unsigned int image_height = image_s.get_height();
  unsigned int image_width = image_s.get_width();
  unsigned int count_strict_vertices = 0;

  cout << "Converting campus to graph (strict paths only)..." << endl;
  mygraph_t g_s;
  WeightMap weightmap_s = get(edge_weight, g_s);

  for (png::uint_32 y = 0; y < image_height; ++y) {
    for (png::uint_32 x = 0; x < image_width; ++x) {
      // Check to see if this pixel can be a node (not a building)
      if (is_traversable_pixel_green(y, x, image_s)) {
        count_strict_vertices++;
        // label node by (y*width + x) number in grid
        int current_node = y * image_width + x;

        // add edges all traversable nodes (8 in all cardinal/45-degree offset
        // directions north
        if (is_traversable_pixel_green(y - 1, x, image_s)) {
          add_edge_f(current_node, y - 1, x, 1, weightmap_s, g_s, image_width);
        }
        // west
        if (is_traversable_pixel_green(y, x + 1, image_s)) {
          add_edge_f(current_node, y, x + 1, 1, weightmap_s, g_s, image_width);
        }
        // south
        if (is_traversable_pixel_green(y + 1, x, image_s)) {
          add_edge_f(current_node, y + 1, x, 1, weightmap_s, g_s, image_width);
        }
        // east
        if (is_traversable_pixel_green(y, x - 1, image_s)) {
          add_edge_f(current_node, y, x - 1, 1, weightmap_s, g_s, image_width);
        }
        // northwest
        if (is_traversable_pixel_green(y - 1, x + 1, image_s)) {
          add_edge_f(current_node, y - 1, x + 1, 1.414, weightmap_s, g_s,
                     image_width);
        }
        // southwest
        if (is_traversable_pixel_green(y + 1, x + 1, image_s)) {
          add_edge_f(current_node, y + 1, x + 1, 1.414, weightmap_s, g_s,
                     image_width);
        }
        // southeast
        if (is_traversable_pixel_green(y + 1, x - 1, image_s)) {
          add_edge_f(current_node, y + 1, x - 1, 1.414, weightmap_s, g_s,
                     image_width);
        }
        // northeast
        if (is_traversable_pixel_green(y - 1, x - 1, image_s)) {
          add_edge_f(current_node, y - 1, x - 1, 1.414, weightmap_s, g_s,
                     image_width);
        }

        // add all (radius 2) edges (going CCW)
        // north north west
        if (is_traversable_pixel_green(y + 2, x - 1, image_s)) {
          add_edge_f(current_node, y + 2, x - 1, 2.236, weightmap_s, g_s,
                     image_width);
        }
        // north west west
        if (is_traversable_pixel_green(y + 1, x - 2, image_s)) {
          add_edge_f(current_node, y + 1, x - 2, 2.236, weightmap_s, g_s,
                     image_width);
        }
        // south west west
        if (is_traversable_pixel_green(y - 1, x - 2, image_s)) {
          add_edge_f(current_node, y - 1, x - 2, 2.236, weightmap_s, g_s,
                     image_width);
        }
        // south south west
        if (is_traversable_pixel_green(y - 2, x - 1, image_s)) {
          add_edge_f(current_node, y - 2, x - 1, 2.236, weightmap_s, g_s,
                     image_width);
        }
        // south south east
        if (is_traversable_pixel_green(y - 2, x + 1, image_s)) {
          add_edge_f(current_node, y - 2, x + 1, 2.236, weightmap_s, g_s,
                     image_width);
        }
        // south east east
        if (is_traversable_pixel_green(y - 1, x + 2, image_s)) {
          add_edge_f(current_node, y - 1, x + 2, 2.236, weightmap_s, g_s,
                     image_width);
        }
        // north east east
        if (is_traversable_pixel_green(y + 1, x + 2, image_s)) {
          add_edge_f(current_node, y + 1, x + 2, 2.236, weightmap_s, g_s,
                     image_width);
        }
        // north north east
        if (is_traversable_pixel_green(y + 2, x + 1, image_s)) {
          add_edge_f(current_node, y + 2, x + 1, 2.236, weightmap_s, g_s,
                     image_width);
        }
      }
    }
  }

  cout << "num vertices: " << count_strict_vertices << endl;
  cout << "num edges: " << num_edges(g_s) << endl;

  // GENERATE OPEN GRAPH
  // open image and grab some constants
  png::image<png::rgb_pixel> image_o("campus_v3.png");

  unsigned int count_open_vertices = 0;

  cout << "Converting campus to graph..." << endl;
  mygraph_t g_o;
  WeightMap weightmap_o = get(edge_weight, g_o);
  // location locations[image.get_height()*image.get_width()];
  for (png::uint_32 y = 0; y < image_height; ++y) {
    for (png::uint_32 x = 0; x < image_width; ++x) {
      // Check to see if this pixel can be a node (not a building)
      if (is_traversable_pixel(y, x, image_o)) {
        count_open_vertices++;
        // label node by (y*width + x) number in grid
        int current_node = y * image_width + x;

        // add edges all traversable nodes (8 in all cardinal/45-degree offset
        // directions north
        if (is_traversable_pixel(y - 1, x, image_o)) {
          add_edge_f(current_node, y - 1, x, 1, weightmap_o, g_o, image_width);
        }
        // west
        if (is_traversable_pixel(y, x + 1, image_o)) {
          add_edge_f(current_node, y, x + 1, 1, weightmap_o, g_o, image_width);
        }
        // south
        if (is_traversable_pixel(y + 1, x, image_o)) {
          add_edge_f(current_node, y + 1, x, 1, weightmap_o, g_o, image_width);
        }
        // east
        if (is_traversable_pixel(y, x - 1, image_o)) {
          add_edge_f(current_node, y, x - 1, 1, weightmap_o, g_o, image_width);
        }
        // northwest
        if (is_traversable_pixel(y - 1, x + 1, image_o)) {
          add_edge_f(current_node, y - 1, x + 1, 1.414, weightmap_o, g_o,
                     image_width);
        }
        // southwest
        if (is_traversable_pixel(y + 1, x + 1, image_o)) {
          add_edge_f(current_node, y + 1, x + 1, 1.414, weightmap_o, g_o,
                     image_width);
        }
        // southeast
        if (is_traversable_pixel(y + 1, x - 1, image_o)) {
          add_edge_f(current_node, y + 1, x - 1, 1.414, weightmap_o, g_o,
                     image_width);
        }
        // northeast
        if (is_traversable_pixel(y - 1, x - 1, image_o)) {
          add_edge_f(current_node, y - 1, x - 1, 1.414, weightmap_o, g_o,
                     image_width);
        }

        // add all (radius 2) edges (going CCW)
        // north north west
        if (is_traversable_pixel(y + 2, x - 1, image_o)) {
          add_edge_f(current_node, y + 2, x - 1, 2.236, weightmap_o, g_o,
                     image_width);
        }
        // north west west
        if (is_traversable_pixel(y + 1, x - 2, image_o)) {
          add_edge_f(current_node, y + 1, x - 2, 2.236, weightmap_o, g_o,
                     image_width);
        }
        // south west west
        if (is_traversable_pixel(y - 1, x - 2, image_o)) {
          add_edge_f(current_node, y - 1, x - 2, 2.236, weightmap_o, g_o,
                     image_width);
        }
        // south south west
        if (is_traversable_pixel(y - 2, x - 1, image_o)) {
          add_edge_f(current_node, y - 2, x - 1, 2.236, weightmap_o, g_o,
                     image_width);
        }
        // south south east
        if (is_traversable_pixel(y - 2, x + 1, image_o)) {
          add_edge_f(current_node, y - 2, x + 1, 2.236, weightmap_o, g_o,
                     image_width);
        }
        // south east east
        if (is_traversable_pixel(y - 1, x + 2, image_o)) {
          add_edge_f(current_node, y - 1, x + 2, 2.236, weightmap_o, g_o,
                     image_width);
        }
        // north east east
        if (is_traversable_pixel(y + 1, x + 2, image_o)) {
          add_edge_f(current_node, y + 1, x + 2, 2.236, weightmap_o, g_o,
                     image_width);
        }
        // north north east
        if (is_traversable_pixel(y + 2, x + 1, image_o)) {
          add_edge_f(current_node, y + 2, x + 1, 2.236, weightmap_o, g_o,
                     image_width);
        }
      }
    }
  }

  //   std::pair<boost::adjacency_list<>::vertex_iterator,
  //             boost::adjacency_list<>::vertex_iterator>
  //       vs = boost::vertices(g_o);

  //   std::copy(vs.first, vs.second,
  //             std::ostream_iterator<boost::adjacency_list<>::vertex_descriptor>{
  //                 std::cout, "\n"});

  cout << "num vertices: " << count_open_vertices << endl;
  cout << "num edges: " << num_edges(g_o) << endl;

  // BUILD SET OF NAVIGABLE NODES IN BOTH GRAPHS
  std::unordered_set<int> intersection;
  for (png::uint_32 y = 0; y < image_height; ++y) {
    for (png::uint_32 x = 0; x < image_width; ++x) {
      if (is_traversable_pixel_green(y, x, image_s) &&
          is_traversable_pixel(y, x, image_o))
        intersection.insert(y * image_width + x);
    }
  }

  // TEST OPEN VS STRICT GRAPHS ON RANDOM STARTS AND GOALS
  vector<double> open_distances;
  vector<double> strict_distances;
  for (int i = 0; i < NUM_TESTS; i++) {
    // SELECT RANDOM START AND GOAL
    auto it = intersection.begin();
    std::advance(it, rand() % intersection.size());
    int start = *it;
    it = intersection.begin();
    std::advance(it, rand() % intersection.size());
    int goal = *it;

    cout << "Start: (" << (int)(start / image_width) << ","
         << (int)(start % image_width) << ")"
         << "[#" << start << "]" << endl;
    cout << "Goal: (" << (int)(goal / image_width) << ","
         << (int)(goal % image_width) << ")"
         << "[#" << goal << "]" << endl;

    double strict = strict_a_star(start, goal, g_s, image_width, image_height);
    double open = open_a_star(start, goal, g_o, image_width, image_height);
    cout << "Strict: " << strict << endl;
    cout << "Open: " << open << endl;
    open_distances.push_back(open);
    strict_distances.push_back(strict);
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

  cout << "Open Average: " << open_average << endl;
  cout << "Strict Average: " << strict_average << endl;
  return 0;
}
