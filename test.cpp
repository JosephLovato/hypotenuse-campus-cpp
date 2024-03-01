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
#include <list>
#include <math.h> // for sqrt
#include <png++/png.hpp>
#include <unordered_set>
#include <vector>

using namespace png;
using namespace std;
using namespace boost;

std::unordered_set<int> edge_nodes;
int count_loop = 0;
int count_vert = 0;
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

typedef adjacency_list<listS, vecS, undirectedS, no_property,
                       property<edge_weight_t, cost>>
    mygraph_t;
typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
typedef mygraph_t::vertex_descriptor vertex;
typedef mygraph_t::edge_descriptor edge_descriptor;
typedef std::pair<int, int> edge;

bool is_traversable_pixel(int y, int x, png::image<png::rgb_pixel> &image) {
  bool f = (y < image.get_height() && x < image.get_width() && y >= 0 &&
            x >= 0 && image[y][x].red < 245);
  // if(x > 650 && x < 705 && y > 175 && y < 215) cout << "(" << y << "," << x
  // << "):" << f << ":" << (int) image[y][x].red << endl;

  return f;
}

bool add_edge_f(int current, int y, int x, float weight, WeightMap &wm,
                mygraph_t &g, int image_width) {
  edge_nodes.insert(current);
  edge_nodes.insert(y * image_width + x);
  count_edges++;
  edge_descriptor e;
  bool inserted;
  boost::tie(e, inserted) = add_edge(current, y * image_width + x, g);
  wm[e] = weight;
  return inserted;
}

int main(int argc, char **argv) {
  // open image and grab some constants
  png::image<png::rgb_pixel> image("campus_v3.png");
  unsigned int image_height = image.get_height();
  unsigned int image_width = image.get_width();
  unsigned int num_nodes = image_height * image_width;

  ofstream nodes("nodes.txt");

  cout << "Converting campus to graph..." << endl;
  mygraph_t g;
  WeightMap weightmap = get(edge_weight, g);
  // location locations[image.get_height()*image.get_width()];
  for (png::uint_32 y = 0; y < image_height; ++y) {
    for (png::uint_32 x = 0; x < image_width; ++x) {
      // Check to see if this pixel can be a node (not a building)
      if (is_traversable_pixel(y, x, image)) {
        count_loop++;
        // label node by (y*width + x) number in grid
        int current_node = y * image_width + x;

        // add edges all traversable nodes (8 in all cardinal/45-degree offset
        // directions north
        if (is_traversable_pixel(y - 1, x, image)) {
          add_edge_f(current_node, y - 1, x, 1, weightmap, g, image_width);
        }
        // west
        if (is_traversable_pixel(y, x + 1, image)) {
          add_edge_f(current_node, y, x + 1, 1, weightmap, g, image_width);
        }
        // south
        if (is_traversable_pixel(y + 1, x, image)) {
          add_edge_f(current_node, y + 1, x, 1, weightmap, g, image_width);
        }
        // east
        if (is_traversable_pixel(y, x - 1, image)) {
          add_edge_f(current_node, y, x - 1, 1, weightmap, g, image_width);
        }
        // northwest
        if (is_traversable_pixel(y - 1, x + 1, image)) {
          add_edge_f(current_node, y - 1, x + 1, 1.141, weightmap, g,
                     image_width);
        }
        // southwest
        if (is_traversable_pixel(y + 1, x + 1, image)) {
          add_edge_f(current_node, y + 1, x + 1, 1.141, weightmap, g,
                     image_width);
        }
        // southeast
        if (is_traversable_pixel(y + 1, x - 1, image)) {
          add_edge_f(current_node, y + 1, x - 1, 1.141, weightmap, g,
                     image_width);
        }
        // northeast
        if (is_traversable_pixel(y - 1, x - 1, image)) {
          add_edge_f(current_node, y - 1, x - 1, 1.141, weightmap, g,
                     image_width);
        }
      } else {
        if (x > 650 && x < 705 && y > 175 && y < 215) {
          // cout << ++count << endl;
        }
      }
    }
  }
  nodes.close();
  ofstream verts("verts.txt");
  for (auto v : g.vertex_set()) {
    verts << v << " ";
    count_vert++;
  }
  verts.close();
  cout << "num vertices: " << num_vertices(g) << endl;
  cout << "num edges: " << num_edges(g) << endl;

  //    cout << "Calculating betweenness centrality" << endl;
  //    clock_t start = clock();
  //    std::vector<double> centrality(num_vertices(g));
  //    brandes_betweenness_centrality(
  //            g,
  //            centrality_map(
  //                    make_iterator_property_map(centrality.begin(),
  //                    get(vertex_index, g),
  //                                               double()))
  //                    .vertex_index_map(get(vertex_index,
  //                    g)).weight_map(get(edge_weight, g))
  //                    .weight_map(get(edge_weight, g)));
  //
  //    clock_t stop = clock();
  //    double seconds = (stop - start) / double(CLOCKS_PER_SEC);
  //    int max_centrality = 0;
  //    int most_central_vertex = 0;
  //    for(int i = 0; i < num_vertices(g); i++) {
  //        if(centrality[i] > max_centrality) {
  //            max_centrality = centrality[i];
  //            most_central_vertex = i;
  //        }
  //    }
  //    ofstream cent("centrality.txt");
  //    for(int i = 0; i < num_vertices(g); i++) {
  //        cent << i << " " << centrality[i] << endl;
  //    }
  //    cent.close();
  //    cout << "Time: " << seconds << endl;
  // cout << "Centrality: " << most_central_vertex << " --> " << max_centrality
  // << endl;

  typedef mygraph_t::vertex_descriptor vertex;
  // pick random start/goal
  string user_input = "y";
  while (user_input != "n") {
    mygraph_t::vertex_descriptor start;
    mygraph_t::vertex_descriptor goal;
    bool valid_start_goal = false;
    while (!valid_start_goal) {
      boost::mt19937 gen(std::time(0));
      start = random_vertex(g, gen);
      goal = random_vertex(g, gen);
      if (is_traversable_pixel(start / image_width, start % image_width,
                               image) &&
          is_traversable_pixel(goal / image_width, goal % image_width, image)) {
        valid_start_goal = true;
      }
    }

    cout << "Start: (" << (int)(start / image_width) << ","
         << (int)(start % image_width) << ")"
         << "[#" << start << "]" << endl;
    cout << "Goal: (" << (int)(goal / image_width) << ","
         << (int)(goal % image_width) << ")"
         << "[#" << goal << "]" << endl;

    vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
    vector<cost> d(num_vertices(g));
    cout << "here" << endl;
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
      cout << "Shortest path from " << start << " to " << goal << ": ";
      list<vertex>::iterator spi = shortest_path.begin();
      cout << start;

      // png::image< png::rgb_pixel > out_image("campus_base_a.png");
      png::image<png::rgb_pixel> out_image(image);
      // png::image< png::rgb_pixel > out_image("test-100x100.png");
      out_image[start / image_width][start % image_width].blue = 255;
      out_image[goal / image_width][goal % image_width].green = 255;
      for (++spi; spi != shortest_path.end(); ++spi) {
        cout << " -> " << *spi;
        out_image[*spi / image_width][*spi % image_width].green = 255;
        out_image[*spi / image_width][*spi % image_width].red = 255;
      }
      cout << endl << "Total travel time: " << d[goal] << endl;

      out_image.write("out.png");
    }

    // cout << "Didn't find a path from " << start << " to " << goal << "!" <<
    // endl;

    cout << "Run again? (y/n)" << endl;
    cin >> user_input;
  }
  return 0;
}
