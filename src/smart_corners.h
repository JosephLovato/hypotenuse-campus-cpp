#include "boost_astar_defs.h"
#include "open_space_shortest_path.h"
#include "types.h"
#include "utils.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/betweenness_centrality.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <math.h> // for sqrt
#include <png++/png.hpp>
#include <unordered_set>
#include <vector>

using namespace png;
using namespace std;
using namespace boost;

/**
 * @brief Return list of points (grid cells) between points a and b,
 * returns false and an empty list if hit a red obstacle in the image
 *
 * @note uses Bresenham's line algorithm, as described by Alois Zingl:
 * https://zingl.github.io/bresenham.html
 *
 * @param a first point
 * @param b second point
 * @return tuple with boolean if hit no obstacles and list of points if
 * didn't hit obstacles
 */
inline std::tuple<bool, vector<Point>> bresenham_plot(Point &a, Point &b,
                                                      png_image &image) {
  int dx = abs(b.x - a.x);
  int sx = a.x < b.x ? 1 : -1;

  int dy = -abs(b.y - a.y);
  int sy = a.y < b.y ? 1 : -1;

  int err = dx + dy;
  int e2 = 0;

  int x = a.x;
  int y = a.y;

  vector<Point> points;

  while (true) {
    points.push_back({x, y});
    // short circuit if we hit obstacle
    if (image[y][x].red >= 245) {
      return {false, {}};
    }
    if (x == b.x && y == b.y)
      break;
    e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
  return {true, points};
}

class SmartCorners : public OpenSpaceShortestPath {
protected:
  undirected_graph_t _graph;
  vector<Point> _all_edge_points;
  vector<Point> _corner_points;

  bool add_edge_internal(Point point_i, Point point_j,
                         undirected_graph_t &graph, weight_map_t &weight_map) {
    edge_descriptor e;
    bool inserted;
    boost::tie(e, inserted) = add_edge(point_to_vertex_label(point_i),
                                       point_to_vertex_label(point_j), graph);
    weight_map[e] = distance_between(point_i, point_j);
    return inserted;
  }

  bool is_on_corner(int x, int y) {
    int count = 0;
    if (x + 1 < _image_width && _image[y][x + 1].red >= 245)
      count++;
    if (x - 1 >= 0 && _image[y][x - 1].red >= 245)
      count++;
    if (y + 1 < _image_height && _image[y + 1][x].red >= 245)
      count++;
    if (y - 1 >= 0 && _image[y - 1][x].red >= 245)
      count++;

    if (x + 1 < _image_width && y + 1 < _image_height &&
        _image[y + 1][x + 1].red >= 245)
      count++;
    if (x - 1 >= 0 && y - 1 >= 0 && _image[y - 1][x - 1].red >= 245)
      count++;
    if (y + 1 < _image_height && x - 1 >= 0 && _image[y + 1][x - 1].red >= 245)
      count++;
    if (y - 1 >= 0 && x + 1 < _image_width && _image[y - 1][x + 1].red >= 245)
      count++;

    return count == 1;
  }

  undirected_graph_t base_graph_with_start_end(Point start, Point end) {
    // copy graph
    auto s = chrono::high_resolution_clock::now();
    undirected_graph_t copy(_graph);
    auto t = chrono::high_resolution_clock::now();

    cout << chrono::duration_cast<chrono::milliseconds>(t - s).count() << endl;

    // connect start and end points to visible corner vertices
    weight_map_t weight_map = get(edge_weight, copy);

    for (auto p : _corner_points) {
      // connect start
      bool no_obstacles_start;
      std::tie(no_obstacles_start, std::ignore) =
          bresenham_plot(start, p, _image);
      // if no obstacle, connect two vertices
      if (no_obstacles_start) {
        add_edge_internal(start, p, copy, weight_map);
      }

      // connect end
      bool no_obstacles_end;
      std::tie(no_obstacles_end, std::ignore) = bresenham_plot(end, p, _image);
      // if no obstacle, connect two vertices
      if (no_obstacles_end) {
        add_edge_internal(end, p, copy, weight_map);
      }
    }
    return copy;
  }

  std::tuple<double, png_image, double>
  shortest_path_internal(Point start, Point end) override {

    auto clock_start = chrono::high_resolution_clock::now();

    undirected_graph_t c_graph = base_graph_with_start_end(start, end);

    ::vertex begin = point_to_vertex_label(start);
    ::vertex goal = point_to_vertex_label(end);

    vector<::vertex> p(::num_vertices(c_graph));
    vector<cost> d(::num_vertices(c_graph));

    try {
      // call astar named parameter interface
      astar_search_tree(
          c_graph, begin,
          distance_heuristic<undirected_graph_t, cost>(goal, _image_width,
                                                       _image_height),
          predecessor_map(
              make_iterator_property_map(p.begin(), get(vertex_index, c_graph)))
              .distance_map(make_iterator_property_map(
                  d.begin(), get(vertex_index, c_graph)))
              .visitor(astar_goal_visitor<::vertex>(goal)));
    } catch (found_goal fg) { // found a path to the goal
      // auto clock_stop = chrono::high_resolution_clock::now();

      list<::vertex> shortest_path;
      for (::vertex v = goal;; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v)
          break;
      }

      auto clock_stop = chrono::high_resolution_clock::now();

      list<::vertex>::iterator spi = shortest_path.begin();

      png::image<png::rgb_pixel> out_image("campus_v3.png");
      out_image[start.y][start.x].blue = 255;
      out_image[end.y][end.x].green = 255;
      vector<Point> full_route;
      ::vertex current = *spi;
      ++spi;
      for (; spi != shortest_path.end(); ++spi) {
        vector<Point> line;
        Point a = {vertex_label_x_part(current), vertex_label_y_part(current)};
        Point b = {vertex_label_x_part(*spi), vertex_label_y_part(*spi)};
        std::tie(std::ignore, line) = bresenham_plot(a, b, _image);
        full_route.insert(full_route.end(), line.begin(), line.end());
        current = *spi;
      }
      draw_on_image(full_route, {252, 3, 248}, out_image);

      return {
          d[goal], out_image,
          chrono::duration_cast<chrono::milliseconds>(clock_stop - clock_start)
              .count()};
    }
    return {-1, png_image(), -1};
  }

public:
  SmartCorners(string filepath) : OpenSpaceShortestPath(filepath) {}

  void initialize() override {

    // build base graph
    // discover all pixels on the corner of innavigable pixels
    for (png::uint_32 y = 0; y < _image_height; y++) {
      for (png::uint_32 x = 0; x < _image_width; x++) {
        if (is_on_corner(x, y))
          _corner_points.push_back({static_cast<int>(x), static_cast<int>(y)});
      }
    }
    _num_vertices = _corner_points.size();
    // draw_on_image_create_new(on_corners, {0, 255, 0}, "test.png");
    // draw line between each pair of corner points and connect them
    // if no obstacles
    weight_map_t weight_map = get(edge_weight, _graph);
    for (int i = 0; i < _corner_points.size(); i++) {
      for (int j = i + 1; j < _corner_points.size(); j++) {
        // calculate all grid cells between points
        auto &point_i = _corner_points[i];
        auto &point_j = _corner_points[j];
        bool no_obstacles;
        vector<Point> points;
        std::tie(no_obstacles, points) =
            bresenham_plot(point_i, point_j, _image);
        // if no obstacle, connect two vertices
        if (no_obstacles) {
          add_edge_internal(point_i, point_j, _graph, weight_map);
          _all_edge_points.insert(_all_edge_points.end(), points.begin(),
                                  points.end());
        }
      }
    }
    _num_edges = ::num_edges(_graph);
    // draw_on_image_create_new(all_edge_points, {0, 0, 255}, "edges.png");
  }
};