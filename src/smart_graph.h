#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/betweenness_centrality.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
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

// graph definitions
typedef float cost;
typedef adjacency_list<setS, vecS, undirectedS, no_property,
                       property<edge_weight_t, cost>>
    undirected_graph_t;
typedef property_map<undirected_graph_t, edge_weight_t>::type WeightMap;
typedef undirected_graph_t::vertex_descriptor vertex;
typedef undirected_graph_t::edge_descriptor edge_descriptor;
typedef std::pair<int, int> edge;

typedef png::image<png::rgb_pixel> png_image;

struct Point {
  int x;
  int y;
};

/**
 * @brief Return list of points (grid cells) between points a and b,
 * returns false and an empty list if hit a red obstacle in the image
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

inline double distance_between(Point a, Point b) {
  double delta_x = a.x - b.x;
  double delta_y = a.y - a.y;
  return sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

class SmartOpenSpaceSearch {
private:
  undirected_graph_t graph;
  png_image image;
  unsigned int image_height;
  unsigned int image_width;
  vector<Point> all_edge_points;

  inline int point_to_vertex_label(Point p) { return p.y * image_width + p.x; }

  bool is_traversable_pixel(int x, int y) {
    return (y < image.get_height() && x < image.get_width() && y >= 0 &&
            x >= 0 && image[y][x].red < 245);
  }

  bool is_on_corner(int x, int y) {
    int count = 0;
    if (x + 1 < image_width && image[y][x + 1].red >= 245)
      count++;
    if (x - 1 >= 0 && image[y][x - 1].red >= 245)
      count++;
    if (y + 1 < image_height && image[y + 1][x].red >= 245)
      count++;
    if (y - 1 >= 0 && image[y - 1][x].red >= 245)
      count++;

    if (x + 1 < image_width && y + 1 < image_height &&
        image[y + 1][x + 1].red >= 245)
      count++;
    if (x - 1 >= 0 && y - 1 >= 0 && image[y - 1][x - 1].red >= 245)
      count++;
    if (y + 1 < image_height && x - 1 >= 0 && image[y + 1][x - 1].red >= 245)
      count++;
    if (y - 1 >= 0 && x + 1 < image_width && image[y - 1][x + 1].red >= 245)
      count++;

    return count == 1;
  }

  void draw_on_image_create_new(vector<Point> points,
                                png::rgb_pixel pixel_colors, string file_name) {
    png_image out_image(image);
    for (auto &point : points) {
      out_image[point.y][point.x] = pixel_colors;
    }
    out_image.write(file_name);
  }

public:
  SmartOpenSpaceSearch(png_image reference_image) {
    this->image = reference_image;
    image_height = reference_image.get_height();
    image_width = reference_image.get_width();

    // build base graph
    // discover all pixels on the corner of innavigable pixels
    vector<Point> corner_points;
    for (png::uint_32 y = 0; y < image_height; y++) {
      for (png::uint_32 x = 0; x < image_width; x++) {
        if (is_on_corner(x, y))
          corner_points.push_back({static_cast<int>(x), static_cast<int>(y)});
      }
    }
    // draw_on_image_create_new(on_corners, {0, 255, 0}, "test.png");
    // draw line between each pair of corner points and connect them
    // if no obstacles
    WeightMap weight_map = get(edge_weight, graph);
    for (int i = 0; i < corner_points.size(); i++) {
      for (int j = i + 1; j < corner_points.size(); j++) {
        // calculate all grid cells between points
        auto &point_i = corner_points[i];
        auto &point_j = corner_points[j];
        bool no_obstacles;
        vector<Point> points;
        std::tie(no_obstacles, points) =
            bresenham_plot(point_i, point_j, image);
        // if no obstacle, connect two vertices
        if (no_obstacles) {
          add_edge_internal(point_i, point_j, graph, weight_map);
          all_edge_points.insert(all_edge_points.end(), points.begin(),
                                 points.end());
        }
      }
    }
    cout << "base smart graph num vertices: " << corner_points.size() << endl;
    cout << "base smart graph num edges: " << num_edges(graph) << endl;
    // draw_on_image_create_new(all_edge_points, {0, 0, 255}, "edges.png");
  }

  bool add_edge_internal(Point point_i, Point point_j,
                         undirected_graph_t &graph, WeightMap &weight_map) {
    edge_descriptor e;
    bool inserted;
    boost::tie(e, inserted) = add_edge(point_to_vertex_label(point_i),
                                       point_to_vertex_label(point_j), graph);
    weight_map[e] = distance_between(point_i, point_j);
    return inserted;
  }

  undirected_graph_t base_graph_with_start_end(Point start, Point end) {
    // copy graph
    undirected_graph_t copy(graph);

    // connect start and end points to visible vertices
    std::pair<boost::adjacency_list<>::vertex_iterator,
              boost::adjacency_list<>::vertex_iterator>
        vs = boost::vertices(copy);
    WeightMap weight_map = get(edge_weight, copy);

    for (auto i = vs.first; i != vs.second; i++) {
      auto v = *i;
      int y = (int)(v / image_width);
      int x = (int)(v % image_width);
      Point p = {x, y};
      // connect start
      bool no_obstacles_start;
      std::tie(no_obstacles_start, std::ignore) =
          bresenham_plot(start, p, image);
      // if no obstacle, connect two vertices
      if (no_obstacles_start) {
        add_edge_internal(start, p, copy, weight_map);
      }

      // connect end
      bool no_obstacles_end;
      std::tie(no_obstacles_end, std::ignore) = bresenham_plot(end, p, image);
      // if no obstacle, connect two vertices
      if (no_obstacles_end) {
        add_edge_internal(end, p, copy, weight_map);
      }
    }
    cout << "old edge count: " << num_edges(graph) << endl;
    cout << "new edge count: " << num_edges(copy) << endl;
    return copy;
  }
};