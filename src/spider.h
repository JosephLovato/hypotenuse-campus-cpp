#pragma once

#include "boost_astar_defs.h"
#include "open_space_shortest_path.h"
#include "types.h"

class Spider : public OpenSpaceShortestPath {
protected:
  bool _add_radius_two_edges;
  undirected_graph_t _graph;
  weight_map_t _weight_map;

  // virtual bool is_traversable_pixel(Point p) override {
  //   return (is_point_in_boundaries(p) && _image[p.y][p.x].red < 245);
  // }

  bool add_edge_internal(Point from, Point to, float weight) {
    edge_descriptor e;
    bool inserted;
    boost::tie(e, inserted) = add_edge(point_to_vertex_label(from),
                                       point_to_vertex_label(to), _graph);
    _weight_map[e] = weight;
    return inserted;
  }

  std::tuple<double, png_image, unsigned int>
  shortest_path_internal(Point start, Point end) override {

    ::vertex begin = point_to_vertex_label(start);
    ::vertex goal = point_to_vertex_label(end);

    vector<::vertex> p(::num_vertices(_graph));
    vector<cost> d(::num_vertices(_graph));

    try {
      // call astar named parameter interface
      astar_search_tree(
          _graph, begin,
          distance_heuristic<undirected_graph_t, cost>(goal, _image_width,
                                                       _image_height),
          predecessor_map(
              make_iterator_property_map(p.begin(), get(vertex_index, _graph)))
              .distance_map(make_iterator_property_map(
                  d.begin(), get(vertex_index, _graph)))
              .visitor(astar_goal_visitor<::vertex>(goal)));
    } catch (found_goal fg) { // found a path to the goal
      list<::vertex> shortest_path;
      for (::vertex v = goal;; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v)
          break;
      }

      list<::vertex>::iterator spi = shortest_path.begin();

      png::image<png::rgb_pixel> out_image("campus_base_a.png");
      out_image[start.y][start.x].blue = 255;
      out_image[end.y][end.x].green = 255;
      for (++spi; spi != shortest_path.end(); ++spi) {
        out_image[vertex_label_y_part(*spi)][vertex_label_x_part(*spi)].green =
            255;
      }

      out_image.write("out_strict.png");
      return {d[goal], out_image, 0};
    }
    return {-1, png_image(), 0};
  }

public:
  Spider(string filepath, bool add_radius_two_edges)
      : OpenSpaceShortestPath(filepath) {
    this->_add_radius_two_edges = add_radius_two_edges;

    _weight_map = get(edge_weight, _graph);
  }

  void initialize() override {
    for (int y = 0; y < _image_height; ++y) {
      for (int x = 0; x < _image_width; ++x) {

        // Check to see if this pixel can be a vertex (not a building)
        if (is_traversable_pixel({x, y})) {
          _num_vertices++;
          Point from = {x, y};

          // add edges all traversable vertices (8 in all cardinal/45-degree
          // offset directions north
          if (is_traversable_pixel({x, y - 1})) {
            add_edge_internal(from, {x, y - 1}, 1);
          }
          // west
          if (is_traversable_pixel({x + 1, y})) {
            add_edge_internal(from, {x + 1, y}, 1);
          }
          // south
          if (is_traversable_pixel({x, y + 1})) {
            add_edge_internal(from, {x, y + 1}, 1);
          }
          // east
          if (is_traversable_pixel({x - 1, y})) {
            add_edge_internal(from, {x - 1, y}, 1);
          }
          // northwest
          if (is_traversable_pixel({x + 1, y - 1})) {
            add_edge_internal(from, {x + 1, y - 1}, 1.414);
          }
          // southwest
          if (is_traversable_pixel({x + 1, y + 1})) {
            add_edge_internal(from, {x + 1, y + 1}, 1.414);
          }
          // southeast
          if (is_traversable_pixel({x - 1, y + 1})) {
            add_edge_internal(from, {x - 1, y + 1}, 1.414);
          }
          // northeast
          if (is_traversable_pixel({x - 1, y - 1})) {
            add_edge_internal(from, {x - 1, y - 1}, 1.414);
          }

          // add all (radius 2) edges (going CCW)
          // north north west
          if (_add_radius_two_edges) {
            if (is_traversable_pixel({x - 1, y + 2})) {
              add_edge_internal(from, {x - 1, y + 2}, 2.236);
            }
            // north west west
            if (is_traversable_pixel({x - 2, y + 1})) {
              add_edge_internal(from, {x - 2, y + 1}, 2.236);
            }
            // south west west
            if (is_traversable_pixel({x - 2, y - 1})) {
              add_edge_internal(from, {x - 2, y - 1}, 2.236);
            }
            // south south west
            if (is_traversable_pixel({x - 1, y - 2})) {
              add_edge_internal(from, {x - 1, y - 2}, 2.236);
            }
            // south south east
            if (is_traversable_pixel({x + 1, y - 2})) {
              add_edge_internal(from, {x + 1, y - 2}, 2.236);
            }
            // south east east
            if (is_traversable_pixel({x + 2, y - 1})) {
              add_edge_internal(from, {x + 2, y - 1}, 2.236);
            }
            // north east east
            if (is_traversable_pixel({x + 2, y + 1})) {
              add_edge_internal(from, {x + 2, y + 1}, 2.236);
            }
            // north north east
            if (is_traversable_pixel({x + 1, y + 2})) {
              add_edge_internal(from, {x + 1, y + 2}, 2.236);
            }
          }
        }
      }
    }
    _num_edges = ::num_edges(_graph);
  }

  vector<::vertex> vertices() {
    auto p = boost::vertices(_graph);
    return vector<::vertex>(p.first, p.second);
  }
};