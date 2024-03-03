#pragma once

#include "types.h"
#include <string>
#include <tuple>

using namespace std;

class OpenSpaceShortestPath {
protected:
  png_image _image;
  unsigned int _num_edges = 0;
  unsigned int _num_vertices = 0;
  unsigned int _image_width;
  unsigned int _image_height;

  bool is_point_in_boundaries(Point p) {
    return (p.x >= 0 && p.x < _image_width && p.y >= 0 && p.y < _image_height);
  }

  inline int point_to_vertex_label(Point p) { return p.y * _image_width + p.x; }

  inline int vertex_label_x_part(::vertex v) { return v % _image_width; }

  inline int vertex_label_y_part(::vertex v) { return v / _image_width; }

  virtual bool is_traversable_pixel(Point p) {
    return (is_point_in_boundaries(p) && _image[p.y][p.x].red < 245);
  }

  void draw_on_image_create_new(vector<Point> points,
                                png::rgb_pixel pixel_colors, string file_name) {
    png_image out_image(_image);
    for (auto &point : points) {
      out_image[point.y][point.x] = pixel_colors;
    }
    out_image.write(file_name);
  }

  virtual std::tuple<double, png_image, unsigned int>
  shortest_path_internal(Point start, Point end) = 0;

  virtual void initialize() = 0;

public:
  OpenSpaceShortestPath(string filepath) {
    _image = png_image(filepath);
    _image_height = _image.get_height();
    _image_width = _image.get_width();
  }

  unsigned int num_edges() { return _num_edges; }
  unsigned int num_vertices() { return _num_vertices; }

  /**
   * @brief Calculate the short path distance between the start
   * and end points
   *
   * @param start
   * @param end
   * @return tuple with (distance pixel distance, png_image of path, timing of
   * traversal in //TODO ? seconds)
   */
  std::tuple<double, png_image, unsigned int> shortest_path(Point start,
                                                            Point end) {
    if (!is_point_in_boundaries(start)) {
      throw error("start point is out of boundaries");
    }
    if (!is_point_in_boundaries(end)) {
      throw error("end ponit is out boundaries");
    }
    return shortest_path_internal(start, end);
  }
};