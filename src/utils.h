#include "types.h"
#include <iostream>

inline double distance_between(Point a, Point b) {
  double delta_x = a.x - b.x;
  double delta_y = a.y - b.y;
  return sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

inline void draw_on_image(std::vector<Point> points, png::rgb_pixel pixel_color,
                          png_image &image) {
  for (auto &point : points) {
    image[point.y][point.x] = pixel_color;
  }
}