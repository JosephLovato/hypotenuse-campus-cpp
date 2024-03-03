#pragma once

#include "spider.h"
#include "types.h"

class StrictSpider : public Spider {
protected:
  virtual bool is_traversable_pixel(Point p) override {
    return (is_point_in_boundaries(p) && _image[p.y][p.x].green > 245);
  }

public:
  StrictSpider(string filepath, bool add_radius_two_edges)
      : Spider(filepath, add_radius_two_edges){};
};