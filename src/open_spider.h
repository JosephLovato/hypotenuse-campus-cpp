#pragma once

#include "spider.h"
#include "types.h"

class OpenSpider : public Spider {
public:
  OpenSpider(string filepath, bool add_radius_two_edges)
      : Spider(filepath, add_radius_two_edges){};
};