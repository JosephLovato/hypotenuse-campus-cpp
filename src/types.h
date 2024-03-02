#pragma once
#include <boost/graph/adjacency_list.hpp>
#include <png++/png.hpp>

using namespace boost;
using namespace png;

// png
typedef image<rgb_pixel> png_image;

// boost undirected graph
typedef float cost;
typedef adjacency_list<setS, vecS, undirectedS, no_property,
                       property<edge_weight_t, cost>>
    undirected_graph_t;
typedef property_map<undirected_graph_t, edge_weight_t>::type weight_map_t;
typedef undirected_graph_t::vertex_descriptor vertex;
typedef undirected_graph_t::edge_descriptor edge_descriptor;
typedef std::pair<int, int> edge;

// auxiliary
struct Point {
  int x;
  int y;
};