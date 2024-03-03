#pragma once

#include <boost/graph/astar_search.hpp>

using namespace boost;

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