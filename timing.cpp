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


#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <ctime>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h> // for sqrt
#include <png++/png.hpp>
#include <unordered_set>

#define NUM_TESTS 100

using namespace png;
using namespace std;
using namespace boost;


// auxiliary types
struct location
{
    float y, x; // lat, long
};
typedef float cost;


// euclidean distance heuristic for A*
template < class Graph, class CostType >
class distance_heuristic : public astar_heuristic< Graph, CostType >
{
public:
    typedef typename graph_traits< Graph >::vertex_descriptor Vertex;
    distance_heuristic(Vertex goal, int image_width, int image_height) : m_goal(goal),
                                                                         width(image_width),
                                                                         height(image_height) {}
    CostType operator()(Vertex u)
    {
        //calculate x and y positions for goal and u
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

// euclidean distance heuristic for A*
template < class Graph, class CostType >
class null_heuristic : public astar_heuristic< Graph, CostType >
{
public:
    typedef typename graph_traits< Graph >::vertex_descriptor Vertex;
    null_heuristic(Vertex goal) : m_goal(goal) {}
    CostType operator()(Vertex u)
    {
        //return 0 to simulate dijkstra's
        return 0;
    }

private:
    Vertex m_goal;
};



struct found_goal
{
}; // exception for termination

// visitor that terminates when we find the goal
template < class Vertex >
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template < class Graph > void examine_vertex(Vertex u, Graph& g)
    {
        if (u == m_goal)
            throw found_goal();
    }

private:
    Vertex m_goal;
};

typedef adjacency_list< listS, vecS, undirectedS, no_property,
        property< edge_weight_t, cost > >
        mygraph_t;
typedef property_map< mygraph_t, edge_weight_t >::type WeightMap;
typedef mygraph_t::vertex_descriptor vertex;
typedef mygraph_t::edge_descriptor edge_descriptor;
typedef std::pair< int, int > edge;

bool is_traversable_pixel(int y, int x, png::image< png::rgb_pixel >& image) {
    bool f =  (y < image.get_height()  && x < image.get_width()  &&
               y >= 0 && x >= 0
               && image[y][x].red < 245);
    return f;
}

bool add_edge_f(int current, int y, int x, float weight, WeightMap& wm, mygraph_t& g, int image_width) {
    edge_descriptor e;
    bool inserted;
    boost::tie(e, inserted)
            = add_edge(current, y*image_width + x, g);
    wm[e] = weight;
    return inserted;
}

int main(int argc, char** argv)
{
    //open image and grab some constants
    png::image< png::rgb_pixel > image("campus_v3.png");
    unsigned int image_height = image.get_height();
    unsigned int image_width = image.get_width();
    unsigned int num_nodes = image_height * image_width;


    cout << "Converting campus to graph..." << endl;
    mygraph_t g;
    WeightMap weightmap = get(edge_weight, g);

    // We will iterate over the entire image, constructing the graph as we go
    // Pixels are valid if they are not red and are in-bounds
    for (png::uint_32 y = 0; y < image_height; ++y) {
        for (png::uint_32 x = 0; x < image_width; ++x) {
            //Check to see if this pixel can be a node (not a building)
            if(is_traversable_pixel(y, x, image)) {
                //label node by (y*width + x) number in grid
                int current_node = y*image_width + x;

                //add edges all traversable nodes (8 in all cardinal/45-degree offset directions
                //::north
                if(is_traversable_pixel(y-1, x, image)) {
                    add_edge_f(current_node, y-1, x, 1, weightmap, g, image_width);
                }
                //::west
                if(is_traversable_pixel(y, x+1, image)) {
                    add_edge_f(current_node, y, x+1, 1, weightmap, g, image_width);
                }
                //::south
                if(is_traversable_pixel(y+1, x, image)) {
                    add_edge_f(current_node, y+1, x, 1, weightmap, g, image_width);
                }
                //::east
                if(is_traversable_pixel(y, x-1, image)) {
                    add_edge_f(current_node, y, x-1, 1, weightmap, g, image_width);
                }
                //::northwest
                if(is_traversable_pixel(y-1, x+1, image)) {
                    add_edge_f(current_node, y-1, x+1, 1.141, weightmap, g, image_width);
                }
                //::southwest
                if(is_traversable_pixel(y+1, x+1, image)) {
                    add_edge_f(current_node, y+1, x+1, 1.141, weightmap, g, image_width);
                }
                //::southeast
                if(is_traversable_pixel(y+1, x-1, image)) {
                    add_edge_f(current_node, y+1, x-1, 1.141, weightmap, g, image_width);
                }
                //::northeast
                if(is_traversable_pixel(y-1, x-1, image)) {
                    add_edge_f(current_node, y-1, x-1, 1.141, weightmap, g, image_width);
                }
            }
        }
    }

    typedef mygraph_t::vertex_descriptor vertex;

    vector<double> a_star_times;
    vector<double> dijkstras_times;

    for(int i = 0; i < NUM_TESTS; i++) {

            //chose valid start and goal nodes
        mygraph_t::vertex_descriptor start;
        mygraph_t::vertex_descriptor goal;

        bool valid_start_goal = false;
        while (!valid_start_goal) {
            boost::mt19937 gen(std::time(0));
            start = random_vertex(g, gen);
            goal = random_vertex(g, gen);
            if(is_traversable_pixel(start/image_width, start % image_width, image) &&
               is_traversable_pixel(goal/image_width, goal % image_width, image)) {
                valid_start_goal = true;
            }
        }

        cout << "Start: (" << (int) (start / image_width) << "," << (int) (start % image_width) << ")" << "[#" << start << "]" << endl;
        cout << "Goal: (" << (int) (goal / image_width) << "," << (int) (goal % image_width) << ")" << "[#" << goal << "]" << endl;

        vector <mygraph_t::vertex_descriptor> p(num_vertices(g));
        vector <cost> d(num_vertices(g));
        clock_t a_star_start, a_star_stop, dijkstras_start, dijkstras_stop;
        /* ------- TIMING A* ---------*/
        a_star_start = clock();
        try {
            // call astar named parameter interface
            astar_search_tree(g, start,
                              distance_heuristic<mygraph_t, cost>(goal, image_width, image_height),
                              predecessor_map(
                                      make_iterator_property_map(p.begin(), get(vertex_index, g)))
                                      .distance_map(
                                              make_iterator_property_map(d.begin(), get(vertex_index, g)))
                                      .visitor(astar_goal_visitor<vertex>(goal)));
        }
        catch (found_goal fg) { // found a path to the goal

        }
        /* --------- DONE A* -----------*/
        a_star_stop = clock();
        double a_star_seconds = (a_star_stop - a_star_start) / double(CLOCKS_PER_SEC);

        dijkstras_start = clock();
        /* ------- TIMING DIJKSTRA'S ---------*/
        try {
            // call astar named parameter interface
            astar_search_tree(g, start,
                              null_heuristic<mygraph_t, cost>(goal),
                              predecessor_map(
                                      make_iterator_property_map(p.begin(), get(vertex_index, g)))
                                      .distance_map(
                                              make_iterator_property_map(d.begin(), get(vertex_index, g)))
                                      .visitor(astar_goal_visitor<vertex>(goal)));
        }
        catch (found_goal fg) { // found a path to the goal

        }
        /* --------- DONE DIJKSTRA'S --------*/
        dijkstras_stop = clock();
        double dijkstras_seconds = (dijkstras_stop - dijkstras_start) / double(CLOCKS_PER_SEC);

        a_star_times.push_back(a_star_seconds);
        dijkstras_times.push_back(dijkstras_seconds);
    }

    //Average times:
    double a_star_average = 0;
    for(double t: a_star_times) a_star_average += t;
    a_star_average /= NUM_TESTS;

    double dijkstras_average = 0;
    for(double t: dijkstras_times) dijkstras_average += t;
    dijkstras_average /= NUM_TESTS;

    cout << "A* Average: " << a_star_average << endl;
    cout << "Dijkstras Average: " << dijkstras_average << endl;
    return 0;
}
