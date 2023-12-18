#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>

// dijkstra algorithm using Boost Graph Library
void dijkstra_demo() {
    using namespace boost;

    // directed graph with 5 vertices and 8 edges
    typedef adjacency_list < vecS, vecS, directedS,
        property<vertex_distance_t, int>,
        property<edge_weight_t, int> > graph;
    
    typedef graph::vertex_descriptor vertex;
    typedef graph::edge_descriptor edge;

    const int num_vertices = 5;

    graph g(num_vertices);

    const int num_edges = 8;
    // std::pair<edge, bool> edges[num_edges] = {
    //     add_edge(0, 1, g),
    //     add_edge(0, 3, g),
    //     add_edge(1, 2, g),
    //     add_edge(1, 4, g),
    //     add_edge(2, 4, g),
    //     add_edge(3, 1, g),
    //     add_edge(4, 3, g),
    //     add_edge(4, 0, g)
    // };
    // auto edge_i = edges[i].first; // edge descriptor
    edge edges[num_edges] = {
        add_edge(0, 1, g).first,
        add_edge(0, 3, g).first,
        add_edge(1, 2, g).first,
        add_edge(1, 4, g).first,
        add_edge(2, 4, g).first,
        add_edge(3, 1, g).first,
        add_edge(4, 3, g).first,
        add_edge(4, 0, g).first
    };

    std::vector<int> weights = {1, 2, 1, 2, 1, 2, 1, 2};

    property_map<graph, edge_weight_t>::type weightmap = get(edge_weight, g);
    for (std::size_t i = 0; i < num_edges; ++i)
        weightmap[edges[i]] = weights[i];
    
    std::vector<vertex> predecessors(num_vertices);
    std::vector<int> distances(num_vertices);

    vertex start_vertex = 0;
    dijkstra_shortest_paths(g, start_vertex,
        predecessor_map(&predecessors[0]).distance_map(&distances[0]));

    std::cout << "distances and parents:" << std::endl;
    graph_traits<graph>::vertex_iterator vi, vi_end;

    for (tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
        std::cout << "distance(" << start_vertex << ", " << *vi << ") = " << distances[*vi] << ", ";
        std::cout << "parent(" << *vi << ") = " << predecessors[*vi] << std::endl;
    }
}

// A* algorithm using Boost Graph Library
#include <boost/graph/astar_search.hpp>
#include <boost/graph/exception.hpp>
#include <boost/graph/graph_traits.hpp>

struct vertex_state {
    int x;
    int y;
    int f;
    int g;
    int h;
};

struct heuristic {
    heuristic(int goal_x, int goal_y) : goal_x_(goal_x), goal_y_(goal_y) {}
    double operator()(vertex_state v) {
        return std::sqrt(std::pow(goal_x_ - v.x, 2) + std::pow(goal_y_ - v.y, 2));
    }

    int goal_x_;
    int goal_y_;
};

struct state_equals {
    bool operator()(vertex_state u, vertex_state v) {
        return u.x == v.x && u.y == v.y;
    }
};

class found_goal {
public:
    found_goal(int v) : v_{v} {}

    int get_goal() const { return v_; }
private:
    int v_;
};

template <typename Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor(vertex_state goal, state_equals eq) : goal_(goal), eq_(eq) {}

    template <typename Graph>
    void examine_vertex(Vertex u, Graph &g)
    {
        if (state_equals()(g[u], goal_))
        {
            throw found_goal(u);
        }
    }

    vertex_state goal_;
    state_equals eq_;
};

// TODO: this code is not compiling, revist later
// void astar_demo() {
//     typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
//         vertex_state, boost::property<boost::edge_weight_t, int>,
//         boost::no_property, boost::listS> graph;

//     typedef boost::graph_traits<graph>::vertex_descriptor vertex;
//     typedef boost::graph_traits<graph>::edge_descriptor edge;

//     // create graph with 4 vertices and 3 edges between them
//     graph g(4);
//     add_edge(0, 1, 1, g);
//     add_edge(1, 2, 1, g);
//     add_edge(2, 3, 1, g);

//     // set the state (postion) of each vertex
//     g[0].x = 0;
//     g[0].y = 0;

//     g[1].x = 1;
//     g[1].y = 0;

//     g[2].x = 1;
//     g[2].y = 1;

//     g[3].x = 0;
//     g[3].y = 1;

//     // set the goal state
//     vertex_state goal;
//     goal.x = 0;
//     goal.y = 1;

//     // set the starting state and call A*
//     vertex_state start;
//     start.x = 0;
//     start.y = 0;
//     start.g = 0;
//     start.f = 0;

//     heuristic h(goal.x, goal.y);
//     state_equals eq;

//     std::vector<vertex> predecessors(boost::num_vertices(g));
//     std::vector<int> distances(boost::num_vertices(g));

//     try {
//         boost::astar_search(g, start, h, boost::predecessor_map(boost::make_iterator_property_map(predecessors.begin(),
//             boost::get(boost::vertex_index, g), predecessors[0])).distance_map(boost::make_iterator_property_map(distances.begin(),
//             boost::get(boost::vertex_index, g), distances[0])).visitor(astar_goal_visitor<vertex>(goal, eq)));
//     } catch (found_goal fg) {
//         // found a path to the goal
//         std::cout << "found a path from start to goal" << std::endl;

//         // output the path from start to goal by following the p array
//         vertex current = fg.get_goal();
//         std::cout << "path from start to goal:" << std::endl;
//         while (current != start) {
//             std::cout << current << " <-- ";
//             current = predecessors[current];
//         }
//         std::cout << start << std::endl;
//     } catch (boost::search_error &e) {
//         std::cout << "Search failed with error: " << e.what() << std::endl;
//     }
// }

int main() {
    dijkstra_demo();
    // astar_demo();

    return EXIT_SUCCESS;
}