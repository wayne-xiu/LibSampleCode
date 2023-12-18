#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>

// DFS visitor
class dfs_visitor : public boost::default_dfs_visitor
{
public:
    template <typename Vertex, typename Graph>
    void discover_vertex(Vertex u, const Graph &g)
    {
        std::cout << "discovered vertex " << u << std::endl;
    }
};

void TestDFS() {
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> graph;

    graph g(4);
    boost::add_edge(0, 1, g);
    boost::add_edge(0, 2, g);
    boost::add_edge(1, 2, g);
    boost::add_edge(2, 0, g);
    boost::add_edge(2, 3, g);
    boost::add_edge(3, 3, g);

    // perform a depth-first search on the graphy
    dfs_visitor vis;
    boost::depth_first_search(g, boost::visitor(vis));
}

int main() {
    TestDFS();
    return 0;
}