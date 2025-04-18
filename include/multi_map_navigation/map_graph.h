#ifndef MAP_GRAPH_H
#define MAP_GRAPH_H

#include <unordered_map>
#include <vector>
#include <string>
#include <utility>
#include <queue>
#include <algorithm>

namespace multi_map_navigation {

/**
 * @brief Directed graph representation for maps and wormholes
 * 
 * This class represents a directed graph where nodes are maps and edges are wormholes.
 * It provides functionality to find the shortest path between two maps using Dijkstra's algorithm.
 */
class MapGraph {
public:
    /**
     * @brief Add an edge (wormhole) between two maps
     * 
     * @param from Source map name
     * @param to Destination map name
     * @param cost Cost of traversing from source to destination (always 1)
     */
    void addEdge(const std::string& from, const std::string& to, int cost);

    /**
     * @brief Get the shortest path between two maps
     * 
     * @param start Starting map name
     * @param goal Target map name
     * @return std::vector<std::string> Sequence of maps to traverse
     */
    std::vector<std::string> getShortestPath(const std::string& start, const std::string& goal);

private:
    std::unordered_map<std::string, std::vector<std::pair<std::string, int>>> adjacency_list_;
};

} // namespace multi_map_navigation

#endif // MAP_GRAPH_H
