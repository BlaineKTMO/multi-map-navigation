#include "multi_map_navigation/map_graph.h"

namespace multi_map_navigation {

void MapGraph::addEdge(const std::string& from, const std::string& to, int cost) {
    adjacency_list_[from].emplace_back(to, cost);
}

std::vector<std::string> MapGraph::getShortestPath(const std::string& start, const std::string& goal) {
    // Maps for tracking distances and previous nodes in path
    std::unordered_map<std::string, int> distances;
    std::unordered_map<std::string, std::string> previous;
    
    // Queue for BFS with pairs of (node, distance)
    std::queue<std::string> q;

    // Initialize all nodes as unvisited
    for (const auto& pair : adjacency_list_) {
        distances[pair.first] = -1; // -1 represents infinity/"not visited"
    }
    
    // Add goal map if it's not in the adjacency list
    if (distances.find(goal) == distances.end()) {
        distances[goal] = -1;
    }
    
    // Initialize starting point
    distances[start] = 0;
    q.push(start);

    // BFS algorithm - simpler approach where each edge has distance 1
    while (!q.empty()) {
        std::string current = q.front();
        q.pop();
        
        // If we've reached the goal, we can stop
        if (current == goal) break;
        
        // Process all neighbors
        if (adjacency_list_.find(current) != adjacency_list_.end()) {
            for (const auto& edge : adjacency_list_[current]) {
                std::string neighbor = edge.first;
                
                // If this node hasn't been visited yet
                if (distances.find(neighbor) == distances.end() || distances[neighbor] == -1) {
                    // Distance is just one more than current node
                    distances[neighbor] = distances[current] + 1;
                    previous[neighbor] = current;
                    q.push(neighbor);
                }
            }
        }
    }

    // Reconstruct path
    std::vector<std::string> path;
    
    // Check if a path exists
    if (previous.find(goal) == previous.end() && start != goal) {
        return path; // Empty path if no route exists
    }
    
    for (std::string at = goal; at != start; at = previous[at]) {
        path.push_back(at);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace multi_map_navigation
