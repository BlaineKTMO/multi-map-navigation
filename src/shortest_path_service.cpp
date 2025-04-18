#include "multi_map_navigation/shortest_path_service.h"
#include "multi_map_navigation/wormhole_database.h"
#include <set>
#include <ros/package.h>

namespace multi_map_navigation {

ShortestPathService::ShortestPathService(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize the service server
    service_ = nh_.advertiseService("shortest_path", &ShortestPathService::serviceCallback, this);
    
    // Initialize the graph with wormhole data
    initializeGraph();
    
    ROS_INFO("Shortest path service is ready");
}

bool ShortestPathService::serviceCallback(multi_map_navigation::ShortestPath::Request& req,
                                         multi_map_navigation::ShortestPath::Response& res) {
    ROS_INFO("Received shortest path request from '%s' to '%s'", 
             req.start_map.c_str(), req.target_map.c_str());
    
    // Get the shortest path
    res.path = map_graph_.getShortestPath(req.start_map, req.target_map);
    
    if (res.path.empty()) {
        ROS_WARN("No path found from '%s' to '%s'", req.start_map.c_str(), req.target_map.c_str());
        return false;
    }
    
    ROS_INFO("Found path with %zu maps", res.path.size());
    return true;
}

void ShortestPathService::initializeGraph() {
    // Load wormhole data from the database and build a bidirectional graph
    
    // Create a database instance with the path to the SQLite database
    std::string db_path = ros::package::getPath("multi_map_navigation") + "/wormholes.db";
    WormholeDatabase db(db_path);
    
    ROS_INFO("Loading wormhole data from: %s", db_path.c_str());
    
    // Query all maps from the database first
    std::set<std::string> maps;
    std::vector<WormholeDatabase::Wormhole> all_wormholes;
    
    // First, get wormholes from each map
    std::vector<std::string> map_list = {"mapA", "mapB", "mapC", "mapD", "mapE", "mapF", "mapG"};
    
    for (const auto& map_name : map_list) {
        auto wormholes = db.getWormholesFrom(map_name);
        
        if (!wormholes.empty()) {
            // Add all maps involved in wormholes to our set of maps
            maps.insert(map_name);
            
            // Keep track of all wormholes
            all_wormholes.insert(all_wormholes.end(), wormholes.begin(), wormholes.end());
            
            for (const auto& wormhole : wormholes) {
                maps.insert(wormhole.target_map);
            }
        }
    }
    
    // For debugging, log the maps and connections we found
    ROS_INFO("Found %zu maps in the database", maps.size());
    ROS_INFO("Found %zu wormhole connections", all_wormholes.size());
    
    // Build the bidirectional graph from all wormholes
    for (const auto& wormhole : all_wormholes) {
        // Add the edge with a uniform cost of 1
        map_graph_.addEdge(wormhole.map_name, wormhole.target_map, 1);
        
        // Log the connection for debugging
        ROS_DEBUG("Added graph edge: %s -> %s", 
                wormhole.map_name.c_str(), wormhole.target_map.c_str());
    }
    
    ROS_INFO("Bidirectional graph successfully built from wormhole database");
    
    for (const auto& map : maps) {
        std::vector<WormholeDatabase::Wormhole> wormholes = db.getWormholesFrom(map);
        
        for (const auto& wormhole : wormholes) {
            // Add an edge to the graph - each edge has a uniform cost of 1
        map_graph_.addEdge(wormhole.map_name, wormhole.target_map, 1);
        }
    }
    
    ROS_INFO("Graph initialized with wormhole data");
}

} // namespace multi_map_navigation
