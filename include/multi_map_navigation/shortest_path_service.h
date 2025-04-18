#ifndef SHORTEST_PATH_SERVICE_H
#define SHORTEST_PATH_SERVICE_H

#include <ros/ros.h>
#include <multi_map_navigation/ShortestPath.h>
#include "multi_map_navigation/map_graph.h"

namespace multi_map_navigation {

/**
 * @brief Service handler for shortest path queries
 * 
 * This class provides a ROS service to find the shortest path between maps.
 */
class ShortestPathService {
public:
    /**
     * @brief Constructor
     * @param nh ROS NodeHandle
     */
    ShortestPathService(ros::NodeHandle& nh);

    /**
     * @brief Callback for the shortest path service
     * @param req Service request
     * @param res Service response
     * @return bool True if successful, false otherwise
     */
    bool serviceCallback(multi_map_navigation::ShortestPath::Request& req,
                         multi_map_navigation::ShortestPath::Response& res);

private:
    ros::NodeHandle& nh_;
    ros::ServiceServer service_;
    MapGraph map_graph_;

    /**
     * @brief Initialize the graph with wormhole data
     */
    void initializeGraph();
};

} // namespace multi_map_navigation

#endif // SHORTEST_PATH_SERVICE_H
