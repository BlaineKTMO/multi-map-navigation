#ifndef WORMHOLE_DATABASE_H
#define WORMHOLE_DATABASE_H

#include <string>
#include <vector>
#include <utility>
#include <sqlite3.h>

namespace multi_map_navigation {

/**
 * @brief Class to manage wormhole positions in a database
 * 
 * This class handles the storage and retrieval of wormhole positions
 * between different maps using an SQLite database.
 */
class WormholeDatabase {
public:
    /**
     * @brief Structure to represent a wormhole position
     */
    struct Wormhole {
        std::string map_name;    ///< Source map name
        std::string target_map;  ///< Destination map name
        double x;                ///< X position of wormhole in source map
        double y;                ///< Y position of wormhole in source map
        double yaw;              ///< Yaw orientation of wormhole in source map
    };

    /**
     * @brief Constructor
     * @param db_path Path to the SQLite database file
     */
    WormholeDatabase(const std::string& db_path);

    /**
     * @brief Destructor
     */
    ~WormholeDatabase();

    /**
     * @brief Add a new wormhole to the database
     * @param wormhole Wormhole position data
     * @return bool True if successful, false otherwise
     */
    bool addWormhole(const Wormhole& wormhole);

    /**
     * @brief Get all wormholes from a specific map
     * @param map_name Name of the map
     * @return std::vector<Wormhole> List of wormholes
     */
    std::vector<Wormhole> getWormholesFrom(const std::string& map_name);

    /**
     * @brief Get the wormhole between two specific maps
     * @param from_map Source map name
     * @param to_map Destination map name
     * @return Wormhole The wormhole data or empty wormhole if not found
     */
    Wormhole getWormholeBetween(const std::string& from_map, const std::string& to_map);

private:
    sqlite3* db_;
    bool initializeDatabase();
    
    /**
     * @brief Populate the database with predefined wormhole data for maps A-G
     * 
     * This method inserts wormhole positions for each map into the database.
     * It is called during initialization if the database is empty.
     * 
     * @return bool True if successful, false otherwise
     */
    bool populateDefaultWormholes();
};

} // namespace multi_map_navigation

#endif // WORMHOLE_DATABASE_H
