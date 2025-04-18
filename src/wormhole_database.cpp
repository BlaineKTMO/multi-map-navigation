#include "multi_map_navigation/wormhole_database.h"
#include <ros/ros.h>

namespace multi_map_navigation {

WormholeDatabase::WormholeDatabase(const std::string& db_path) : db_(nullptr) {
    // Open the SQLite database
    if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
        ROS_ERROR("Failed to open SQLite database: %s", sqlite3_errmsg(db_));
    } else {
        initializeDatabase();
    }
}

WormholeDatabase::~WormholeDatabase() {
    if (db_) {
        sqlite3_close(db_);
    }
}

bool WormholeDatabase::initializeDatabase() {
    // Create the wormhole table if it doesn't exist
    const char* create_table_query = 
        "CREATE TABLE IF NOT EXISTS wormholes ("
        "id INTEGER PRIMARY KEY, "
        "map_name TEXT, "        // The map this wormhole belongs to
        "target_map TEXT, "      // The map this wormhole leads to
        "x REAL, "              // X position of this wormhole in map_name
        "y REAL, "              // Y position of this wormhole in map_name
        "yaw REAL, "            // Yaw orientation of this wormhole in map_name
        "UNIQUE(map_name, target_map));";
        
    char* err_msg = nullptr;
    if (sqlite3_exec(db_, create_table_query, nullptr, nullptr, &err_msg) != SQLITE_OK) {
        ROS_ERROR("Failed to create wormhole table: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    // Check if the database is empty and populate it if needed
    const char* count_query = "SELECT COUNT(*) FROM wormholes;";
    sqlite3_stmt* stmt;
    
    if (sqlite3_prepare_v2(db_, count_query, -1, &stmt, nullptr) == SQLITE_OK) {
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            int count = sqlite3_column_int(stmt, 0);
            sqlite3_finalize(stmt);
            
            // If the database is empty, populate it with default wormholes
            if (count == 0) {
                ROS_INFO("Populating database with default wormholes");
                return populateDefaultWormholes();
            }
        } else {
            sqlite3_finalize(stmt);
            return false;
        }
    } else {
        ROS_ERROR("Failed to check if database is empty: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    ROS_INFO("Wormhole database initialized successfully");
    return true;
}

bool WormholeDatabase::populateDefaultWormholes() {
    // Define wormhole connections between maps
    // Each map has wormholes to its neighboring maps
    std::vector<Wormhole> wormholes = {
        // Forward connections (A->B->C->D->E->F->G)
        { "mapA", "mapB", 3.19, -2.68, 0.0 },     // A to B
        { "mapB", "mapC", 6.37, 0.01, 1.57 },     // B to C
        { "mapC", "mapD", 3.96, 4.38, 1.57 },     // C to D
        { "mapD", "mapE", -0.93, 6.11, 3.14 },    // D to E
        { "mapE", "mapF", -5.88, 5.33, 3.14 },    // E to F
        { "mapF", "mapG", -7.89, -2.32, -1.57 },  // F to G
        
        // Reverse connections (B->A, C->B, etc.)
        // The start position in each map is at the same position as the forward wormhole
        // but with yaw rotated 180 degrees
        { "mapB", "mapA", 3.19, -2.68, 0.0 + M_PI },  // B back to A
        { "mapC", "mapB", 6.37, 0.01, 1.57 + M_PI },  // C back to B
        { "mapD", "mapC", 3.96, 4.38, 1.57 + M_PI },  // D back to C
        { "mapE", "mapD", -0.93, 6.11, 3.14 + M_PI }, // E back to D
        { "mapF", "mapE", -5.88, 5.33, 3.14 + M_PI }, // F back to E
        { "mapG", "mapF", -7.89, -2.32, -1.57 + M_PI } // G back to F
    };
    
    // Insert each wormhole into the database
    for (const auto& wormhole : wormholes) {
        if (!addWormhole(wormhole)) {
            ROS_ERROR("Failed to add wormhole from %s to %s", 
                      wormhole.map_name.c_str(), wormhole.target_map.c_str());
            return false;
        }
    }
    
    ROS_INFO("Successfully populated database with default wormholes");
    return true;
}

bool WormholeDatabase::addWormhole(const Wormhole& wormhole) {
    const char* insert_query = 
        "INSERT OR REPLACE INTO wormholes "
        "(map_name, target_map, x, y, yaw) "
        "VALUES (?, ?, ?, ?, ?);";
        
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, insert_query, -1, &stmt, nullptr) != SQLITE_OK) {
        ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    sqlite3_bind_text(stmt, 1, wormhole.map_name.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, wormhole.target_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_double(stmt, 3, wormhole.x);
    sqlite3_bind_double(stmt, 4, wormhole.y);
    sqlite3_bind_double(stmt, 5, wormhole.yaw);
    
    bool success = (sqlite3_step(stmt) == SQLITE_DONE);
    sqlite3_finalize(stmt);
    
    if (success) {
        ROS_INFO("Added wormhole: %s -> %s", wormhole.map_name.c_str(), wormhole.target_map.c_str());
    } else {
        ROS_ERROR("Failed to add wormhole: %s", sqlite3_errmsg(db_));
    }
    
    return success;
}

std::vector<WormholeDatabase::Wormhole> WormholeDatabase::getWormholesFrom(const std::string& map_name) {
    std::vector<Wormhole> wormholes;
    
    const char* query = 
        "SELECT map_name, target_map, x, y, yaw "
        "FROM wormholes "
        "WHERE map_name = ?;";
        
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, query, -1, &stmt, nullptr) != SQLITE_OK) {
        ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db_));
        return wormholes;
    }
    
    sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_STATIC);
    
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        Wormhole wormhole;
        wormhole.map_name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        wormhole.target_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        wormhole.x = sqlite3_column_double(stmt, 2);
        wormhole.y = sqlite3_column_double(stmt, 3);
        wormhole.yaw = sqlite3_column_double(stmt, 4);
        
        wormholes.push_back(wormhole);
    }
    
    sqlite3_finalize(stmt);
    return wormholes;
}

WormholeDatabase::Wormhole WormholeDatabase::getWormholeBetween(const std::string& from_map, const std::string& to_map) {
    Wormhole wormhole;
    
    const char* query = 
        "SELECT map_name, target_map, x, y, yaw "
        "FROM wormholes "
        "WHERE map_name = ? AND target_map = ?;";
        
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, query, -1, &stmt, nullptr) != SQLITE_OK) {
        ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db_));
        return wormhole;
    }
    
    sqlite3_bind_text(stmt, 1, from_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, to_map.c_str(), -1, SQLITE_STATIC);
    
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        wormhole.map_name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        wormhole.target_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        wormhole.x = sqlite3_column_double(stmt, 2);
        wormhole.y = sqlite3_column_double(stmt, 3);
        wormhole.yaw = sqlite3_column_double(stmt, 4);
    }
    
    sqlite3_finalize(stmt);
    return wormhole;
}

} // namespace multi_map_navigation
