#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <yaml-cpp/yaml.h> // Include yaml-cpp header
#include <fstream>
#include <vector>
#include <string>

nav_msgs::OccupancyGrid loadMapFromFile(const std::string& filename) {
    nav_msgs::OccupancyGrid map;

    try {
        // Load the YAML file
        YAML::Node yaml_map = YAML::LoadFile(filename);

        // Extract map parameters from YAML
        int width = yaml_map["width"].as<int>();
        int height = yaml_map["height"].as<int>();
        double resolution = yaml_map["resolution"].as<double>();
        double origin_x = yaml_map["origin"][0].as<double>();
        double origin_y = yaml_map["origin"][1].as<double>();

        // Fill the map metadata
        map.info.width = width;
        map.info.height = height;
        map.info.resolution = resolution;
        map.info.origin.position.x = origin_x;
        map.info.origin.position.y = origin_y;

        // Fill the map data
        const YAML::Node& data_node = yaml_map["data"];
        map.data.resize(width * height);
        for (size_t i = 0; i < data_node.size(); ++i) {
            map.data[i] = data_node[i].as<int8_t>();
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR_STREAM("Error loading map from YAML file: " << e.what());
    }

    return map;
}

bool handleMapRequest(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res) {
    // Load map from the specified YAML file
    std::string mapFilePath = "/home/rohanshenoy30/Downloads/map.yaml"; // Specified file path
    nav_msgs::OccupancyGrid map = loadMapFromFile(mapFilePath);

    // Fill the response with the loaded map
    res.map = map;

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh;

    // Advertise the GetMap service
    ros::ServiceServer service = nh.advertiseService("/static_map", handleMapRequest);
    ROS_INFO("Map server ready to provide static map.");

    ros::spin();

    return 0;
}