#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher map_metadata_pub;
ros::Publisher map_pub;
ros::Subscriber map_sub;

struct Cell {
    int x;
    int y;
    double f;
    double g;
    double h;
    std::unique_ptr<Cell> parent; // Using smart pointer for automatic memory management

    // Define constructor for Cell
    Cell(int x, int y, double f, double g, double h, std::unique_ptr<Cell> parent) : 
        x(x), y(y), f(f), g(g), h(h), parent(std::move(parent)) {}

    // Define copy constructor
    Cell(const Cell& other) : 
    x(other.x), y(other.y), f(other.f), g(other.g), h(other.h) {
    if (other.parent) {
        parent = std::unique_ptr<Cell>(new Cell(*other.parent)); // Make a deep copy of the parent
    }
}
};

struct CompareCells {
    bool operator()(const Cell* a, const Cell* b) const {
        return a->f > b->f;
    }
};

nav_msgs::Path aStar(const nav_msgs::OccupancyGrid& map, Cell start, Cell goal) {
    std::priority_queue<Cell*, std::vector<Cell*>, CompareCells> openList;
    std::vector<std::vector<bool>> closedList(map.info.width, std::vector<bool>(map.info.height, false));

    openList.push(new Cell(start));

    while (!openList.empty()) {
        Cell* current = openList.top();
        openList.pop();

        if (current->x == goal.x && current->y == goal.y) {
            nav_msgs::Path path_msg;
            while (current != nullptr) {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = current->x * map.info.resolution;
                pose.pose.position.y = current->y * map.info.resolution;
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
                current = current->parent.get(); // Using get() to access the raw pointer
            }
            return path_msg;
        }

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                int nx = current->x + dx;
                int ny = current->y + dy;

                if (nx < 0 || nx >= map.info.width || ny < 0 || ny >= map.info.height) continue;

                if (map.data[ny * map.info.width + nx] != 0) continue;

                if (closedList[nx][ny]) continue;

                double tentative_g = current->g + std::sqrt(dx*dx + dy*dy);

                Cell* neighbor = nullptr;
                bool found = false;
                std::priority_queue<Cell*, std::vector<Cell*>, CompareCells> temp;
                while (!openList.empty()) {
                    Cell* cell = openList.top();
                    openList.pop();
                    if (cell->x == nx && cell->y == ny) {
                        neighbor = cell;
                        found = true;
                        break;
                    }
                    temp.push(cell);
                }
                openList = temp;

                if (!found || tentative_g < neighbor->g) {
                    if (neighbor == nullptr) {
                        neighbor = new Cell(nx, ny, 0, 0, 0, nullptr);
                    }
                    neighbor->parent = std::unique_ptr<Cell>(new Cell(nx, ny, 0, 0, 0, nullptr));
                    neighbor->g = tentative_g;
                    neighbor->h = std::sqrt((nx - goal.x)*(nx - goal.x) + (ny - goal.y)*(ny - goal.y));
                    neighbor->f = neighbor->g + neighbor->h;

                    openList.push(neighbor);
                }
            }
        }
        closedList[current->x][current->y] = true;
    }
    return nav_msgs::Path(); // No path found
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)  {
    // Perform A* pathfinding
    Cell start(5, 5, 0, 0, 0, nullptr); // Example start cell
    Cell goal(45, 45, 0, 0, 0, nullptr); // Example goal cell
    nav_msgs::Path path_msg = aStar(*map_msg, start, goal);

    // Publish the path back to the publisher via the "map" topic
    map_pub.publish(path_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_publisher_subscriber");
    ros::NodeHandle nh;

    // Create a service client for the GetMap service
    ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("/static_map");

    // Create a GetMap service request
    nav_msgs::GetMap srv;

    // Call the GetMap service
    if (map_client.call(srv)) {
        // Process the received map
        nav_msgs::OccupancyGrid map = srv.response.map;

        // Publisher for map metadata
        map_metadata_pub = nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

        // Publisher for path
        map_pub = nh.advertise<nav_msgs::Path>("path", 1, true); // Changed topic name to "path"

        // Subscriber for receiving map
        map_sub = nh.subscribe("map", 1, mapCallback); // Modified topic name and callback function

        // Populate map metadata
        nav_msgs::MapMetaData map_metadata;
        map_metadata.resolution = map.info.resolution;
        map_metadata.origin = map.info.origin;

        // Publish map metadata
        map_metadata_pub.publish(map_metadata);
    } else {
        ROS_ERROR("Failed to call GetMap service");
        return 1;
    }

    ros::spin();

    return 0;
}