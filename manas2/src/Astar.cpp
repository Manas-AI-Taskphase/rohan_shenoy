#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <queue>
#include <limits>
#include <visualization_msgs/Marker.h>

// Define your Node structure for A* algorithm
struct Node {
    int i, j;
    float f, g, h;
    Node* parent;

    Node(int _i, int _j) : i(_i), j(_j), f(0), g(std::numeric_limits<float>::infinity()), h(0), parent(nullptr) {}

    float dist(const Node& other) const {
        return std::sqrt(std::pow(i - other.i, 2) + std::pow(j - other.j, 2));
    }
};

// Structure to compare nodes
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        return a->f > b->f;
    }
};

// A* algorithm implementation
class AStarPathFinder {
public:
    AStarPathFinder(const std::vector<std::vector<int>>& _map, Node* _start, Node* _end, bool _allowDiagonals)
        : map(_map), start(_start), end(_end), allowDiagonals(_allowDiagonals) {}

    std::vector<Node*> findPath() {
        std::vector<Node*> openSet;
        openSet.push_back(start);

        while (!openSet.empty()) {
            Node* current = openSet.front();
            openSet.erase(openSet.begin());

            if (current == end) {
                return reconstructPath(current);
            }

            for (Node* neighbor : getNeighbors(current)) {
                float tentativeG = current->g + current->dist(*neighbor);
                if (tentativeG < neighbor->g && !contains(openSet, neighbor)) {
                    neighbor->parent = current;
                    neighbor->g = tentativeG;
                    neighbor->h = neighbor->dist(*end);
                    neighbor->f = neighbor->g + neighbor->h;
                    openSet.push_back(neighbor);
                }
            }
        }

        return {}; // No path found
    }

private:
    const std::vector<std::vector<int>>& map;
    Node* start;
    Node* end;
    bool allowDiagonals;

    // Function to check if a node exists in a vector
    bool contains(const std::vector<Node*>& nodes, Node* node) const {
        for (Node* n : nodes) {
            if (n->i == node->i && n->j == node->j) {
                return true;
            }
        }
        return false;
    }

    // Function to reconstruct path from the goal node to the start node
    std::vector<Node*> reconstructPath(Node* current) {
        std::vector<Node*> path;
        while (current != nullptr) {
            path.push_back(current);
            current = current->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // Function to get neighboring nodes
    std::vector<Node*> getNeighbors(Node* node) const {
        std::vector<Node*> neighbors;
        int di[8] = {1, 1, 0, -1, -1, -1, 0, 1};
        int dj[8] = {0, 1, 1, 1, 0, -1, -1, -1};
        int numDirs = allowDiagonals ? 8 : 4;

        for (int d = 0; d < numDirs; ++d) {
            int ni = node->i + di[d];
            int nj = node->j + dj[d];
            if (ni >= 0 && ni < map.size() && nj >= 0 && nj < map[0].size() && map[ni][nj] == 0) {
                neighbors.push_back(new Node(ni, nj));
            }
        }

        return neighbors;
    }
};

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& data) {
    int width = data->info.width;
    int height = data->info.height;
    double resolution = data->info.resolution;
    std::vector<std::vector<int>> map(height, std::vector<int>(width));

    // Populate map from the received data
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            map[i][j] = data->data[i * width + j];
        }
    }

    Node* start = new Node(0, 0); // Example start node
    Node* goal = new Node(10, 10);   // Example goal node

    // Check map resolution
    double map_resolution = data->info.resolution;
    if (fabs(map_resolution - resolution) > 0.0001) {
        ROS_ERROR("Map resolution mismatch: Expected %f, Received %f", resolution, map_resolution);
        return;
    }

    // Check start and goal positions
    if (start->i < 0 || start->i >= height || start->j < 0 || start->j >= width || map[start->i][start->j] != 0) {
        ROS_ERROR("Invalid start position or position inside an obstacle.");
        delete start;
        delete goal;
        return;
    }

    if (goal->i < 0 || goal->i >= height || goal->j < 0 || goal->j >= width || map[goal->i][goal->j] != 0) {
        ROS_ERROR("Invalid goal position or position inside an obstacle.");
        delete start;
        delete goal;
        return;
    }

    AStarPathFinder pathFinder(map, start, goal, true);
    std::vector<Node*> path = pathFinder.findPath();

    // Correction: Publish path only if found
    if (path.empty()) {
        ROS_INFO("No path found. Not publishing.");
        delete start;
        delete goal;
        return;
    }

    // Publish path visualization markers
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";

    for (Node* node : path) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = node->i * resolution;
        pose.pose.position.y = node->j * resolution;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    path_pub.publish(path_msg);

    delete start;
    delete goal;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner");  // Initialize ROS node
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("map", 1, mapCallback);
    ros::spin();
    return 0;
}