#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <memory>

// using common std commands
using std::cout;
using std::endl;

//defining a struct node to represent node in algorithms
struct Node 
{
    int x, y;//coordinates of node in map
    double cost;// g cost -dist from starting node to current node
    double heuristic;// h cost- dist from end node
    Node* parent;//pointer to point to the parent node of the current node


    //constructor initialises node object sets values of the variables acc to the provided arguments
    Node(int x_, int y_, double cost_, double heuristic_, Node* parent_) :
            x(x_), y(y_), cost(cost_), heuristic(heuristic_), parent(parent_) {}

    
    // method to calculate f cost= sum of g cost and h cost
    double totalCost() const 
    {
        return cost + heuristic;
    }
};

// structure to compare nodes based on f cost
//used to order nodes in priority queue based on total cost(f)
struct CompareNodes 
{
    bool operator()(const Node* lhs, const Node* rhs) const 
    {
        return lhs->totalCost() > rhs->totalCost();//true if lhs>rhs else false
    }
};

// Function to calculate Euclidean distance b/w 2 pts
double calculateHeuristic(int x1, int y1, int x2, int y2) 
{
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//astar algorithm
std::vector<Node*> astar(Node* start, Node* goal, const std::vector<std::vector<int>>& map, int width, int height) 
{
    //priority queue to store nodes to be evaluated, prioritised by comparenodes, stored in openset
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openSet;

    //2d boolean vector to mark the nodes which have been visited
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    openSet.push(start);

    //algorithm iterates while openset isnt empty
    while (!openSet.empty()) 
    {
        //fetch node with lowest cost from priority queue
        Node* current = openSet.top();
        openSet.pop();


        //if the goal is found we return the reconstructed path by following parent pointers from goal to start
        if (current->x == goal->x && current->y == goal->y) //is current node goal node
        {
            //reconstruct the path
            std::vector<Node*> path;
            while (current != nullptr) 
            {
                path.push_back(current);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;//return the path
        }

        visited[current->y][current->x] = true; // mark the node as visited

        // Generate neighbor nodes by exploring adjacent cells on the grid
        for (int dx = -1; dx <= 1; ++dx) 
        {
            for (int dy = -1; dy <= 1; ++dy) 
            {
                if (dx == 0 && dy == 0) continue; // Skip current node
                int nx = current->x + dx;
                int ny = current->y + dy;

                //calculate the new costs for each neighbour node
                if (nx >= 0 && nx < width && ny >= 0 && ny < height && map[ny][nx] == 0 && !visited[ny][nx]) 
                {
                    double newCost = current->cost + 1; // Assuming uniform cost for all movements

                    //create new node object for the neighbour w its updated costs and parent
                    Node* neighbor = new Node(nx, ny, newCost, calculateHeuristic(nx, ny, goal->x, goal->y), current);
                    openSet.push(neighbor);//push neighbour node into priority queue openset
                }
            }
        }
    }

    return {}; //if loop terminates without finding goal node, No path found
}

//fn that gets executed whenever new occupancy grid map message is rcvd
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& data) 
{
    //extracting info from occupancy grid map
    int width = data->info.width;
    int height = data->info.height;
    double resolution = data->info.resolution;
    cout << "width: " << width << endl;
    cout << "height: " << height << endl;

    //populating the map

    //creating 2d vector 'map' of size height*width to store occ grid map data
    std::vector<std::vector<int>> map(height, std::vector<int>(width));

    //iterates over each map message and assigns corresponding values to each cell
    for (int i = 0; i < height; i++) 
    {
        for (int j = 0; j < width; j++) 
        {
            int index = i * width + j;
            map[i][j] = data->data[index];
        }
    }

    Node* start = new Node(10, 10, 0, 0, nullptr); // Assuming start point coordinates
    Node* goal = new Node(50, 50, 0, 0, nullptr); // Assuming goal point coordinates

    //calling astar fn to generate the path
    std::vector<Node*> path = astar(start, goal, map, width, height);
    
    //creating nodehandle and advertising the topic to publish the path to
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1); // Advertise "path" topic
    nav_msgs::Path path_msg;//create msg named path_msg
    path_msg.header.frame_id = "map";//setting its frame id to map

    //converting the path(list of nodes) obtained from astar to pose messages
    for (auto it = path.rbegin(); it != path.rend(); ++it) 
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = (*it)->x * resolution;
        pose.pose.position.y = (*it)->y * resolution;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose); //adding and storing them in path_msg.poses
    }

    path_pub.publish(path_msg); //publishing the path to path_msg

    // Clean up the node obj to avoid memory leaks
    for (Node* node : path) {
        delete node;
    }
}

//call back fn that gets executed when new path message is received by the subscriber node
void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) 
{
    cout << "Received path message!" << endl;
    // Process the path data here

        // Prepare the path for RViz
    std::vector<geometry_msgs::PoseStamped> path_poses = path_msg->poses;

    // Publish the path for RViz
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("display_path", 1);

    // Prepare the path message
    nav_msgs::Path display_path_msg;
    display_path_msg.header = path_msg->header;
    display_path_msg.poses = path_poses;

    // Publish the path
    path_pub.publish(display_path_msg);
}



int main(int argc, char ** argv) 
{
    // initialise ros
    ros::init(argc, argv, "AStar");//initialising ros system
    ros::NodeHandle n;
    ros::Subscriber map_sub = n.subscribe("map", 1, mapCallback); // subscribing to the map topic
    ros::Subscriber path_sub = n.subscribe("path", 1, pathCallback); // subscribing to the path topic
    ros::spin();//continues to loop until node is shut down
    return 0;
}
