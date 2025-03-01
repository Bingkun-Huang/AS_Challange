#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <octomap/OcTree.h>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <mutex>

ros::Publisher path_pub, vis_pub;
std::mutex map_mutex;
octomap::OcTree* octree = nullptr;
double resolution = 0.1;  // 真实地图分辨率

struct Node {
    int x, y, z;
    double g, h;
    Node* parent;
    Node(int x, int y, int z, Node* parent = nullptr) 
        : x(x), y(y), z(z), g(0), h(0), parent(parent) {}
    double cost() const { return g + h; }
    bool isGoal(const Node* goal) const { return x == goal->x && y == goal->y && z == goal->z; }
};

struct CompareNode {
    bool operator()(const Node* a, const Node* b) { return a->cost() > b->cost(); }
};

std::string hashKey(int x, int y, int z) {
    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(z);
}

bool isCollision(int x, int y, int z) {
    octomap::point3d point(x * resolution, y * resolution, z * resolution);
    octomap::OcTreeNode* node = octree->search(point);
    return node && node->getOccupancy() > 0.5;
}

void findNearestFree(int& x, int& y, int& z) {
    for (int r = 0; r < 5; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                for (int dz = -r; dz <= r; ++dz) {
                    if (!isCollision(x + dx, y + dy, z + dz)) {
                        x += dx;
                        y += dy;
                        z += dz;
                        return;
                    }
                }
            }
        }
    }
}

std::vector<std::vector<int>> shifts = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
    {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},
    {1, 0, 1}, {-1, 0, 1}, {0, 1, 1}, {0, -1, 1},
    {1, 1, 1}, {1, -1, 1}, {-1, 1, 1}, {-1, -1, 1}
};

std::vector<geometry_msgs::PoseStamped> reconstructPath(Node* node) {
    std::vector<geometry_msgs::PoseStamped> path;
    while (node) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = node->x * resolution;
        pose.pose.position.y = node->y * resolution;
        pose.pose.position.z = node->z * resolution;
        path.push_back(pose);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<geometry_msgs::PoseStamped> aStarSearch(int sx, int sy, int sz, int gx, int gy, int gz) {
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openList;
    std::unordered_map<std::string, Node*> allNodes;

    Node* startNode = new Node(sx, sy, sz);
    Node* goalNode = new Node(gx, gy, gz);
    openList.push(startNode);
    allNodes[hashKey(sx, sy, sz)] = startNode;

    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();

        if (current->isGoal(goalNode)) return reconstructPath(current);

        for (auto& shift : shifts) {
            int nx = current->x + shift[0];
            int ny = current->y + shift[1];
            int nz = current->z + shift[2];

            if (isCollision(nx, ny, nz)) continue;

            Node* neighbor = new Node(nx, ny, nz, current);
            neighbor->g = current->g + 1;
            neighbor->h = std::sqrt(pow(nx - gx, 2) + pow(ny - gy, 2) + pow(nz - gz, 2));

            std::string key = hashKey(nx, ny, nz);
            if (allNodes.find(key) == allNodes.end() || neighbor->g < allNodes[key]->g) {
                allNodes[key] = neighbor;
                openList.push(neighbor);
            }
        }
    }
    return {};
}

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(map_mutex);
    delete octree;
    octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));
    resolution = octree->getResolution();
    ROS_INFO("OctoMap received, resolution: %.2f", resolution);
}

void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    int gx = std::round(msg->point.x / resolution);
    int gy = std::round(msg->point.y / resolution);
    int gz = std::round(msg->point.z / resolution);
    findNearestFree(gx, gy, gz);

    int sx, sy, sz;
    {
        std::lock_guard<std::mutex> lock(map_mutex);
        octomap::point3d start = octree->keyToCoord(octree->coordToKey(msg->point.x, msg->point.y, msg->point.z));
        sx = std::round(start.x() / resolution);
        sy = std::round(start.y() / resolution);
        sz = std::round(start.z() / resolution);
        findNearestFree(sx, sy, sz);
    }

    auto path = aStarSearch(sx, sy, sz, gx, gy, gz);
    if (path.empty()) {
        ROS_WARN("A* failed to find a path!");
        return;
    }

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.poses = path;
    path_pub.publish(path_msg);
    vis_pub.publish(path_msg);
    ROS_INFO("Path published with %ld points.", path.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "a_star_planner");
    ros::NodeHandle nh;

    path_pub = nh.advertise<nav_msgs::Path>("waypoints", 1, true);
    vis_pub = nh.advertise<nav_msgs::Path>("visualization_marker", 1, true);

    ros::Subscriber map_sub = nh.subscribe("/octomap_full", 1, octomapCallback);
    ros::Subscriber goal_sub = nh.subscribe("/point/correction", 1, goalCallback);

    ros::spin();
}
