#include <astar.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <algorithm>
#include <unordered_map>
#include <math.h>
#include <string>

using geometry_msgs::Point;
using geometry_msgs::PoseStamped;
using nav_msgs::Path;

geometry_msgs::Point point;

namespace std {
    template <>
    struct hash<Point> {
        size_t operator()(const Point& node) const {
            return std::hash<double>()(node.x) ^ std::hash<double>()(node.y);
        }
    };
}

Point get_point(double x, double y) {
    // ROS should really add constructors...
    Point p;
    p.x = x;
    p.y = y;
    return p;
}

Point pop_minimum_fscore(std::vector<Point> &points, std::unordered_map<Point, double> &fscores) {
    // Get the minimum fscore. This function should be removed and replaced by a priority queue (binsearch insert) eventually.

    Point min;
    double minFScore = INFINITY;
    int minIter = -1;

    for(int i = 0; i < points.size(); ++i) {
        if(fscores[points[i]] < minFScore) {
            minFScore = fscores[points[i]];
            min = points[i];
            minIter = i;
        }
    }

    points.erase(points.begin() + minIter);

    return min;
}

inline double distance(Point gridpoint, Point tgt) {
    // Return Euclidean Distance.
    return(sqrt((gridpoint.x - tgt.x)*(gridpoint.x - tgt.x) + (gridpoint.y - tgt.y)*(gridpoint.y - tgt.y)));
}

bool contains(std::vector<Point> &frontiers, Point point) {
    for(auto &pt : frontiers) {
        if(point == pt) {
            return true;
        }
    }
    return false;
}

std::vector<Point> get_neighbors(Point pt) {
    std::vector<Point> neighbors;

    neighbors.push_back(get_point(pt.x - 1, pt.y - 1));
    neighbors.push_back(get_point(pt.x - 1, pt.y    ));
    neighbors.push_back(get_point(pt.x - 1, pt.y + 1));
    neighbors.push_back(get_point(pt.x,     pt.y - 1));
    neighbors.push_back(get_point(pt.x,     pt.y + 1));
    neighbors.push_back(get_point(pt.x + 1, pt.y - 1));
    neighbors.push_back(get_point(pt.x + 1, pt.y    ));
    neighbors.push_back(get_point(pt.x + 1, pt.y + 1));

    return neighbors;
}

Point get_closest_point(std::vector<Point> &points, Point target) {
    Point closest;
    double min_dist = INFINITY;

    for(auto& pt : points) {
        if(distance(pt, target) < min_dist) {
            min_dist = distance(pt, target);
            closest = pt;
        }
    }
    return closest;
}

inline bool collinear(Point pt1, Point pt2, Point pt3) {
    // Checks if three points lie on the same line.
    return (pt2.y - pt1.y)*(pt3.x - pt2.x) == (pt3.y - pt2.y)*(pt2.x - pt1.x);
}

PoseStamped posestamped_from_point(Point pt, std::string frame_id) {
    PoseStamped ps;

    // The divide by 20 is there to adjust for rviz map sizes. should probably fix in rviz and remove before use.
    ps.pose.position.x = pt.x / 20;
    ps.pose.position.y = pt.y / 20;
    ps.header.frame_id = frame_id;
    return ps;
}

Path reconstruct_path(Point origin, Point current, std::unordered_map<Point, Point> &reverse_list, std::string frame_id) {
    // This function takes the list of nodes generated by A* and converts it into a list of waypoints.

    Path path;

    path.header.frame_id = frame_id;

    Point last_added = current;
    path.poses.push_back(posestamped_from_point(current, frame_id));
    current = reverse_list[current];

    while(reverse_list[current].x != INFINITY) {
        if(!collinear(last_added, current, reverse_list[current])) {
            path.poses.push_back(posestamped_from_point(current, frame_id));
            last_added = current;
        }
        current = reverse_list[current];
    }

    path.poses.push_back(posestamped_from_point(origin, frame_id));

    return path;
}

Path AStar::FindPathFrontier(std::vector<Point> &frontiers, std::vector<Point> &obstacles, Point target, Point start, bool DirectToDest = false) {
    // A Star Implementation based off https://en.wikipedia.org/wiki/A*_search_algorithm

    Point dest = DirectToDest ? target : get_closest_point(frontiers, target);

    Point origin = start;

    std::unordered_map<Point, double> gScores;
    std::unordered_map<Point, double> fScores;

    std::vector<Point> open_set; // TODO: Optimize by replacing with custom priority queue.
    open_set.push_back(origin);

    std::unordered_map<Point, Point> came_from;
    came_from[origin] = get_point(INFINITY, INFINITY);

    
    gScores[origin] = 0;
    fScores[origin] = distance(origin, dest);

    while(!open_set.empty()) {
        auto current = pop_minimum_fscore(open_set, fScores); // TODO: prio queue.

        if(current == dest) {
            return reconstruct_path(origin, current, came_from, "frameidplaceholder");
        }

        if(contains(obstacles, current) || contains(frontiers, current)) continue;

        for(auto &neighbor : get_neighbors(current)) {

            auto tentative_gscore = gScores[current] + distance(current, neighbor);
            if(gScores.find(neighbor) == gScores.end()) gScores[neighbor] = INFINITY;
            if(tentative_gscore < gScores[neighbor]) {
                gScores[neighbor] = tentative_gscore;
                came_from[neighbor] = current;
                fScores[neighbor] = gScores[neighbor] + distance(neighbor, dest);
                if(std::find(open_set.begin(), open_set.end(), neighbor) == open_set.end()) { // TODO: prio queue.
                    open_set.push_back(neighbor);
                }
            }
        }
    }

    printf("[WARNING] Call to navigation failed to find valid path.\n");
    return Path();
}

inline int access_oGrid(int x, int y, nav_msgs::OccupancyGrid oGrid) {
    if((y * oGrid.info.width + x) > oGrid.data.size()) return 100;
    if((y * oGrid.info.width + x) < 0) return 100;
    return oGrid.data[y * oGrid.info.width + x];
}

Path AStar::FindPathOccGrid(nav_msgs::OccupancyGrid oGrid, Point target, Point start) {
    // A Star Implementation based off https://en.wikipedia.org/wiki/A*_search_algorithm
    std::unordered_map<Point, double> gScores;
    std::unordered_map<Point, double> fScores;

    Point origin = start;

    std::vector<Point> open_set; // TODO: Optimize by replacing with custom priority queue.
    open_set.push_back(start);

    std::unordered_map<Point, Point> came_from;
    came_from[start] = get_point(INFINITY, INFINITY);

    gScores[start] = 0;
    fScores[start] = distance(start, target);

    while(!open_set.empty()) {
        auto current = pop_minimum_fscore(open_set, fScores); // TODO: prio queue.

        if(current == target) {
            return reconstruct_path(origin, current, came_from, oGrid.header.frame_id.c_str());
        }

        if(access_oGrid(current.x, current.y, oGrid) >= 50) continue;

        for(auto &neighbor : get_neighbors(current)) {

            auto tentative_gscore = gScores[current] + distance(current, neighbor);
            if(gScores.find(neighbor) == gScores.end()) gScores[neighbor] = INFINITY;
            if(tentative_gscore < gScores[neighbor]) {
                gScores[neighbor] = tentative_gscore;
                came_from[neighbor] = current;
                fScores[neighbor] = gScores[neighbor] + distance(neighbor, target);
                if(std::find(open_set.begin(), open_set.end(), neighbor) == open_set.end()) { // TODO: prio queue.
                    open_set.push_back(neighbor);
                }
            }
        }
    }

    printf("[WARNING] Call to navigation failed to find valid path.\n");
    return Path();
}