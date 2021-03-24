#include <include/astar.h>

#include <algorithm>
#include <unordered_map>
#include <math.h>

using geometry_msgs::Point;

geometry_msgs::Point point;

namespace std {
    template <>
    struct hash<Point> {
        size_t operator()(const Point& node) const {
            return std::hash<double>()(node.x) ^ std::hash<double>()(node.y);
        }
    };
}

inline bool collinear(Point pt1, Point pt2, Point pt3) {
    return (pt2.y - pt1.y)*(pt3.x - pt2.x) == (pt3.y - pt2.y)*(pt2.x - pt1.x);
}

Point get_point(double x, double y) {
    // ROS should really add constructors...
    Point p;
    p.x = x;
    p.y = y;
    return p;
}

Point pop_minimum_fscore(std::vector<Point> &points, std::unordered_map<Point, double> &fscores) {
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

bool is_frontier(std::vector<Point> &frontiers, Point point) {
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

std::vector<Point> reconstruct_path(Point current, std::unordered_map<Point, Point> &reverse_list) {
    std::vector<Point> waypoints;

    Point last_added = current;
    waypoints.push_back(current);
    current = reverse_list[current];

    while(reverse_list[current].x != INFINITY) {
        if(!collinear(last_added, current, reverse_list[current])) {
            waypoints.push_back(current);
            last_added = current;
        }
        current = reverse_list[current];
    }

    return waypoints;
}

std::vector<Point> AStar::FindPath(std::vector<Point> &frontiers, Point target, Point start) {
    Point dest = get_closest_point(frontiers, target);

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
            return reconstruct_path(current, came_from);
        }

        if(is_frontier(frontiers, current)) continue;

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
    return std::vector<Point>();
}

int main() {
    std::vector<Point> fntrs = {
        get_point(0, 10),
        get_point(0, 5),
        get_point(1, 5),
        get_point(-1, 5)
    };

    Point goal = get_point(0, 20);
    Point origin = get_point(0, 0);

    auto wps = AStar::FindPath(fntrs, goal, origin);
    for(auto wp : wps) {
        printf("Waypoint: (%f, %f)\n", wp.x, wp.y);
    }
}