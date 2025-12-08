#pragma once
#include "config.h"
#include <vector>

// The full definition of Node is required by planner.cpp
struct Node {
    Point pos;
    double g_cost;        // 累积代价
    double h_cost;        // 启发式代价
    double f_cost;        // 总代价 f = g + h
    double incoming_az;   // 到达该节点的航向
    Node* parent;
    int nfz_idx;          // 从哪个禁飞区扩展而来
};

class AStarPlanner {
public:
    // The public interface remains compatible with the call in main.cpp
    static std::vector<Point> plan(const State& start_state, const Point& target, const std::vector<NoFlyZone>& nfzs);
    static double calcDist(const Point& p1, const Point& p2);

private:
    // Internal plan implementation using Point
    static std::vector<Point> plan(const Point& start, const Point& target, const std::vector<NoFlyZone>& nfzs);

    // Internal helpers for the new algorithm
    static std::vector<Point> getSphericalTangents(const Point& p, const NoFlyZone& zone);
    static bool isBlockedGlobal(const Point& p1, const Point& p2, const std::vector<NoFlyZone>& nfzs, int ignore_idx);
    static bool isTooCloseToNFZ(const Point& p1, const Point& p2, const std::vector<NoFlyZone>& nfzs, int ignore_idx, double safety_factor = 1.2);
};