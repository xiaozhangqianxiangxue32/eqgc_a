#include "planner.h"
#include <iostream>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159
#endif
#include <set>
#ifndef PI
#define PI 3.14159 // 修复：原来的 3.14/4 是错误的
#endif
using namespace Config;

// ==========================================
// 1. 球面数学工具库实现 (Utils)
// ==========================================
namespace Utils {

    // 角度归一化 [-PI, PI]
    double normalizeAngle(double angle) {
        while (angle > PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
        return angle;
    }

    // 球面大圆距离 (Haversine) - 返回弧度
    double calcDistRad(const Point& p1, const Point& p2) {
        double dlat = p2.lat - p1.lat;
        double dlon = p2.lon - p1.lon;
        double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
            std::cos(p1.lat) * std::cos(p2.lat) *
            std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
        return 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    }

    // 计算初始方位角 (Azimuth) - 论文 Eq(10) 等价形式
    double calcAzimuth(const Point& start, const Point& end) {
        double y = std::sin(end.lon - start.lon) * std::cos(end.lat);
        double x = std::cos(start.lat) * std::sin(end.lat) - 
            std::sin(start.lat) * std::cos(end.lat) * std::cos(end.lon - start.lon);
        return std::atan2(y, x);
    }

    // 正算: 已知起点、方位角、距离(弧度)，求终点
    Point calcDirect(const Point& start, double az, double dist) {
        double lat2 = std::asin(std::sin(start.lat) * std::cos(dist) +
            std::cos(start.lat) * std::sin(dist) * std::cos(az));
        double lon2 = start.lon + std::atan2(std::sin(az) * std::sin(dist) * std::cos(start.lat),
            std::cos(dist) - std::sin(start.lat) * std::sin(lat2));
        return { lon2, lat2 };
    }

    // 碰撞检测: 判断大圆航线是否被禁飞区阻挡
    bool isPathBlocked(const Point& p1, const Point& p2, const NoFlyZone& nfz) {
        Point c = { nfz.lon, nfz.lat };
        double r_safe = nfz.getSafeRadius(); // 论文 Eq(9)

        // 1. 端点检查
        if (calcDistRad(p1, c) < r_safe || calcDistRad(p2, c) < r_safe) return true;

        // 2. Cross-Track Distance (CTD) 检查
        double az_12 = calcAzimuth(p1, p2);
        double az_1c = calcAzimuth(p1, c);
        double dist_1c = calcDistRad(p1, c);

        // sin(CTD) = sin(dist_1c) * sin(az_12 - az_1c)
        double ctd_sin = std::sin(dist_1c) * std::sin(normalizeAngle(az_12 - az_1c));
        if (ctd_sin > 1.0) ctd_sin = 1.0;
        if (ctd_sin < -1.0) ctd_sin = -1.0;
        double ctd = std::asin(ctd_sin);

        if (std::abs(ctd) > r_safe) return false; // 垂距足够大，安全

        // 3. Along-Track Distance (ATD) 检查 - 判断垂足是否在线段内
        // cos(dist_1c) = cos(CTD) * cos(ATD) => cos(ATD) = cos(dist_1c)/cos(CTD)
        double cos_atd = std::cos(dist_1c) / std::cos(ctd);
        if (cos_atd > 1.0) cos_atd = 1.0;
        if (cos_atd < -1.0) cos_atd = -1.0;
        double atd = std::acos(cos_atd);
        double total_dist = calcDistRad(p1, p2);

        // 如果圆心在起点后方 (|angle| > 90)，atd 取负
        if (std::abs(normalizeAngle(az_12 - az_1c)) > PI / 2.0) {
            atd = -atd;
        }

        // 如果 0 < ATD < TotalDist，说明垂足在线段内部，且 CTD < Radius -> 阻挡
        if (atd > 0 && atd < total_dist) return true;

        return false;
    }
}

// Add the public static function back to the class
double AStarPlanner::calcDist(const Point& p1, const Point& p2) {
    return Utils::calcDistRad(p1, p2);
}

// ==========================================
// 2. A* 规划器实现
// ==========================================

// 优先队列包装器
struct NodeWrapper {
    Node* node;
    bool operator<(const NodeWrapper& other) const {
        return node->f_cost > other.node->f_cost; // 最小堆
    }
};

// Public interface that takes a State object
std::vector<Point> AStarPlanner::plan(const State& start_state, const Point& target, const std::vector<NoFlyZone>& nfzs) {
    Point start_point = { start_state.lon, start_state.lat };
    return plan(start_point, target, nfzs);
}

// Private implementation that takes a Point object
std::vector<Point> AStarPlanner::plan(const Point& start, const Point& target, const std::vector<NoFlyZone>& nfzs) {
    std::set<std::pair<double, double>> closed_set;
    // 内存池管理
    std::vector<Node*> node_pool;
    std::priority_queue<NodeWrapper> open_list;

    // 初始化起点
    // 起始航向设为直连目标的航向 (Paper隐含)
    double init_az = Utils::calcAzimuth(start, target);
    // 启发式代价为到终点的直线距离
    double init_h = Utils::calcDistRad(start, target);

    Node* start_node = new Node{ start, 0.0, init_h, init_h, init_az, nullptr, -1 };
    node_pool.push_back(start_node);
    open_list.push({ start_node });

    Node* best_node = nullptr;
    int iterations = 0;
    const int MAX_ITER = 50000;

    std::cout << "[Planner] Start Spherical A* Search (Minimum Turn Angle Mode)..." << std::endl;
    std::cout << "[Planner] Start: (" << start.lon * Config::RAD2DEG << "°, " 
   << start.lat * Config::RAD2DEG << "°)" << std::endl;
    std::cout << "[Planner] Target: (" << target.lon * Config::RAD2DEG << "°, " 
 << target.lat * Config::RAD2DEG << "°)" << std::endl;
    std::cout << "[Planner] Number of NFZs: " << nfzs.size() << std::endl;

    while (!open_list.empty()) {
        if (++iterations > MAX_ITER) {
            std::cerr << "[Planner] Timeout!" << std::endl;
            break;
        }

        Node* current = open_list.top().node;
        open_list.pop();

        // 调试：输出当前节点信息
   if (iterations <= 10 || iterations % 100 == 0) {
        std::cout << "[Planner] Iter " << iterations 
  << ": Node at (" << current->pos.lon * Config::RAD2DEG 
           << "°, " << current->pos.lat * Config::RAD2DEG 
         << "°), g=" << current->g_cost * Re / 1000.0 
      << "km, h=" << current->h_cost * Re / 1000.0 
      << "km, f=" << current->f_cost * Re / 1000.0 << "km" << std::endl;
        }

        // 计算到终点的距离（只计算一次，后续复用）
        double dist_to_target = Utils::calcDistRad(current->pos, target);

        // *** 优先检查：是否已经到达终点 ***
        if (dist_to_target < 1e-6) {  // 已经在终点（距离几乎为0）
      best_node = current;
            std::cout << "[Planner] Target reached! Total cost: " 
         << current->g_cost * Re / 1000.0 << " km, Iterations: " << iterations << std::endl;
    break;
     }

   // 检查：能否直连终点
     if (!isBlockedGlobal(current->pos, target, nfzs, current->nfz_idx)) {
 // 可以直连！计算代价
    double az_to_target = Utils::calcAzimuth(current->pos, target);
            double turn_angle = std::abs(Utils::normalizeAngle(az_to_target - current->incoming_az));
         double turn_cost = turn_angle;
      double dist_cost = dist_to_target * 0.1;
 double final_g = current->g_cost + turn_cost + dist_cost;

        // 生成目标节点
   best_node = new Node{ target, final_g, 0.0, final_g, az_to_target, current, -2 };
   node_pool.push_back(best_node);

       std::cout << "[Planner] Direct path to target found! Turn: " 
    << (turn_angle * Config::RAD2DEG) << "°, Dist: " 
    << (dist_to_target * Re / 1000.0) << " km, Total cost: " << final_g << std::endl;
       break; 
 }

        // 如果不能直连，检查是否已经非常接近终点（备用退出条件）
if (dist_to_target * Re < 500.0) { // 500m范围内（从5km修改为500m）
            // 强制连接到终点
         double az_final = Utils::calcAzimuth(current->pos, target);
    double turn_angle = std::abs(Utils::normalizeAngle(az_final - current->incoming_az));
    double turn_cost = turn_angle;
            double dist_cost = dist_to_target * 0.1;
            double final_g = current->g_cost + turn_cost + dist_cost;
      
         best_node = new Node{ target, final_g, 0.0, final_g, az_final, current, -2 };
     node_pool.push_back(best_node);
            
     std::cout << "[Planner] Reached target vicinity (<500m), forcing connection. "
      << "Turn: " << (turn_angle * Config::RAD2DEG) << "°, "
         << "Dist: " << (dist_to_target * Re) << " m, "
          << "Cost: " << final_g << std::endl;
   break;
        }

        // 2. 扩展切点 (Tangent Expansion)
        for (int i = 0; i < (int)nfzs.size(); ++i) {

            if (i == current->nfz_idx) continue; // 防止回跳

            // 获取球面切点
            std::vector<Point> tangents = getSphericalTangents(current->pos, nfzs[i]);

            for (const auto& next_pos : tangents) {
                // 全局阻挡检测
                if (isBlockedGlobal(current->pos, next_pos, nfzs, current->nfz_idx)) {
                    continue;
                }
                // 在创建 next_node 前添加
                std::pair<double, double> pos_key = { std::round(next_pos.lon * 1e6), std::round(next_pos.lat * 1e6) };
                if (closed_set.find(pos_key) != closed_set.end()) continue;
                // 创建 next_node 后添加
                closed_set.insert(pos_key);
                // *** 关键优化: 在生成切点后，立即检查该切点能否直连终点 ***
   // 不仅要检查是否被阻挡，还要检查是否距离其他禁飞区太近
    if (!isBlockedGlobal(next_pos, target, nfzs, i) && 
     !isTooCloseToNFZ(next_pos, target, nfzs, i, 1.5)) { // 1.5倍安全裕度
     // 这个切点可以安全地直连终点！
           double dist_to_tangent = Utils::calcDistRad(current->pos, next_pos);
              double dist_tangent_to_target = Utils::calcDistRad(next_pos, target);
        
   // 计算转向角代价
  double az_to_tangent = Utils::calcAzimuth(current->pos, next_pos);
        double az_tangent_to_target = Utils::calcAzimuth(next_pos, target);
  double turn1 = std::abs(Utils::normalizeAngle(az_to_tangent - current->incoming_az));
     double turn2 = std::abs(Utils::normalizeAngle(az_tangent_to_target - az_to_tangent));
      
  // 总代价 = 累积转向角 + 距离代价
double turn_cost_total = turn1 + turn2;
      double dist_cost_total = (dist_to_tangent + dist_tangent_to_target) * 0.1;
        double final_g = current->g_cost + turn_cost_total + dist_cost_total;

        // 创建一个临时切点节点
    Node* tangent_node = new Node{
next_pos,
     current->g_cost + turn1 + dist_to_tangent * 0.1,
       dist_tangent_to_target,
     final_g,  // f_cost
    az_to_tangent,
       current,
   i
    };
  node_pool.push_back(tangent_node);

 // 直接创建终点节点
  Node* final_node = new Node{
    target,
             final_g,
      0.0,
        final_g,
  az_tangent_to_target,
     tangent_node,
 -2
        };
     node_pool.push_back(final_node);

            // 加入优先队列（可能会找到更优路径）
        open_list.push({ final_node });

      std::cout << "[Planner] Tangent point can SAFELY see target! Turn cost: " 
         << (turn_cost_total * Config::RAD2DEG) << "°, Dist: " 
    << (dist_to_tangent + dist_tangent_to_target) * Re / 1000.0 << " km" << std::endl;
   
         // 不再将切点作为中间节点加入（因为已经连接到终点了）
      continue;
         }

                // 如果切点不能直连终点，则正常扩展
                double dist_segment = Utils::calcDistRad(current->pos, next_pos);
                
             // *** 修改代价计算：使用转向角作为主要代价 ***               
                // 计算转向角：当前航向与到下一个点的航向之间的夹角
         double az_next = Utils::calcAzimuth(current->pos, next_pos);
 double turn_angle = std::abs(Utils::normalizeAngle(az_next - current->incoming_az));
                
            // 代价组成：
   // g_cost = 累积转向角 + 距离代价（较小权重）
       double turn_cost = turn_angle;  // 转向角代价（主要）
       double dist_cost = dist_segment * 0;  // 距离代价（次要，权重0.1）
      double new_g = current->g_cost + turn_cost + dist_cost;
      
        // h_cost 仍然使用直线距离作为启发式
      double az_to_target = Utils::calcAzimuth(next_pos, target);
      double new_h = std::abs(Utils::normalizeAngle(az_to_target - az_next));
      double new_f = new_g + new_h;
        Node* next_node = new Node{
                    next_pos,
                    new_g,
                    new_h,
                    new_f, // f = g + h
                    az_next,  // 记录新的航向
                    current,
                    i
                    };

  node_pool.push_back(next_node);
        open_list.push({ next_node });
            }
        }
    }

    // 回溯路径
    std::vector<Point> path;
    if (best_node) {
      Node* curr = best_node;
        while (curr) {
  path.push_back(curr->pos);
         curr = curr->parent;
    }
        std::reverse(path.begin(), path.end());
    
        // 输出最终路径的详细信息
        std::cout << "\n[Planner] Final Path (" << path.size() << " waypoints):" << std::endl;
        for (size_t i = 0; i < path.size(); ++i) {
      std::cout << "  WP" << i << ": (" 
    << path[i].lon * Config::RAD2DEG << "°, " 
        << path[i].lat * Config::RAD2DEG << "°)" << std::endl;
        }
    }
    else {
        std::cerr << "[Planner] Failed to find a path." << std::endl;
    }

    // 内存清理
    for (auto p : node_pool) {
        delete p;
    }

    return path;
}

// 内部辅助: 获取球面切点
std::vector<Point> AStarPlanner::getSphericalTangents(const Point& p, const NoFlyZone& zone) {
    Point c = { zone.lon, zone.lat };
    double D = Utils::calcDistRad(p, c);
    double R_safe = zone.getSafeRadius();

    if (D <= R_safe + 1e-9) return {}; // 在圆内或非常接近

    // 球面正弦定理: sin(alpha) = sin(R) / sin(D)
    double sin_val = std::sin(R_safe) / std::sin(D);
    if (sin_val > 1.0) sin_val = 1.0;
    if (sin_val < -1.0) sin_val = -1.0;
    double alpha = std::asin(sin_val);

    // 球面余弦定理: cos(D) = cos(R)*cos(Dt)
    double cos_Dt = std::cos(D) / std::cos(R_safe);
    if (cos_Dt > 1.0) cos_Dt = 1.0;
    if (cos_Dt < -1.0) cos_Dt = -1.0;
    double Dt = std::acos(cos_Dt);

    double az_center = Utils::calcAzimuth(p, c);

    return {
        Utils::calcDirect(p, az_center - alpha, Dt),
        Utils::calcDirect(p, az_center + alpha, Dt)
    };
}

// 内部辅助: 全局阻挡检测
bool AStarPlanner::isBlockedGlobal(const Point& p1, const Point& p2, const std::vector<NoFlyZone>& nfzs, int ignore_idx) {
    for (int i = 0; i < (int)nfzs.size(); ++i) {
        if (i == ignore_idx) continue;
        if (Utils::isPathBlocked(p1, p2, nfzs[i])) {
            return true;
        }
    }
    return false;
}

// 新增辅助函数：检查路径是否太接近任何禁飞区（安全裕度检查）
bool AStarPlanner::isTooCloseToNFZ(const Point& p1, const Point& p2, const std::vector<NoFlyZone>& nfzs, int ignore_idx, double safety_factor) {
    for (int i = 0; i < (int)nfzs.size(); ++i) {
        if (i == ignore_idx) continue;
        
        Point c = { nfzs[i].lon, nfzs[i].lat };
        double r_safe = nfzs[i].getSafeRadius();
        double r_extra_safe = r_safe * safety_factor; // 额外的安全裕度
        
        // 计算路径到禁飞区中心的最小距离（Cross-Track Distance）
    double az_12 = Utils::calcAzimuth(p1, p2);
double az_1c = Utils::calcAzimuth(p1, c);
        double dist_1c = Utils::calcDistRad(p1, c);
        
      // sin(CTD) = sin(dist_1c) * sin(az_12 - az_1c)
        double ctd_sin = std::sin(dist_1c) * std::sin(Utils::normalizeAngle(az_12 - az_1c));
     if (ctd_sin > 1.0) ctd_sin = 1.0;
        if (ctd_sin < -1.0) ctd_sin = -1.0;
        double ctd = std::abs(std::asin(ctd_sin)); // 取绝对值
 
        // 检查垂足是否在线段内
        double cos_atd = std::cos(dist_1c) / std::cos(ctd);
        if (cos_atd > 1.0) cos_atd = 1.0;
   if (cos_atd < -1.0) cos_atd = -1.0;
        double atd = std::acos(cos_atd);
        double total_dist = Utils::calcDistRad(p1, p2);
        
        if (std::abs(Utils::normalizeAngle(az_12 - az_1c)) > M_PI / 2.0) {
     atd = -atd;
        }
        
        // 如果垂足在线段内，且距离小于安全裕度，则认为太接近
        if (atd > 0 && atd < total_dist && ctd < r_extra_safe) {
 return true; // 太接近了
  }
     
        // 也检查端点距离
   if (Utils::calcDistRad(p1, c) < r_extra_safe || Utils::calcDistRad(p2, c) < r_extra_safe) {
      return true;
        }
    }
    return false;
}