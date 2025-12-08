#include "guidance.h"
#include "planner.h" // 包含 planner.h 以识别 AStarPlanner 类
#include <algorithm>
#include <iostream>
#include <cmath> // 确保包含 math 库

using namespace Config;
// [辅助] 滑模平滑切换函数 tanh(s/eps)
// 论文中使用 tanh 代替 sign 以消除抖动 [cite: 345]
inline double smc_tanh(double s, double eps) {
    if (std::abs(eps) < 1e-6) eps = 1e-6;
    return std::tanh(s / eps);
}
// 辅助函数：角度归一化到 (-pi, pi]
double GuidanceQEGC::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
 while (angle <= -M_PI) angle += 2 * M_PI;
    return angle;
}

// 核心函数：二分法反解攻角
// 已知: 需要的 CL_req 和当前的 Mach
// 求解: 对应的 Alpha (deg)
double GuidanceQEGC::solveAlpha(double CL_req, double Mach) {
double low = Config::alpha_min;  // 2 deg
    double high = Config::alpha_max; // 20 deg
    
    double cl, cd; // 临时变量

    // 二分查找 (15次迭代精度足够)
    for(int i=0; i<15; ++i) {
        double mid = (low + high) * 0.5;
        
        // 调用 dynamics 获取该攻角下的 CL
    // 注意：传入的是弧度
  dyn_ptr->getAerodynamics(mid * Config::DEG2RAD, Mach, cl, cd);
        
        // 假设 CL 随 Alpha 单调递增
     if(cl < CL_req) {
         low = mid;
        } else {
            high = mid;
        }
    }
    return (low + high) * 0.5; // 返回角度制
}

// 制导律更新函数
GuidanceQEGC::Command GuidanceQEGC::update(const State& s) {
    // --- 航迹跟踪逻辑 ---
    // 确保路径有效且索引在范围内
    if (path.empty() || current_waypoint_idx >= path.size()) {
        // 如果没有路径或已完成，可以返回一个默认指令或最后状态
        std::cerr << "[Guidance] Path is empty or finished." << std::endl;
        return { 0.0, 0.0 };
    }

    // 1. 获取当前目标航点
    Point current_target = path[current_waypoint_idx];

    // 2. 计算到当前航点的距离
    Point current_pos = { s.lon, s.lat }; // 从 State 创建 Point
    double dist_to_waypoint = AStarPlanner::calcDist(current_pos, current_target) * Config::Re;

    // 3. 检查是否到达航点，如果到达则切换到下一个
    // 阈值设为 20km，且确保不是最后一个航点
    if (dist_to_waypoint < 20000.0 && current_waypoint_idx < path.size() - 1) {
        current_waypoint_idx++;
        current_target = path[current_waypoint_idx]; // 更新目标
        std::cout << "[Guidance] Waypoint reached. Switching to next waypoint #" 
                  << current_waypoint_idx << std::endl;
    }
    // --- 结束航迹跟踪逻辑 ---


    // --- 0. 初始化静态变量和常量 ---
    static double prev_alpha_deg = 10.0;
    const double dt = 0.05;

    // --- 1. 导航解算与轨迹生成 ---
    // 目标点现在是 current_target
    double current_psi_los = std::atan2(std::sin(current_target.lon - s.lon) * std::cos(current_target.lat),
        std::cos(s.lat) * std::sin(current_target.lat) - std::sin(s.lat) * std::cos(current_target.lat) * std::cos(current_target.lon - s.lon));

    // 不再需要平滑转弯逻辑，直接使用视线角
    double commanded_psi = current_psi_los;

    // 1.3. 计算剩余距离等
    double central_angle = std::acos(std::sin(s.lat) * std::sin(current_target.lat) +
    std::cos(s.lat) * std::cos(current_target.lat) * std::cos(current_target.lon - s.lon));
    double L_go = central_angle * Config::Re;

    // --- 2. 环境参数计算 ---
    double h = s.r - Config::Re;
    double rho = dyn_ptr->getDensity(h);
    double sos = dyn_ptr->getSoS(h);
    double mach = s.v / sos;
    double q_dyn = 0.5 * rho * s.v * s.v;
    if (q_dyn < 10.0) q_dyn = 10.0;

    // --- 3. 纵向制导 ---
    double r_target = Config::Re + 30000.0; 
    double dist_angle = std::max(central_angle, 1e-6);
    double theta_c = std::atan(std::log(r_target / s.r) / dist_angle);
    double e_theta = s.gamma - theta_c;
    double g = Config::mu / (s.r * s.r);
    double f_theta = (s.v * std::cos(s.gamma) / s.r - g * std::cos(s.gamma) / s.v);
    double theta_dot_req = -k_t1 * e_theta - k_t2 * std::tanh(e_theta / eps_t);
    double az_vert = s.v * (theta_dot_req - f_theta);

    // --- 4. 侧向制Guidance (使用平滑后的航向指令) ---
    double e_psi = normalizeAngle(s.psi - commanded_psi);
    
    // T_g (时间常数) 的计算现在使用到当前航点的剩余距离
    double T_g = std::max(1.0, L_go / (s.v * std::cos(s.gamma)));
    double f_psi = (s.v * std::cos(s.gamma) * std::sin(s.psi) * std::tan(s.lat)) / s.r;
    double psi_dot_req = -(k_s1 / T_g) * e_psi - (k_s2 / T_g) * smc_tanh(e_psi, eps_s);
    double az_horz = s.v * std::cos(s.gamma) * (psi_dot_req - f_psi);

    // --- 5. 指令合成 ---
    double total_acc = std::sqrt(az_vert * az_vert + az_horz * az_horz);
    double bank = std::atan2(az_horz, az_vert);
    double CL_req = (Config::Mass * total_acc) / (q_dyn * Config::S_ref);
    double alpha_deg = solveAlpha(CL_req, mach);
    alpha_deg = std::max(Config::alpha_min, std::min(alpha_deg, Config::alpha_max));

    // === 攻角速率限制 (5 deg/s) ===
    const double MAX_ALPHA_RATE = 5.0; // 5°/s
    double alpha_rate = (alpha_deg - prev_alpha_deg) / dt;
    if (std::abs(alpha_rate) > MAX_ALPHA_RATE) {
        double sign = (alpha_rate > 0) ? 1.0 : -1.0;
        alpha_deg = prev_alpha_deg + sign * MAX_ALPHA_RATE * dt;
    }
    prev_alpha_deg = alpha_deg;

    return { alpha_deg * Config::DEG2RAD, bank };
}
