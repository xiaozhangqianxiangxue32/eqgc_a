// main.cpp
#include "config.h"
#include "dynamics.h"
#include "planner.h"
#include "guidance.h"
#include "utils.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath> 

using namespace Config;

// 辅助函数：将角度半径转换为米（球面距离）
double deg2meters(double degrees) {
    return degrees * Config::DEG2RAD * Config::Re;
}

int main() {
    // =============================================================
    // 1. 任务设置 (初始状态, 目标点, 禁飞区)
    // =============================================================
    std::cout << "--- Step 1: Mission Setup ---\n";

    std::string result_path = "D:\\4-program\\1\\eqgc\\result\\bao_simulation.csv";
    std::string config_path = "D:\\4-program\\1\\eqgc\\result\\mission_config.csv";
    std::string waypoint_path = "D:\\4-program\\1\\eqgc\\result\\waypoints.csv";
    DataLogger logger(result_path);

    // 初始状态 (根据论文 Section 6.2)
    State start_state;
    start_state.r = Config::Re + 70000.0;     // H0 = 70 km
    start_state.lon = 0.0 * Config::DEG2RAD;  // lambda0 = 0 deg
    start_state.lat = 0.0 * Config::DEG2RAD;  // phi0 = 0 deg
    start_state.v = 6800.0;                   // V0 = 6800 m/s
    start_state.gamma = 0.0 * Config::DEG2RAD;// theta0 = 0 deg
    start_state.psi = 90.0 * Config::DEG2RAD; // sigma0 = 105 deg
    start_state.t = 0.0;

    // 禁飞区加载 (根据论文 Section 6.2 Table/Text)
    std::vector<NoFlyZone> no_fly_zones;
    struct RawNFZ { double lat; double lon; double r_deg; double h_m; };
    std::vector<RawNFZ> raw_list = {
        { -10.0, 28.0, 6.0, 70000.0 },
        {   3.0, 35.0, 7.0, 70000.0 },
        {  -7.0, 58.0, 9.0, 70000.0 },
        {  12.0, 60.0, 8.0, 70000.0 },
        {  -6.0, 76.0, 5.0, 70000.0 }
    };

    for (const auto& item : raw_list) {
        NoFlyZone z;
        z.lat = item.lat * Config::DEG2RAD;
        z.lon = item.lon * Config::DEG2RAD;
        double radius_in_rad = item.r_deg * Config::DEG2RAD;
        z.radius = radius_in_rad; // 半径以弧度存储
        z.omega = 0.03; // 安全系数
        no_fly_zones.push_back(z);
    }

    // 目标点 (根据论文 Section 6.2)
    Point target_point = { 90.0 * Config::DEG2RAD, 0.0 * Config::DEG2RAD };

    saveMissionConfig(config_path, start_state, target_point, no_fly_zones);

    // =============================================================
    // 2. 航迹规划 (A* Planner)
    // =============================================================
    std::cout << "\n--- Step 2: Path Planning ---\n";
    auto path = AStarPlanner::plan(start_state, target_point, no_fly_zones);

    if (path.size() < 2) {
        std::cerr << "[Main] Path planning failed or path is too short. Exiting." << std::endl;
        return -1;
    }

    // 打印规划出的航迹点
    std::cout << "[Main] Planned Path Points (Lon, Lat) in degrees:\n";
    std::cout << std::fixed << std::setprecision(6);
    for (const auto& p : path) {
        std::cout << "  - Lon: " << p.lon * 180.0 / M_PI << ", Lat: " << p.lat * 180.0 / M_PI << std::endl;
    }

    // 保存航迹点到文件
    saveWaypoints(waypoint_path, path);

    // =============================================================
    // 3. 制导与动力学仿真
    // =============================================================
    std::cout << "\n--- Step 3: Guidance and Simulation ---\n";
    HGVDynamics dynamics;
    GuidanceQEGC guidance(&dynamics, path); // *** 使用新的构造函数 ***
    State current_state = start_state;

    std::vector<State> history;
    history.push_back(current_state);

    const double dt = 0.05;
    const int max_steps = 800000;
    const int log_interval = (int)(0.5 / dt);   // 每 0.5s 记录一次
    const int print_interval = (int)(10.0 / dt); // 每 10s 打印一次

    std::cout << "[Main] Starting Simulation...\n";

    for (int i = 0; i < max_steps; ++i) {
        // 1. 调用制导律
        auto command = guidance.update(current_state);

        // 2. 动力学积分
        current_state = dynamics.propagate(current_state, command.alpha, command.bank, dt);
        history.push_back(current_state);

        // 3. 检查终止/异常条件
        double altitude = current_state.r - Config::Re;

        if (altitude <= 25000.0) {
            std::cout << "[Exit] Reached TAEM Altitude (30km): " << altitude / 1000.0 << " km at t=" << current_state.t << "s\n";
            break;
        }

        if (altitude < 20000.0) {
            std::cout << "[Warning/Exit] Altitude too low (<20km) without reaching target.\n";
            break;
        }

        if (!std::isfinite(current_state.v) || !std::isfinite(current_state.gamma) || !std::isfinite(current_state.r)) {
            std::cerr << "[Error] NaN/Inf detected at t=" << current_state.t << "s\n";
            break;
        }

        // 4. 数据记录与状态输出
        if (i % log_interval == 0) {
            logger.log(current_state, command);
        }
        Point cur_pos = { current_state.lon, current_state.lat };
        double dist_target = AStarPlanner::calcDist(cur_pos, target_point) * Config::Re;
        if (dist_target / 1000.0 < 1)break;
        if (i % print_interval == 0) {
            Point cur_pos = { current_state.lon, current_state.lat };
            double dist_target = AStarPlanner::calcDist(cur_pos, target_point) * Config::Re;
            std::cout << "t=" << std::fixed << std::setprecision(1) << current_state.t
                << "s H=" << std::setprecision(2) << altitude / 1000.0 << "km"
                << " v=" << std::setprecision(1) << current_state.v << "m/s"
                << " gamma=" << std::setprecision(2) << current_state.gamma * Config::RAD2DEG << "°"
                << " psi=" << std::setprecision(2) << current_state.psi * Config::RAD2DEG << "°"
                << " DistT=" << std::setprecision(1) << dist_target / 1000.0 << "km"
                << std::endl;
        }
    }

    // =============================================================
    // 4. 仿真结束与结果输出
    // =============================================================
    logger.flush();
    std::cout << "\n=== Simulation Complete ===\n";
    
    Point end_pos = { current_state.lon, current_state.lat };
    double final_miss_distance = AStarPlanner::calcDist(end_pos, target_point) * Config::Re / 1000.0;

    std::cout << "[Final] Time: " << current_state.t << "s\n";
    std::cout << "[Final] Altitude: " << (current_state.r - Config::Re) / 1000.0 << "km\n";
    std::cout << "[Final] Velocity: " << current_state.v << "m/s\n";
    std::cout << "[Final] Miss Distance: " << final_miss_distance << "km\n";

    return 0;
}