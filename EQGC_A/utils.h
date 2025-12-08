#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include "config.h"
#include "guidance.h"
#include "planner.h" // 需要 Point 定义

// 保存任务配置 (起点, 终点, 禁飞区) 到 CSV
// 路径建议与仿真结果相同
inline void saveMissionConfig(const std::string& filename,
    const State& start,
    const Point& target,
    const std::vector<NoFlyZone>& nfzs) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[Error] 无法创建配置文件: " << filename << std::endl;
        return;
    }

    // 写入表头
    // Type: 0=Start, 1=Target, 2=NFZ
    file << "Type,Lat_deg,Lon_deg,Radius_m\n";

    // 1. 保存起点 (Type 0)
    file << "0," << start.lat * Config::RAD2DEG << ","
        << start.lon * Config::RAD2DEG << ",0\n";

    // 2. 保存终点 (Type 1)
    file << "1," << target.lat * Config::RAD2DEG << ","
        << target.lon * Config::RAD2DEG << ",0\n";

    // 3. 保存禁飞区 (Type 2)
    for (const auto& z : nfzs) {
        // 半径需转回米: R_m = R_rad * Re
        double r_m = z.radius * Config::Re;
        file << "2," << z.lat * Config::RAD2DEG << ","
            << z.lon * Config::RAD2DEG << ","
            << r_m << "\n";
    }

    file.close();
    std::cout << "[Config] Mission configuration saved to: " << filename << std::endl;
}

// 保存航迹规划点到CSV文件
inline void saveWaypoints(const std::string& filename, const std::vector<Point>& waypoints) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[Error] 无法打开航迹点文件: " << filename << std::endl;
        return;
    }

    // 写入表头
    file << "Index,Lat_deg,Lon_deg\n";

    // 写入每个航迹点
    for (size_t i = 0; i < waypoints.size(); ++i) {
        file << std::fixed << std::setprecision(6);
        file << i << ","
            << waypoints[i].lat * Config::RAD2DEG << ","
            << waypoints[i].lon * Config::RAD2DEG << "\n";
    }

    file.close();
    std::cout << "[Config] 航迹点已保存到: " << filename << " (" << waypoints.size() << " points)" << std::endl;
}

class DataLogger {
    std::ofstream file;
    int log_count;

public:
    DataLogger(std::string name) : log_count(0) {
        // 使用二进制模式打开，避免缓冲问题
        file.open(name, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            std::cerr << "[Error] 无法打开结果文件: " << name << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // 关闭缓冲区，确保实时写入
        file << std::unitbuf;

        file << "Time,Alt_km,Lon_deg,Lat_deg,Vel,Gamma_deg,Psi_deg,Alpha_deg,Bank_deg\n";
        file.flush();

        std::cout << "[DataLogger] Initialized: " << name << std::endl;
    }

    ~DataLogger() {
        if (file.is_open()) {
            file.flush();
            file.close();
            std::cout << "[DataLogger] Closed. Total logs: " << log_count << std::endl;
        }
    }

    void log(const State& s, const GuidanceQEGC::Command& u) {
        if (!file.is_open() || !file.good()) {
            std::cerr << "[Error] File stream is not valid!" << std::endl;
            return;
        }

        file << std::fixed << std::setprecision(6);
        file << s.t << ","
            << (s.r - Config::Re) / 1000.0 << ","
            << s.lon * Config::RAD2DEG << ","
            << s.lat * Config::RAD2DEG << ","
            << s.v << ","
            << s.gamma * Config::RAD2DEG << ","
            << s.psi * Config::RAD2DEG << ","
            << u.alpha * Config::RAD2DEG << ","
            << u.bank * Config::RAD2DEG << "\n";

        log_count++;

        // 每5次记录强制刷新一次
        if (log_count % 5 == 0) {
            file.flush();
        }
    }

    // 添加手动刷新接口
    void flush() {
        if (file.is_open()) {
            file.flush();
        }
    }

    // 获取记录计数
    int getLogCount() const {
        return log_count;
    }
};