// config.h
#pragma once // 防止头文件重复包含
#define _USE_MATH_DEFINES // 解决 VS 中 M_PI 未定义问题

#include <cmath>
#include <vector>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Config {
    // --- 物理常数 (SI) ---
    constexpr double Re = 6371000.0;        // 地球半径 (m)
    constexpr double g0 = 9.80665;          // 标准重力
    constexpr double mu = 3.986004418e14;   // 地球引力常数
    constexpr double rho0 = 1.225;          // 海平面密度
    constexpr double Hs = 7200.0;           // 标尺高度

    // --- 飞行器参数 ---
    constexpr double Mass = 500.0;          // 质量 (kg)
    constexpr double S_ref = 0.5;           // 参考面积 (m^2)

    // --- 约束参数 ---
  // [修正] 名称需匹配 guidance.cpp 中的 Config::Q_dot_max
    constexpr double Q_dot_max = 3.8e6;     // 热流密度上限 (W/m^2)
    constexpr double q_max = 90000.0;       // 动压上限 (Pa)

    // [新增] 缺失的过载约束和热流系数
    constexpr double n_max = 4.0;           // 最大过载 (g)
    constexpr double kh = 9.437e-5;         // 热流计算系数
    constexpr double Q_max = 3.8e6;         // 热流
  //  constexpr double q_max = 90000.0;       // 动压
    constexpr double alpha_min = 2.0;       // deg
    constexpr double alpha_max = 20.0;      // deg

    // --- 辅助转换 ---
    constexpr double DEG2RAD = M_PI / 180.0;
    constexpr double RAD2DEG = 180.0 / M_PI;
}

// 状态结构体
struct State {
    double r;      // m
    double lon;    // rad
    double lat;    // rad
    double v;      // m/s
    double gamma;  // rad
    double psi;    // rad
    double t;      // s
};

// 禁飞区结构体
// 【重要修正】放在这里，确保 Planner 能找到它
struct NoFlyZone {
    double lon;    // rad
    double lat;    // rad
    double radius; // rad
    double omega = 0.03;

    double getSafeRadius() const { return radius * (1.0 + omega); }
};

// 简单的点结构
struct Point {
    double lon; // rad
    double lat; // rad
};