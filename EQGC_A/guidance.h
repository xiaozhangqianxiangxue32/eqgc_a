#pragma once
#include "config.h"
#include "dynamics.h"
#include <vector> // 包含 vector 头文件

class GuidanceQEGC {
private:
    // 调整增益参数以提供更强的高度控制
    double k_t1 = 2.0, k_t2 = 0.5, eps_t = 0.01;  // 增加纵向增益
    double k_s1 = 4.0, k_s2 = 0.001, eps_s = 0.01;
    HGVDynamics* dyn_ptr; // 持有动力学指针

    // 新增：航迹跟踪相关成员
    std::vector<Point> path;
    int current_waypoint_idx = 1; // 从第二个点开始作为目标 (第一个点是起点)

    double normalizeAngle(double angle);
    double solveAlpha(double CL_req, double Mach);

public:
    // 修改构造函数以接收航迹
    GuidanceQEGC(HGVDynamics* dyn, const std::vector<Point>& p) : dyn_ptr(dyn), path(p) {}

    struct Command { double alpha; double bank; };
    // 修改 update 函数，不再需要 target 参数
    Command update(const State& state);
};