#pragma once
#include "config.h"
#include <vector>
#include <stdexcept>
#include <algorithm>

// ========================================================================
// B-Spline Class for Path Smoothing
// ========================================================================
//
// 核心功能：
// 1. 使用三阶(cubic)B-样条对A*生成的离散航迹点进行插值，生成C2连续的平滑路径。
// 2. 提供 "evaluate" 方法，用于计算样条曲线上任意参数u对应的点坐标。
// 3. 提供 "findClosestParam" 方法，用于在样条上寻找离飞行器当前位置最近的点。
//
// 使用方法：
// 1. 创建 Spline 对象。
// 2. 调用 generate(waypoints) 生成样条。
// 3. 在仿真循环中：
//    a. 调用 findClosestParam(vehicle_pos) 找到当前最近点参数 u_current。
//    b. 计算前馈参数 u_lookahead。
//    c. 调用 evaluate(u_current + u_lookahead) 获得制导目标点。
//
// ========================================================================

class Spline {
public:
    Spline() : degree(3) {} // 使用三阶B样条，保证C2连续性

    /**
     * @brief 从一系列控制点（航迹点）生成B样条
     * @param points A*算法生成的航迹点
     */
    void generate(const std::vector<Point>& points) {
        if (points.size() < 4) {
            // 控制点太少，无法生成三阶B样条，可以采用线性插值等方式降级处理
            // 这里为了简化，我们直接复制点，但这在实际应用中应更智能地处理
            control_points = points;
            if (points.size() > 1) {
                while (control_points.size() < 4) {
                    control_points.push_back(points.back());
                }
            }
            else if (!points.empty()) {
                while (control_points.size() < 4) {
                    control_points.push_back(points.front());
                }
            }
            else {
                throw std::runtime_error("Spline generation failed: No control points provided.");
            }
        }
        else {
            control_points = points;
        }

        // 创建一个开放、均匀的节点向量 (clamped knot vector)
        // 节点数量 = 控制点数量 + 阶数 + 1
        knot_vector.clear();
        size_t n = control_points.size();
        size_t p = degree;
        size_t num_knots = n + p + 1;

        // 前 p+1 个节点为0
        for (size_t i = 0; i <= p; ++i) {
            knot_vector.push_back(0.0);
        }
        // 中间的节点均匀分布
        for (size_t i = 1; i <= n - p - 1; ++i) {
            knot_vector.push_back(static_cast<double>(i));
        }
        // 最后 p+1 个节点为 n-p
        for (size_t i = 0; i <= p; ++i) {
            knot_vector.push_back(static_cast<double>(n - p));
        }
    }

    /**
     * @brief 计算样条上给定参数u处的点坐标 (De Boor's Algorithm)
     * @param u 参数，范围在 [getMinKnot(), getMaxKnot()]
     * @return Point 样条上的点
     */
    Point evaluate(double u) const {
        if (knot_vector.empty()) {
            throw std::runtime_error("Spline not generated yet.");
        }

        // 将u限制在有效范围内
        u = std::max(getMinKnot(), std::min(u, getMaxKnot()));

        // 找到u所在的节点区间 [knot[k], knot[k+1])
        size_t k = 0;
        for (size_t i = degree; i < knot_vector.size() - degree - 1; ++i) {
            if (u >= knot_vector[i]) {
                k = i;
            }
        }

        // De Boor's algorithm
        std::vector<Point> d = control_points;
        for (size_t r = 1; r <= degree; ++r) {
            for (size_t j = degree; j >= r; --j) {
                size_t idx = j + k - degree;
                double alpha_num = u - knot_vector[idx];
                double alpha_den = knot_vector[idx + degree - r + 1] - knot_vector[idx];
                double alpha = (alpha_den == 0.0) ? 0.0 : alpha_num / alpha_den;

                d[j].lon = (1.0 - alpha) * d[j - 1].lon + alpha * d[j].lon;
                d[j].lat = (1.0 - alpha) * d[j - 1].lat + alpha * d[j].lat;
            }
        }
        return d[degree];
    }

    /**
     * @brief 在样条上寻找离给定点p最近的点对应的参数u
     * @param p 飞行器当前位置
     * @return double 最近点参数u
     */
    double findClosestParam(const Point& p) const {
        if (knot_vector.empty()) return 0.0;

        double min_dist_sq = -1.0;
        double best_u = getMinKnot();

        // 1. 粗略搜索：在整个样条上采样，找到距离最近的采样点
        const int num_samples = 200; // 采样点数量，可以根据路径长度调整
        for (int i = 0; i <= num_samples; ++i) {
            double u = getMinKnot() + (getMaxKnot() - getMinKnot()) * i / num_samples;
            Point spline_pt = evaluate(u);

            // 使用简化的平面距离平方进行比较，避免开方运算
            double d_lon = p.lon - spline_pt.lon;
            double d_lat = p.lat - spline_pt.lat;
            double dist_sq = d_lon * d_lon + d_lat * d_lat;

            if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_u = u;
            }
        }

        // 2. 精细搜索：在粗略找到的最佳u值附近进行迭代优化
        // 使用简单的梯度下降思想，迭代几次以提高精度
        const int refine_steps = 5;
        double step_size = (getMaxKnot() - getMinKnot()) / num_samples / 2.0; // 步长为采样间隔的一半

        for (int i = 0; i < refine_steps; ++i) {
            double u_minus = std::max(getMinKnot(), best_u - step_size);
            double u_plus = std::min(getMaxKnot(), best_u + step_size);

            Point pt_minus = evaluate(u_minus);
            Point pt_plus = evaluate(u_plus);

            double d_lon_m = p.lon - pt_minus.lon;
            double d_lat_m = p.lat - pt_minus.lat;
            double dist_sq_m = d_lon_m * d_lon_m + d_lat_m * d_lat_m;

            double d_lon_p = p.lon - pt_plus.lon;
            double d_lat_p = p.lat - pt_plus.lat;
            double dist_sq_p = d_lon_p * d_lon_p + d_lat_p * d_lat_p;

            if (dist_sq_m < min_dist_sq) {
                min_dist_sq = dist_sq_m;
                best_u = u_minus;
            }
            if (dist_sq_p < min_dist_sq) {
                min_dist_sq = dist_sq_p;
                best_u = u_plus;
            }
            step_size /= 2.0; // 减小步长
        }

        return best_u;
    }

    double getMinKnot() const {
        if (knot_vector.empty()) return 0.0;
        return knot_vector[degree];
    }

    double getMaxKnot() const {
        if (knot_vector.empty()) return 0.0;
        return knot_vector[knot_vector.size() - 1 - degree];
    }

private:
    size_t degree;
    std::vector<Point> control_points;
    std::vector<double> knot_vector;
};