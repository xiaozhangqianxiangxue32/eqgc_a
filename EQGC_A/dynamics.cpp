#include "dynamics.h"
#include <algorithm>
#include <iostream>

using namespace Config;

HGVDynamics::HGVDynamics() {
    // 1. 初始化系数 (基于您提供的 FMT_Dynamics 数据)
    MASS_ = 907.0;
    S_REF_ = 0.4839;
    V_ORBITAL_ = std::sqrt(g0 * Re);
    T_SCALE_ = Re / V_ORBITAL_;

    // 大气密度系数
    RHO_A_[0] = -0.009334500409893852; RHO_A_[1] = -0.07713480853494097;
    RHO_A_[2] = -0.00382948508292597;  RHO_A_[3] = 5.242803244759748E-05;
    RHO_A_[4] = 6.454862591920205E-07; RHO_A_[5] = -2.031133609734722E-08;
    RHO_A_[6] = 1.568378909033718E-10; RHO_A_[7] = -3.928350728483702E-13;

    // 声速系数
    SOS_COEFFS_[0] = 340.29112;   SOS_COEFFS_[1] = -0.87245462;
    SOS_COEFFS_[2] = -300.10255;  SOS_COEFFS_[3] = 0.3077688;
    SOS_COEFFS_[4] = 106.65518;   SOS_COEFFS_[5] = -0.052091446;
    SOS_COEFFS_[6] = -18.273173;  SOS_COEFFS_[7] = 0.004143208;
    SOS_COEFFS_[8] = 1.485658;    SOS_COEFFS_[9] = -0.00012124848;
    SOS_COEFFS_[10] = -0.045201417;

    // CL 系数
    CL_C_[0] = -2.1339761045e-01; CL_C_[1] = 1.8276022823e-02;
    CL_C_[2] = -4.2706874139e-04; CL_C_[3] = 6.1331641681e-02;
    CL_C_[4] = -2.8911829827e-03 / 2.0; CL_C_[5] = 6.2069931984e-05 / 2.0;
    CL_C_[6] = -3.4186422713e-04; CL_C_[7] = 2.2512555205e-05 / 2.0;
    CL_SCALE_ = 1.0;

    // CD 系数
    CD_C_[0] = 1.4307508046e-01; CD_C_[1] = -1.0540914763e-02;
    CD_C_[2] = 1.9330624745e-04; CD_C_[3] = -3.9163179502e-03;
    CD_C_[4] = 2.7954514516e-04; CD_C_[5] = 9.2107503583e-06;
    CD_C_[6] = 9.6523608833e-04; CD_C_[7] = -2.1065993691e-05;
    CD_SCALE_ = 0.57;
}

double HGVDynamics::getDensity(double h_meters) const {
    if (h_meters > 130000.0) return 0.0;
    double h_km = h_meters / 1000.0;
    double h_pow = 1.0;
    double poly = 0.0;
    for (int i = 0; i < 8; ++i) {
        poly += RHO_A_[i] * h_pow;
        h_pow *= h_km;
    }
    return 1.225 * std::exp(poly);
}

double HGVDynamics::getSoS(double h_meters) const {
    double h_km = std::min(85.0, std::max(0.0, h_meters / 1000.0));
    double sqrt_z = std::sqrt(h_km);
    double z = h_km;
    double p1_5 = std::pow(z, 1.5);
    double p2_0 = z * z;
    double p2_5 = std::pow(z, 2.5);

    double d1 = SOS_COEFFS_[0] + SOS_COEFFS_[2] * sqrt_z + SOS_COEFFS_[4] * z +
        SOS_COEFFS_[6] * p1_5 + SOS_COEFFS_[8] * p2_0 + SOS_COEFFS_[10] * p2_5;

    double d2 = 1.0 + SOS_COEFFS_[1] * sqrt_z + SOS_COEFFS_[3] * z +
        SOS_COEFFS_[5] * p1_5 + SOS_COEFFS_[7] * p2_0 + SOS_COEFFS_[9] * p2_5;
    return d1 / d2;
}

double HGVDynamics::getCL(double alpha_deg, double mach) const {
    double A = alpha_deg;
    double M = mach;
    double cl = CL_C_[0] + CL_C_[1] * M + CL_C_[2] * M * M +
        CL_C_[3] * A + CL_C_[4] * A * M + CL_C_[5] * A * M * M +
        CL_C_[6] * A * A + CL_C_[7] * A * A * M;
    return cl * CL_SCALE_;
}

double HGVDynamics::getCD(double alpha_deg, double mach) const {
    double A = alpha_deg;
    double M = mach;
    double cd = CD_C_[0] + CD_C_[1] * M + CD_C_[2] * M * M +
        CD_C_[3] * A + CD_C_[4] * A * M + CD_C_[5] * A * M * M +
        CD_C_[6] * A * A + CD_C_[7] * A * A * M;
    return cd * CD_SCALE_;
}

void HGVDynamics::getAerodynamics(double alpha_rad, double mach, double& CL, double& CD) {
    double deg = alpha_rad * RAD2DEG;
    CL = getCL(deg, mach);
    CD = getCD(deg, mach);
}

// 核心: 无量纲微分方程 (修复点：填入完整的动力学公式)
HGVDynamics::DimensionlessState HGVDynamics::calculateDerivatives(const DimensionlessState& s, const ControlInput& u) {
    DimensionlessState ds;

    // 1. 辅助变量
    double sin_theta = std::sin(s.theta);
    double cos_theta = std::cos(s.theta);
    double sin_psi = std::sin(s.psi);
    double cos_psi = std::cos(s.psi);
    double cos_phi = std::cos(s.lat);
    double tan_phi = std::tan(s.lat);
    double sin_sigma = std::sin(u.sigma);
    double cos_sigma = std::cos(u.sigma);

    // 2. 物理量反算 (用于气动计算)
    double H_meters = (s.r - 1.0) * Config::Re;
    double V_mps = s.v * V_ORBITAL_;
    double alpha_deg = u.alpha * RAD2DEG;

    // 3. 气动计算
    double rho = getDensity(H_meters);
    double sos = getSoS(H_meters);
    double mach = V_mps / sos;
    double cl = getCL(alpha_deg, mach);
    double cd = getCD(alpha_deg, mach);

    // 4. 无量纲气动力
    double v_sq = s.v * s.v;
    // L_norm = (0.5 * rho * v^2 * S * Re) / m
    double L_norm = (0.5 * rho * v_sq * S_REF_ * Config::Re * cl) / MASS_;
    double D_norm = (0.5 * rho * v_sq * S_REF_ * Config::Re * cd) / MASS_;

    // 5. 运动方程 (完整实现)
    ds.r = s.v * sin_theta;

    // 避免除以零 (极点奇异性保护)
    double cos_phi_safe = (std::abs(cos_phi) < 1e-6) ? 1e-6 : cos_phi;
    ds.lon = (s.v * cos_theta * sin_psi) / (s.r * cos_phi_safe);
    ds.lat = (s.v * cos_theta * cos_psi) / s.r;

    // 修正：速度微分方程 (包含阻力和重力切向分量)
    // dv/dt = -D - g*sin(theta)
    // 无量纲形式：dv/dτ = -D_norm - (1/r?)*sin(theta)
    ds.v = -D_norm - (sin_theta / (s.r * s.r));

    // 修正：航迹角微分方程
    // dθ/dt = (L*cos(σ)/v) + (v*cos(θ)/r - g*cos(θ)/v)
    // 无量纲形式：dθ/dτ = (L_norm*cos(σ)/v) + (v*cos(θ)/r - cos(θ)/r?/v)
    // 简化：dθ/dτ = (L_norm*cos(σ)/v) + cos(θ)*(v/r - 1/(r?*v))
    ds.theta = (L_norm * cos_sigma / s.v) + cos_theta * (s.v / s.r - 1.0 / (s.r * s.r * s.v));

    // psi_dot (航向角微分方程)
    ds.psi = (L_norm * sin_sigma / (s.v * cos_theta)) +
        (s.v * cos_theta * sin_psi * tan_phi) / s.r;

    return ds;
}

State HGVDynamics::propagate(const State& s, double alpha_rad, double bank_rad, double dt) {
    // 1. SI -> 无量纲
    DimensionlessState y0;
    y0.r = (Config::Re + s.r - Config::Re) / Config::Re; // 注意：输入s.r是地心距
    y0.lon = s.lon; y0.lat = s.lat;
    y0.v = s.v / V_ORBITAL_;
    y0.theta = s.gamma; y0.psi = s.psi;

    ControlInput u = { alpha_rad, bank_rad };
    double h = dt / T_SCALE_; // 无量纲步长

    // 2. RK4 积分
    // k1
    DimensionlessState k1 = calculateDerivatives(y0, u);

    // k2
    DimensionlessState y2 = y0;
    y2.r += k1.r * h * 0.5; y2.lon += k1.lon * h * 0.5; y2.lat += k1.lat * h * 0.5;
    y2.v += k1.v * h * 0.5; y2.theta += k1.theta * h * 0.5; y2.psi += k1.psi * h * 0.5;
    DimensionlessState k2 = calculateDerivatives(y2, u);

    // k3
    DimensionlessState y3 = y0;
    y3.r += k2.r * h * 0.5; y3.lon += k2.lon * h * 0.5; y3.lat += k2.lat * h * 0.5;
    y3.v += k2.v * h * 0.5; y3.theta += k2.theta * h * 0.5; y3.psi += k2.psi * h * 0.5;
    DimensionlessState k3 = calculateDerivatives(y3, u);

    // k4
    DimensionlessState y4 = y0;
    y4.r += k3.r * h; y4.lon += k3.lon * h; y4.lat += k3.lat * h;
    y4.v += k3.v * h; y4.theta += k3.theta * h; y4.psi += k3.psi * h;
    DimensionlessState k4 = calculateDerivatives(y4, u);

    // Update
    DimensionlessState y_next = y0;
    y_next.r += (h / 6.0) * (k1.r + 2 * k2.r + 2 * k3.r + k4.r);
    y_next.lon += (h / 6.0) * (k1.lon + 2 * k2.lon + 2 * k3.lon + k4.lon);
    y_next.lat += (h / 6.0) * (k1.lat + 2 * k2.lat + 2 * k3.lat + k4.lat);
    y_next.v += (h / 6.0) * (k1.v + 2 * k2.v + 2 * k3.v + k4.v);
    y_next.theta += (h / 6.0) * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta);
    y_next.psi += (h / 6.0) * (k1.psi + 2 * k2.psi + 2 * k3.psi + k4.psi);

    // 3. 无量纲 -> SI
    State s_next;
    s_next.r = y_next.r * Config::Re;
    s_next.lon = y_next.lon;
    s_next.lat = y_next.lat;
    s_next.v = y_next.v * V_ORBITAL_;
    s_next.gamma = y_next.theta;
    s_next.psi = y_next.psi;
    s_next.t = s.t + dt;

    return s_next;
}