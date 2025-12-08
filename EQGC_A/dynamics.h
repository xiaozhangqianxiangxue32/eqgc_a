#pragma once
#include "config.h"

class HGVDynamics {
private:
    // 模型系数
    double MASS_;
    double S_REF_;
    double V_ORBITAL_;
    double T_SCALE_;
    double CL_C_[8], CD_C_[8];
    double CL_SCALE_, CD_SCALE_;
    double RHO_A_[8], SOS_COEFFS_[11];

    struct DimensionlessState { double r, lon, lat, v, theta, psi; };
    struct ControlInput { double alpha, sigma; };

   
    
    double getCL(double alpha_deg, double mach) const;
    double getCD(double alpha_deg, double mach) const;
    DimensionlessState calculateDerivatives(const DimensionlessState& s, const ControlInput& u);

public:
    HGVDynamics();
    // 【关键修改】这两个函数现在是 public 的，供制导律调用以保持一致性
    double getDensity(double h_meters) const;
    double getSoS(double h_meters) const;
    // 【修正】统一接口：输入 alpha(rad), mach，输出 CL, CD
    void getAerodynamics(double alpha_rad, double mach, double& CL, double& CD);

    State propagate(const State& s, double alpha_rad, double bank_rad, double dt);
};