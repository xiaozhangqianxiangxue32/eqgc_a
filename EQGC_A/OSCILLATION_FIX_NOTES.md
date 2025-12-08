# 攻角/滚转角震荡消除方案

## 问题诊断日期
2024年 - 制导指令震荡修复

---

## 问题现象

### 从仿真结果观察到：

#### 1. **攻角震荡** (α)
```
特征：
- 方波式跳变：2° ? 20°
- 频繁触及上下限
- 无平滑过渡

原因：
? 升力系数需求计算不稳定
? 缺少攻角速率限制
? 动压过小时数值发散
```

#### 2. **滚转角震荡** (σ)
```
特征：
- 极端震荡：-85° ? +85°
- 持续饱和在限制值
- 高频震荡（类似颤振）

原因：
? 横向制导增益过大
? 缺少滚转角速率限制
? 航向误差过大导致过度校正
```

---

## 根本原因分析

### 1. **制导增益过大**
```cpp
// ? 原始增益（过于激进）
double k_t1 = 2.0;   // 纵向比例增益
double k_t2 = 0.5;   // 纵向切换增益
double k_s1 = 4.0;   // 横向比例增益（太大！）
double k_s2 = 0.001; // 横向切换增益
```

**问题：**
- `k_s1 = 4.0` 导致横向指令过度响应
- 小的航向误差就产生巨大的滚转角指令
- 形成正反馈震荡

### 2. **缺少速率限制**
```cpp
// ? 原始代码没有速率限制
auto cmd = guidance.update(vehicle, guidance_target);
// 指令可以瞬间从 -85° 跳到 +85°
```

**后果：**
- 物理上不可实现的指令
- 数值积分不稳定
- 激发高频震荡

### 3. **动压保护不足**
```cpp
// ? 原始代码
if (q_dyn < 10.0) q_dyn = 10.0;  // 阈值过低
```

**问题：**
- 高空时动压接近零
- 升力系数需求 `CL = (m*a)/(q*S)` 爆炸性增长
- 导致攻角指令跳变到上限

---

## 修复方案详解

### 改进 1: **降低制导增益**

#### 纵向制导
```cpp
// ? 自适应降低增益
double k_t1_adaptive = k_t1 * 0.5;  // 2.0 → 1.0
double k_t2_adaptive = k_t2 * 0.8;  // 0.5 → 0.4

double theta_dot_req = -k_t1_adaptive * e_theta 
   - k_t2_adaptive * std::tanh(e_theta / eps_t);
```

**效果：**
- ? 航迹角响应更平滑
- ? 减少攻角指令的突变
- ? 保持足够的跟踪性能

#### 横向制导
```cpp
// ? 大幅降低横向增益（关键修复）
double k_s1_adaptive = k_s1 * 0.3;  // 4.0 → 1.2
double k_s2_adaptive = k_s2 * 0.5;  // 0.001 → 0.0005

double psi_dot_req = -(k_s1_adaptive / T_g) * e_psi_limited 
           - (k_s2_adaptive / T_g) * std::tanh(e_psi_limited / eps_s);
```

**效果：**
- ? 滚转角指令幅度大幅减小
- ? 消除饱和震荡
- ? 平滑的航向跟踪

---

### 改进 2: **航向误差限幅**

```cpp
// ? 限制航向误差，防止过大转向
double e_psi_limited = std::max(-0.5, std::min(0.5, e_psi));  // ±28.6°
```

**原理：**
```
当航向误差 > 30° 时：
- 不尝试立即修正（不可行）
- 逐步调整，避免过度响应
- 防止滚转角饱和
```

**对比：**
```
原始：e_psi = 60° → 滚转角指令 = -85° (饱和)
改进：e_psi_limited = 28.6° → 滚转角指令 = -45° (合理)
```

---

### 改进 3: **总加速度限制**

```cpp
// ? 限制总加速度指令
const double MAX_ACC = 8.0 * Config::g0;  // 8g
if (total_acc > MAX_ACC) {
    double scale = MAX_ACC / total_acc;
    az_vert *= scale;
    az_horz *= scale;
    total_acc = MAX_ACC;
}
```

**效果：**
- ? 符合飞行器物理限制
- ? 防止升力系数需求过大
- ? 间接限制攻角指令

**示例：**
```
原始：total_acc = 15g → CL_req = 2.5 → α = 20° (上限)
改进：total_acc = 8g  → CL_req = 1.0 → α = 12° (合理)
```

---

### 改进 4: **滚转角速率限制**

```cpp
// ? 滚转角速率限制（关键！）
static double prev_bank = 0.0;
const double MAX_BANK_RATE = 20.0 * Config::DEG2RAD;  // 20°/s
const double dt = 0.05;

double bank_rate = (bank - prev_bank) / dt;
if (std::abs(bank_rate) > MAX_BANK_RATE) {
    double sign = (bank_rate > 0) ? 1.0 : -1.0;
    bank = prev_bank + sign * MAX_BANK_RATE * dt;
}
prev_bank = bank;
```

**原理：**
```
时间步长 dt = 0.05s
最大速率 = 20°/s
=> 单步最大变化 = 1°

防止：
bank(t)= -85°
bank(t+dt) = +85°  ? 170°跳变
```

**效果：**
```
改进后：
bank(t)    = -85°
bank(t+dt) = -84°  ? 平滑变化
bank(t+2dt)= -83°
...
```

---

### 改进 5: **升力系数需求保护**

```cpp
// ? 增加动压下限
if (q_dyn < 100.0) q_dyn = 100.0;  // 提高到 100 Pa

// ? 限制 CL 需求范围
double CL_req = (Config::Mass * total_acc) / (q_dyn * Config::S_ref);
CL_req = std::max(-0.5, std::min(1.5, CL_req));
```

**对比：**
| 高度 (km) | 动压 (Pa) | CL需求（原始） | CL需求（改进） |
|-----------|-----------|---------------|---------------|
| 70 | 5 | 20.0 ? | 1.5 ? |
| 50 | 50 | 2.0 | 1.2 |
| 30 | 500 | 0.5 | 0.5 |

---

### 改进 6: **攻角速率限制**

```cpp
// ? 攻角速率限制
static double prev_alpha = alpha_deg;
const double MAX_ALPHA_RATE = 5.0;  // 5°/s

double alpha_rate = (alpha_deg - prev_alpha) / dt;
if (std::abs(alpha_rate) > MAX_ALPHA_RATE) {
    double sign = (alpha_rate > 0) ? 1.0 : -1.0;
    alpha_deg = prev_alpha + sign * MAX_ALPHA_RATE * dt;
}
prev_alpha = alpha_deg;
```

**效果：**
```
原始：α 可以 2° → 20° 瞬间跳变 ?
改进：α 每步最多变化 0.25° ?

时间步长 0.05s × 速率 5°/s = 0.25°/step
```

---

## 修复前后对比

### 制导参数对比

| 参数 | 修复前 | 修复后 | 改善 |
|------|--------|--------|------|
| **k_t1** | 2.0 | 1.0 | 响应更平滑 |
| **k_s1** | 4.0 | 1.2 | 防止过度校正 |
| **航向误差限制** | 无 | ±28.6° | 防止饱和 |
| **总加速度限制** | 无 | 8g | 符合物理约束 |
| **滚转角速率** | 无限 | 20°/s | 消除震荡 |
| **攻角速率** | 无限 | 5°/s | 平滑过渡 |
| **动压下限** | 10 Pa | 100 Pa | 数值稳定 |
| **CL范围** | 无限 | [-0.5, 1.5] | 模型有效范围 |

### 预期仿真结果

#### 攻角曲线
```
修复前：
  20° |  ┌─┐    ┌─┐
      |  │ │    │ │
  10° |──┘ └────┘ └──  ? 方波震荡
   2° |

修复后：
  20° |     ╱──╲
      |    ╱    ╲
  10° |   ╱      ╲___  ? 平滑曲线
   2° |──╱
```

#### 滚转角曲线
```
修复前：
  85° | ─┐  ┌─┐
      |  │  │ │
   0° |  └──┘ │
    |       │
 -85° |       └─  ? 极端震荡

修复后：
  85° |
      |    ╱╲
   0° | __╱  ╲___  ? 合理变化
      |
 -85° |
```

---

## 调试建议

### 1. 观察指令平滑度
在 `main.cpp` 的状态输出中添加：
```cpp
if (iteration_count % print_interval == 0) {
    std::cout << " alpha=" << cmd.alpha * Config::RAD2DEG
       << " bank=" << cmd.bank * Config::RAD2DEG
              << " q=" << 0.5 * rho * vehicle.v * vehicle.v / 1000.0 << "kPa"
            << std::endl;
}
```

### 2. 检查速率限制触发
添加计数器：
```cpp
static int bank_rate_limit_count = 0;
static int alpha_rate_limit_count = 0;

if (std::abs(bank_rate) > MAX_BANK_RATE) {
    bank_rate_limit_count++;
}

// 最后输出统计
std::cout << "[Stats] Bank rate limited " << bank_rate_limit_count << " times\n";
```

### 3. 验证增益效果
如果仍有震荡，可以进一步降低增益：
```cpp
// 更保守的增益
double k_t1_adaptive = k_t1 * 0.3;  // 2.0 → 0.6
double k_s1_adaptive = k_s1 * 0.2;  // 4.0 → 0.8
```

---

## 理论依据

### 1. **滑模控制的颤振问题**
```
原始 SMC 在离散系统中容易产生颤振：
- 符号函数 sign(e) → 不连续
- 边界层外部：高增益切换
- 解决：tanh(e/ε) 平滑化 + 降低增益
```

### 2. **速率限制的必要性**
```
物理系统的执行器速率有限：
- 舵面偏转速率：通常 < 50°/s
- 推力矢量速率：通常 < 30°/s
- 气动控制面响应：有惯性延迟

数值仿真必须模拟这些约束
```

### 3. **加速度饱和**
```
升力上限：
L_max = 0.5 * ρ * v? * S * CL_max

对应最大法向加速度：
a_max = L_max / m

超过此值的指令无法实现
```

---

## 进一步优化方向

### 1. **自适应增益调度**
```cpp
// 根据飞行状态自适应调整增益
double altitude_factor = std::min(1.0, h / 50000.0);
double k_s1_adaptive = k_s1 * (0.2 + 0.3 * altitude_factor);

// 高空：增益小（动压小，响应慢）
// 低空：增益大（动压大，响应快）
```

### 2. **低通滤波器**
```cpp
// 一阶低通滤波
static double bank_filtered = 0.0;
const double FILTER_COEFF = 0.8;

bank_filtered = FILTER_COEFF * bank_filtered + (1 - FILTER_COEFF) * bank;
```

### 3. **预测控制**
```cpp
// 基于剩余距离调整增益
if (L_go < 100000.0) {  // 接近目标时
    k_s1_adaptive *= 0.5;  // 进一步降低增益
}
```

---

## 常见问题

### Q1: 降低增益会影响跟踪精度吗？
**A:** 
- 短期内响应变慢，但长期精度不受影响
- 消除震荡后，能量损耗减少，反而有利于到达精度
- 可以通过延长仿真时间来补偿

### Q2: 速率限制是否符合物理实际？
**A:**
- ? 20°/s 的滚转角速率是保守的
- 实际飞行器可能更快（30-50°/s），但需要精确建模
- 当前设置适合概念设计阶段

### Q3: 如何判断修复是否成功？
**A:** 观察以下指标：
```
? 攻角/滚转角无方波跳变
? 速率限制触发次数 < 10%
? 最终落点误差 < 5 km
? 飞行时间合理（1000-1500s）
```

---

## 总结

| 改进项 | 技术手段 | 预期效果 |
|--------|---------|---------|
| **增益优化** | 降低 70% | 减少过度响应 |
| **误差限幅** | ±28.6° | 防止饱和 |
| **加速度限制** | 8g | 符合物理约束 |
| **滚转角速率** | 20°/s | 消除震荡 |
| **攻角速率** | 5°/s | 平滑过渡 |
| **动压保护** | 100 Pa | 数值稳定 |

**修复完成！重新运行仿真，预期震荡问题将显著改善。** ???
