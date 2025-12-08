# 主程序代码清理和滚转角限制说明

## 修复日期
2024年 - 代码清理 + 滚转角限制实现

---

## 问题诊断

### 原代码存在的问题

#### 1. **重复和冲突的代码块**
```cpp
// ? 问题：重复的变量声明
vehicle.v = 6800.0;   // 第一次
vehicle.v = 6800.0;   // 第二次重复

// ? 问题：重复的 Point 定义
Point cur_pos = { vehicle.lon, vehicle.lat };  // 第一次
// ...
Point cur_pos = { vehicle.lon, vehicle.lat };  // 第二次（在同一作用域）

// ? 问题：重复的 end_pos 定义
Point end_pos = { vehicle.lon, vehicle.lat };  // 第一次
// ...
Point end_pos = { vehicle.lon, vehicle.lat };  // 第二次
```

#### 2. **冲突的终止条件逻辑**
```cpp
// ? 旧的终止条件（已废弃）
if (altitude <= 27000.0 || missle_distance < 5) {
  std::cout << "[Exit] Reached TAEM Altitude (30km): ...";
    // ...
    
    // ? 新的终止条件（嵌套在旧的内部，逻辑混乱）
    if (altitude <= ALTITUDE_THRESHOLD_M || distance_to_target_km < DISTANCE_THRESHOLD_KM) {
     // ...
    }
}
```

#### 3. **未使用的静态变量**
```cpp
double static missle_distance = 1e6;  // ? 声明但从未有效使用
```

#### 4. **重复的输出**
```cpp
// ? 两次输出距离到目标
<< " DistT=" << dist_target / 1000.0 << "km"
<< " DistT=" << distance_to_target_km << "km"  // 重复！
```

#### 5. **缺少滚转角限制**
虽然需求提到 ±85° 限制，但原代码没有实现。

---

## 修复方案

### 1. **清理重复代码**

#### 变量声明
```cpp
// ? 修复后：只保留一次
vehicle.v = 6800.0;  // V0 = 6800 m/s
```

#### 位置计算
```cpp
// ? 在循环开始时统一计算，复用整个循环
Point cur_pos = { vehicle.lon, vehicle.lat };
double distance_to_target_km = AStarPlanner::calcDist(cur_pos, target) * Config::Re / 1000.0;
```

### 2. **统一终止条件逻辑**

#### 删除旧逻辑
```cpp
// ? 完全删除
// if (altitude <= 27000.0 || missle_distance < 5) { ... }
// double static missle_distance = 1e6;
```

#### 使用清晰的新逻辑
```cpp
// ? 唯一的终止条件检查
const double ALTITUDE_THRESHOLD_M = 27000.0;
const double DISTANCE_THRESHOLD_KM = 5.0;

if (altitude <= ALTITUDE_THRESHOLD_M || distance_to_target_km < DISTANCE_THRESHOLD_KM) {
    // 详细的退出信息
    std::cout << "\n=== Termination Condition Reached ===" << std::endl;
    
    if (altitude <= ALTITUDE_THRESHOLD_M) {
        std::cout << "[Exit Reason] Altitude threshold: " << ... << std::endl;
    }
    
    if (distance_to_target_km < DISTANCE_THRESHOLD_KM) {
        std::cout << "[Exit Reason] Distance threshold: " << ... << std::endl;
    }
    
    break;
}
```

### 3. **实现滚转角限制 ±85°**

#### 定义限制常量
```cpp
// 滚转角限制（根据需求）
const double BANK_ANGLE_MAX_DEG = 85.0;         // ±85°
const double BANK_ANGLE_MAX_RAD = BANK_ANGLE_MAX_DEG * Config::DEG2RAD;
```

#### 限幅处理
```cpp
// 制导指令生成后立即限幅
auto cmd = guidance.update(vehicle, guidance_target);

// === 滚转角限制：±85° ===
if (cmd.bank > BANK_ANGLE_MAX_RAD) {
  cmd.bank = BANK_ANGLE_MAX_RAD;
} else if (cmd.bank < -BANK_ANGLE_MAX_RAD) {
    cmd.bank = -BANK_ANGLE_MAX_RAD;
}

// 可选：检测是否触及限制（调试用）
if (std::abs(cmd.bank * Config::RAD2DEG) > BANK_ANGLE_MAX_DEG - 1.0) {
    if (iteration_count % 20 == 0) { // 避免过多输出
        std::cout << "[Warning] Bank angle near limit: " 
            << cmd.bank * Config::RAD2DEG << "° at t=" << vehicle.t << "s\n";
    }
}
```

### 4. **改进输出信息**

#### 启动时显示配置
```cpp
std::cout << "[Debug] Target: (" << target.lat * Config::RAD2DEG << "° Lat, "
   << target.lon * Config::RAD2DEG << "° Lon)" << std::endl;
std::cout << "[Debug] Termination Conditions:\n";
std::cout << "     - Altitude <= 27 km\n";
std::cout << " - Distance to Target < 5 km\n";
std::cout << "[Debug] Bank Angle Limit: ±85°\n";  // ← 新增
```

#### 运行时显示滚转角
```cpp
// 在状态输出中添加滚转角信息
std::cout << " bank=" << std::setprecision(1) << cmd.bank * Config::RAD2DEG << "°"
    << " WP=" << wp_idx << "/" << waypoints.size()
          << " DistT=" << distance_to_target_km << "km"
        << std::endl;
```

---

## 修复前后对比

### 代码结构

| 方面 | 修复前 | 修复后 |
|------|--------|--------|
| **重复代码** | 多处重复声明和计算 | ? 统一计算，复用结果 |
| **终止逻辑** | 两套逻辑嵌套，混乱 | ? 单一清晰的逻辑 |
| **滚转角限制** | ? 未实现 | ? ±85° 限制 + 警告 |
| **变量命名** | `missle_distance` (拼写错误) | ? 移除未使用变量 |
| **输出信息** | 重复、不完整 | ? 清晰、完整 |

### 运行时输出

#### 修复前
```
[Debug] Target: (0 Lat, 90 Lon)
[Debug] Time step: 0.05s
[Debug] Termination Altitude: 30 km
[Debug] Termination Conditions:
     - Altitude <= 27 km
   - Distance to Target < 5 km

t=10.0s H=69.50km v=6750.0m/s gamma=0.50° psi=89.50° WP=1/3 DistT=9800.5km DistT=9800.5km
  ^^^^^^^ 重复！
```

#### 修复后
```
[Debug] Target: (0° Lat, 90° Lon)
[Debug] Time step: 0.05s
[Debug] Termination Conditions:
     - Altitude <= 27 km
     - Distance to Target < 5 km
[Debug] Bank Angle Limit: ±85°

t=10.0s H=69.50km v=6750.0m/s gamma=0.50° psi=89.50° bank=12.3° WP=1/3 DistT=9800.5km
        ^^^^^^^^^^^^ 新增滚转角显示
```

---

## 滚转角限制的重要性

### 1. **物理约束**
```
- 再入飞行器的气动舵面/推力矢量有物理限制
- 过大的滚转角可能导致：
  * 失速
  * 控制饱和
  * 结构载荷超限
```

### 2. **数值稳定性**
```
- 限制滚转角可以防止制导算法发散
- 避免 atan2 在极端情况下的数值问题
- 确保仿真的鲁棒性
```

### 3. **实际应用**
```
- 工程实践中通常限制在 ±60° 到 ±85°
- 本实现使用 ±85° 是偏激进的设计
- 可根据实际飞行器能力调整
```

---

## 测试验证

### 1. 正常场景
运行仿真，观察：
```
- 滚转角是否始终在 ±85° 以内
- 终止条件是否正确触发
- 无重复输出
```

### 2. 极端机动场景
```cpp
// 可以测试急转弯情况
// 例如：目标在飞行器后方，需要大幅度转向
Point target = { -90.0 * Config::DEG2RAD, 0.0 * Config::DEG2RAD };
```

预期看到：
```
[Warning] Bank angle near limit: 84.5° at t=123.5s
[Warning] Bank angle near limit: -84.8° at t=134.2s
```

### 3. 边界条件
```
- 检查是否在距离 < 5km 时退出
- 检查是否在高度 < 27km 时退出
- 验证最终落点精度
```

---

## 代码质量改进

### 1. **消除魔法数字**
```cpp
// ? 使用命名常量
const double ALTITUDE_THRESHOLD_M = 27000.0;
const double DISTANCE_THRESHOLD_KM = 5.0;
const double BANK_ANGLE_MAX_DEG = 85.0;

// ? 避免硬编码
if (altitude <= 27000.0) { ... }
```

### 2. **统一单位标注**
```cpp
// ? 变量名清晰标注单位
double altitude  // 单位：米 (m)
double distance_to_target_km  // 单位：公里 (km)
double BANK_ANGLE_MAX_RAD     // 单位：弧度 (rad)
```

### 3. **减少重复计算**
```cpp
// ? 在循环开始时计算一次，整个循环复用
Point cur_pos = { vehicle.lon, vehicle.lat };
double distance_to_target_km = AStarPlanner::calcDist(cur_pos, target) * Config::Re / 1000.0;

// 在终止条件、航点切换、状态输出中都使用这个值
```

---

## 进一步优化建议

### 1. 可配置的限制参数
```cpp
struct FlightConstraints {
    double max_bank_angle_deg = 85.0;
    double max_alpha_deg = 20.0;
  double min_alpha_deg = 2.0;
  double max_q_dynamic = 90000.0;
};
```

### 2. 滚转角速率限制
```cpp
// 添加滚转角速率限制（deg/s）
const double BANK_RATE_MAX = 30.0 * Config::DEG2RAD;  // 30°/s

double bank_rate = (cmd.bank - prev_bank) / dt;
if (std::abs(bank_rate) > BANK_RATE_MAX) {
    // 限制速率
}
```

### 3. 记录限制触发统计
```cpp
struct LimitStats {
    int bank_limit_count = 0;
    int alpha_limit_count = 0;
  double max_bank_observed = 0.0;
};

// 在结束时输出
std::cout << "[Stats] Bank limit triggered " << stats.bank_limit_count << " times\n";
std::cout << "[Stats] Max bank angle: " << stats.max_bank_observed << "°\n";
```

---

## 常见问题

### Q1: 为什么选择 ±85° 而不是 ±90°？
**A:** 
- 90° 时 `cos(bank) = 0`，升力的垂直分量为零
- 接近 90° 时数值不稳定
- 85° 保留了一定的控制余量

### Q2: 滚转角限制会影响到达精度吗？
**A:** 
- 正常情况下不会，因为制导律设计合理
- 如果频繁触及限制，说明：
  * 路径规划过于激进
  * 制导参数需要调整
  * 飞行器机动能力不足

### Q3: 如何判断限制是否合理？
**A:** 观察仿真输出：
```
- 偶尔触及限制（<1%时间）：正常
- 频繁触及限制（>10%时间）：需要调整
- 持续饱和（>50%时间）：任务不可行
```

---

## 总结

| 修复项 | 状态 | 影响 |
|--------|------|------|
| **清理重复代码** | ? 完成 | 提高可读性 |
| **统一终止逻辑** | ? 完成 | 消除混乱 |
| **实现滚转角限制** | ? 完成 | 符合物理约束 |
| **改进输出信息** | ? 完成 | 便于调试 |
| **代码质量提升** | ? 完成 | 易于维护 |

**修复完成！代码现在更清晰、更健壮、更符合工程实践。** ??
