# 终止条件修复说明

## 问题诊断

### 原始代码的问题

```cpp
// ? 问题 1: 变量初始化在循环外，且值过大
double static missle_distance = 1e6;  // 1,000,000 米 = 1000 km

// ? 问题 2: 终止条件单位不一致
if (altitude <= 27000.0 || missle_distance < 5) {  
    // altitude 是米，missle_distance 初始值是米，但判断值 5 看起来是想用 km
}

// ? 问题 3: 变量赋值在循环结束后（太晚了）
missle_distance = AStarPlanner::calcDist(end_pos, target) * Config::Re / 1000.0;
```

**核心问题：**
- `missle_distance` 在整个循环中一直是初始值 1,000,000 米
- 条件 `missle_distance < 5` 永远不会触发（1,000,000 不小于 5）
- 即使想用公里，1000 km 也不会小于 5 km

---

## 修复方案

### 关键改进

#### 1. **实时计算距离**
```cpp
// ? 在每次迭代中计算当前距离
Point cur_pos = { vehicle.lon, vehicle.lat };
double distance_to_target_km = AStarPlanner::calcDist(cur_pos, target) * Config::Re / 1000.0;
```

#### 2. **明确的终止条件常量**
```cpp
// ? 定义清晰的阈值
const double ALTITUDE_THRESHOLD_M = 27000.0;    // 27 km
const double DISTANCE_THRESHOLD_KM = 5.0;       // 5 km
```

#### 3. **统一的单位和逻辑**
```cpp
// ? 终止条件：高度（米）或距离（公里）
if (altitude <= ALTITUDE_THRESHOLD_M || distance_to_target_km < DISTANCE_THRESHOLD_KM) {
    // 详细的退出信息
    std::cout << "\n=== Termination Condition Reached ===" << std::endl;
    
    if (altitude <= ALTITUDE_THRESHOLD_M) {
        std::cout << "[Exit Reason] Altitude: " << altitude / 1000.0 << " km" << std::endl;
    }
    
    if (distance_to_target_km < DISTANCE_THRESHOLD_KM) {
        std::cout << "[Exit Reason] Distance: " << distance_to_target_km << " km" << std::endl;
    }
    
    break;
}
```

---

## 代码对比

### 修改前
```cpp
double static missle_distance = 1e6;  // 初始化为超大值

while (1) {
    iteration_count++;
    double altitude = vehicle.r - Config::Re;

    // ? missle_distance 一直是 1e6，条件永远不满足
    if (altitude <= 27000.0 || missle_distance < 5) {
    break;
    }
    
    // ... 仿真循环
}

// ? 循环结束后才计算（太晚了）
missle_distance = AStarPlanner::calcDist(end_pos, target) * Config::Re / 1000.0;
```

### 修改后
```cpp
const double ALTITUDE_THRESHOLD_M = 27000.0;
const double DISTANCE_THRESHOLD_KM = 5.0;

while (1) {
    iteration_count++;
    
    // ? 每次迭代都计算当前距离
    double altitude = vehicle.r - Config::Re;
    Point cur_pos = { vehicle.lon, vehicle.lat };
    double distance_to_target_km = AStarPlanner::calcDist(cur_pos, target) * Config::Re / 1000.0;

    // ? 使用实时计算的距离
    if (altitude <= ALTITUDE_THRESHOLD_M || distance_to_target_km < DISTANCE_THRESHOLD_KM) {
      // 详细的退出日志
        std::cout << "[Exit] Altitude: " << altitude / 1000.0 << " km" << std::endl;
    std::cout << "[Exit] Distance: " << distance_to_target_km << " km" << std::endl;
        break;
    }
    
    // ... 仿真循环
}

// ? 最终统计使用已经计算好的值
double final_miss_distance_km = AStarPlanner::calcDist(end_pos, target) * Config::Re / 1000.0;
std::cout << "[Final] Miss Distance: " << final_miss_distance_km << " km\n";
```

---

## 改进效果

### 1. **正确的终止逻辑**
- ? 实时监控距离
- ? 距离 < 5 km 时立即退出
- ? 高度 < 27 km 时也能退出（双重保险）

### 2. **更好的调试信息**
```
=== Termination Condition Reached ===
[Exit Reason] Distance threshold: 4.73 km < 5.00 km
[Exit] Time: 1234.5s
[Exit] Final Altitude: 35.2 km
[Exit] Final Distance to Target: 4.73 km
```

### 3. **性能优化**
```cpp
// ? 状态输出时复用距离计算结果
if (iteration_count % print_interval == 0) {
    std::cout << " DistT=" << distance_to_target_km << "km" << std::endl;
}
```

---

## 测试验证

### 预期行为

#### 场景 1：正常到达终点
```
t=1230.0s H=32.50km v=2500.0m/s ... DistT=12.3km
t=1240.0s H=31.20km v=2400.0m/s ... DistT=7.8km
t=1250.0s H=30.10km v=2300.0m/s ... DistT=4.5km

=== Termination Condition Reached ===
[Exit Reason] Distance threshold: 4.50 km < 5.00 km
[Exit] Time: 1252.5s
[Final] Miss Distance: 4.230 km
```

#### 场景 2：高度先到达
```
t=1100.0s H=28.50km v=2200.0m/s ... DistT=15.3km
t=1110.0s H=27.20km v=2100.0m/s ... DistT=12.1km
t=1120.0s H=26.80km v=2000.0m/s ... DistT=10.5km

=== Termination Condition Reached ===
[Exit Reason] Altitude threshold: 26.80 km <= 27.00 km
[Exit] Time: 1122.0s
[Final] Miss Distance: 10.120 km
```

---

## 进一步优化建议

### 1. 添加提前预警
```cpp
// 在接近目标时提示
if (distance_to_target_km < 20.0 && distance_to_target_km >= 10.0) {
    static bool warned_20km = false;
    if (!warned_20km) {
        std::cout << "[Info] Approaching target: " << distance_to_target_km << " km" << std::endl;
        warned_20km = true;
    }
}
```

### 2. 记录终止原因到 CSV
```cpp
// 在 DataLogger 中添加字段
struct TerminationInfo {
    double time;
    double altitude_km;
    double distance_km;
    std::string reason;
};
```

### 3. 调整阈值为命令行参数
```cpp
int main(int argc, char* argv[]) {
    double distance_threshold = (argc > 1) ? std::atof(argv[1]) : 5.0;
    // ...
}
```

---

## 常见问题

### Q: 为什么同时检查高度和距离？
**A:** 双重保险机制：
- **距离判据**：确保到达目标附近
- **高度判据**：防止过早进入 TAEM 阶段

### Q: 5 km 是否合理？
**A:** 取决于应用场景：
- **高精度任务**：建议 1-3 km
- **常规任务**：5 km 合理
- **快速验证**：可放宽到 10 km

### Q: 为什么删除 `static` 关键字？
**A:** 原代码的 `static` 无实际意义：
- 局部 `static` 变量在程序运行期间保持，但 `main` 只执行一次
- 改为循环内局部变量更符合逻辑

---

## 总结

| 方面 | 修改前 | 修改后 |
|------|--------|--------|
| **距离计算** | 仅在结束后 | 每次迭代实时 |
| **终止逻辑** | ? 无法触发 | ? 正常工作 |
| **单位一致性** | ? 混乱 | ? 明确标注 |
| **调试信息** | 简单 | 详细+原因 |
| **代码可读性** | 低 | 高（常量+注释） |

**修复完成！** 现在程序会在距离目标 5 km 时正确终止。??
