# 代价函数优化：从最短路径到最小转向角

## 修改动机

### 原有问题
之前的代价函数使用**最短路径**作为优化目标：
```cpp
g_cost = 累积距离
h_cost = 到终点的直线距离
f_cost = g_cost + h_cost
```

### 为什么需要改变？

对于**高超声速飞行器**来说：
1. **转向消耗大**：
   - 每次转向需要滚转角建立
   - 消耗能量和时间
   - 影响精度

2. **路径长度次要**：
   - 速度很快（6000+ m/s）
   - 多飞几公里影响不大
 - 减少转向更重要

3. **实际飞行约束**：
   - 频繁转向导致轨迹震荡
   - 控制饱和风险
   - 降低跟踪性能

## 新的代价函数

### 核心思想
**最小化累积转向角，而不是最短路径**

### 代价组成

```cpp
// 主要代价：转向角
turn_angle = |新航向 - 当前航向|

// 次要代价：距离（权重0.1）
dist_cost = 距离 × 0.1

// 总代价
g_cost = 累积转向角 + 距离代价
f_cost = g_cost + h_cost
```

### 权重说明

#### 转向角代价 (权重 = 1.0)
- **单位**：弧度 (rad)
- **典型值**：0 ~ π (0° ~ 180°)
- **含义**：每次转向的角度

#### 距离代价 (权重 = 0.1)
- **单位**：弧度 (rad，球面距离)
- **典型值**：0 ~ 2.0 (对应地球表面约0~12000km)
- **作用**：打破平局，避免路径过长

**为什么是0.1？**
- 转向90° ≈ 1.57 rad
- 飞行1000km ≈ 0.157 rad × 0.1 = 0.0157
- **转向的代价是距离的100倍左右**

## 代码实现

### 1. 正常扩展切点

**修改前** (最短路径):
```cpp
double dist_segment = Utils::calcDistRad(current->pos, next_pos);
double new_g = current->g_cost + dist_segment;  // 只考虑距离
```

**修改后** (最小转向角):
```cpp
double dist_segment = Utils::calcDistRad(current->pos, next_pos);
double az_next = Utils::calcAzimuth(current->pos, next_pos);
double turn_angle = std::abs(Utils::normalizeAngle(az_next - current->incoming_az));

// 代价 = 转向角 + 距离×0.1
double turn_cost = turn_angle;
double dist_cost = dist_segment * 0.1;
double new_g = current->g_cost + turn_cost + dist_cost;
```

### 2. 直连终点

**修改前**:
```cpp
double final_g = current->g_cost + dist_final;
```

**修改后**:
```cpp
double az_to_target = Utils::calcAzimuth(current->pos, target);
double turn_angle = std::abs(Utils::normalizeAngle(az_to_target - current->incoming_az));
double turn_cost = turn_angle;
double dist_cost = dist_to_target * 0.1;
double final_g = current->g_cost + turn_cost + dist_cost;
```

### 3. 切点直连终点

需要计算两次转向：
1. 当前节点 → 切点
2. 切点 → 终点

```cpp
double az_to_tangent = Utils::calcAzimuth(current->pos, next_pos);
double az_tangent_to_target = Utils::calcAzimuth(next_pos, target);

// 第一次转向
double turn1 = std::abs(Utils::normalizeAngle(az_to_tangent - current->incoming_az));

// 第二次转向
double turn2 = std::abs(Utils::normalizeAngle(az_tangent_to_target - az_to_tangent));

// 总代价
double turn_cost_total = turn1 + turn2;
double dist_cost_total = (dist_to_tangent + dist_tangent_to_target) * 0.1;
double final_g = current->g_cost + turn_cost_total + dist_cost_total;
```

## 预期效果

### 路径特征变化

#### 修改前（最短路径）
```
特点：
- 路径最短
- 可能有多次急转弯
- 总距离最小

示例：
起点 ──┐
       └─> 切点1 ──┐
     └─> 切点2 ──> 终点
   (多次转向，但距离短)
```

#### 修改后（最小转向角）
```
特点：
- 转向次数少
- 转向角度小
- 路径可能稍长，但更平滑

示例：
起点 ────────> 切点1 ────────> 终点
    (少转向，路径平滑)
```

### 控制台输出变化

**修改前**:
```
[Planner] Start Spherical A* Search (Shortest Path Mode)...
[Planner] Direct path to target found! Total cost: 10130.8 km
```

**修改后**:
```
[Planner] Start Spherical A* Search (Minimum Turn Angle Mode)...
[Planner] Direct path to target found! Turn: 45.23°, Dist: 8500.5 km, Total cost: 0.9397
```

**解读**:
- `Turn: 45.23°` - 从当前方向到终点需要转向45.23°
- `Dist: 8500.5 km` - 直线距离8500.5公里
- `Total cost: 0.9397` - 总代价 = 0.79 rad (转向) + 0.15 (距离代价)

## 参数调整

### 距离权重调整

#### 如果路径过长（太注重转向）
增加距离权重：
```cpp
double dist_cost = dist_segment * 0.2;  // 从0.1增加到0.2
```

#### 如果转向仍然太多
降低距离权重：
```cpp
double dist_cost = dist_segment * 0.05;  // 从0.1降低到0.05
```

### 平衡建议

| 距离权重 | 路径特征 | 适用场景 |
|---------|----------|----------|
| 0.05 | 极少转向，路径可能很长 | 燃料充足，注重平稳 |
| 0.1 | **平衡** (推荐) | 一般场景 |
| 0.2 | 较多转向，路径较短 | 航程受限 |
| 0.5 | 接近最短路径 | 距离最重要 |

## 优势分析

### 1. 更符合动力学特性
- 减少滚转角变化
- 降低控制饱和风险
- 提高跟踪精度

### 2. 能量效率
- 转向消耗能量
- 平滑路径更节能
- 减少热防护压力

### 3. 任务适应性
- 可通过权重调整
- 适应不同任务需求
- 灵活性强

### 4. 计算复杂度
- 与原算法相同 O(N log N)
- 只是代价计算方式变化
- 性能影响可忽略

## 示例对比

### 场景：三个禁飞区

#### 最短路径模式
```
路径: 起点 → A → B → C → 终点
转向: 90° → 80° → 75° → 60° (累积 305°)
距离: 8000 km
```

#### 最小转向角模式
```
路径: 起点 → A → C → 终点
转向: 45° → 50° (累积 95°)
距离: 8500 km
```

**结论**:
- 路径长度增加 6.25%
- 转向角减少 68.8%
- 更适合高速飞行器

## 调试技巧

### 1. 查看每个节点的代价
```cpp
std::cout << "Node cost - Turn: " << turn_cost * RAD2DEG 
       << "°, Dist: " << dist_cost * 10 << " km" << std::endl;
```

### 2. 统计路径特征
```cpp
double total_turn = 0;
double total_dist = 0;
for (size_t i = 1; i < path.size(); ++i) {
  double az1 = Utils::calcAzimuth(path[i-1], path[i]);
 double az2 = (i < path.size()-1) ? Utils::calcAzimuth(path[i], path[i+1]) : az1;
    total_turn += std::abs(Utils::normalizeAngle(az2 - az1));
    total_dist += Utils::calcDistRad(path[i-1], path[i]);
}
std::cout << "Total turn: " << total_turn * RAD2DEG << "°" << std::endl;
std::cout << "Total dist: " << total_dist * Re / 1000 << " km" << std::endl;
```

## 总结

这次修改的核心是：**重新定义什么是"好"的路径**

- **之前**: 最短路径是最好的
- **现在**: 转向最少的路径是最好的

对于高超声速飞行器，这是更合理的选择，因为：
1. 速度很快，多飞一点距离影响小
2. 转向代价大，应该尽量减少
3. 平滑路径更容易跟踪和控制

修改后的规划器会生成**更适合实际飞行**的航迹！
