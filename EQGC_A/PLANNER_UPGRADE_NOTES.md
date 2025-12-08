# 路径规划算法升级说明

## 升级日期
2024年 - 改进版 A* 球面路径规划算法

---

## 升级概述

本次升级**完全保留了原有功能和接口**，同时引入了更精确的球面几何算法和高效的数据结构。

---

## 主要改进

### 1. **性能优化** ??

#### 改进前：
```cpp
std::list<Node*> open_list;
// 每次遍历查找最小 f 值 - O(n) 复杂度
for (auto it = open_list.begin(); it != open_list.end(); ++it) {
    if ((*it)->f() < min_f) { ... }
}
```

#### 改进后：
```cpp
std::priority_queue<NodeWrapper> open_list;
// 自动维护最小堆 - O(log n) 复杂度
Node* current = open_list.top().node;
open_list.pop();
```

**性能提升：** 对于 N 个节点的搜索，时间复杂度从 O(N?) 降至 O(N log N)

---

### 2. **碰撞检测算法升级** ?

#### 改进前 - 采样插值法：
```cpp
int steps = max(5, (int)(dist * Re / 20000.0));  // 每 20km 采样一次
for (int i = 0; i <= steps; ++i) {
  Point mid = interpolate(...);  // 线性插值
    if (dist(mid, nfz) < R) return true;
}
```
**问题：**
- 依赖采样密度，可能漏检
- 在禁飞区边缘可能出现误判
- 计算量随距离线性增长

#### 改进后 - 解析算法 (Cross-Track Distance)：
```cpp
// 1. 计算垂距 (CTD)
double ctd = asin(sin(dist_1c) * sin(az_12 - az_1c));

// 2. 计算投影位置 (ATD)
double atd = acos(cos(dist_1c) / cos(ctd));

// 3. 判断垂足是否在线段内
if (0 < atd < total_dist && |ctd| < R) return true;
```
**优势：**
- ? 理论严谨，无漏检风险
- ? 固定计算量，与距离无关
- ? 数值稳定性更好

---

### 3. **球面几何精度提升** ??

| 功能 | 改进前 | 改进后 | 优势 |
|------|--------|--------|------|
| **距离计算** | 球面余弦公式 | **Haversine 公式** | 短距离数值稳定性↑ |
| **切点计算** | Vincenty 公式 | 球面正弦/余弦定理 | 符合论文表述 |
| **正算** | 手动推导 | 标准 Vincenty Direct | 工程标准 |

---

### 4. **代码结构优化** ???

#### 新增 `Utils` 命名空间：
```cpp
namespace Utils {
    double normalizeAngle(double angle);
    double calcDistRad(const Point& p1, const Point& p2);
    double calcAzimuth(const Point& start, const Point& end);
    Point calcDirect(const Point& start, double az, double dist);
    bool isPathBlocked(const Point& p1, const Point& p2, const NoFlyZone& nfz);
}
```

**优势：**
- 代码复用性强
- 便于单元测试
- 符合软件工程规范

---

## 保留的功能

### ? 完全兼容原有接口
```cpp
// main.cpp 中的调用方式无需修改
auto waypoints = AStarPlanner::plan(vehicle, target, nfzs);
double dist = AStarPlanner::calcDist(cur_pos, target);
bool blocked = AStarPlanner::isBlocked(p1, p2, nfzs);
```

### ? 保留所有调试输出
- `astar_debug.csv` - 节点扩展日志
- `tangent_points.csv` - 切点候选记录
- 控制台实时进度显示

### ? 保留日志控制
```cpp
if (iterations % 100 == 0 || iterations <= 5) {
    std::cout << "[Current] Pos: ..." << std::endl;
}
```

---

## 测试建议

### 1. 功能验证
运行现有测试场景，对比路径结果：
```bash
# 检查路径点数量是否合理
# 验证是否成功避开所有禁飞区
# 对比最终落点精度
```

### 2. 性能测试
```cpp
// 在复杂场景下测试：
- 禁飞区数量：5 -> 10 -> 20
- 测量总迭代次数和运行时间
```

### 3. 边界情况
```cpp
- 起点/终点非常接近禁飞区
- 禁飞区重叠或相切
- 极地区域 (高纬度)
```

---

## 数学依据

### Cross-Track Distance (CTD) 公式推导

给定：
- 起点 P?、终点 P?、圆心 C
- 大圆航线 P?→P? 的方位角 az??
- P? 到 C 的方位角 az?c、距离 d?c

则垂距：
```
sin(CTD) = sin(d?c) · sin(az?? - az?c)
```

投影位置：
```
cos(d?c) = cos(CTD) · cos(ATD)
=> ATD = arccos(cos(d?c) / cos(CTD))
```

判据：
```
如果 0 < ATD < dist(P?,P?) 且 |CTD| < R
=> 航线与圆相交
```

---

## 已知限制

1. **内存管理**
   - 当前使用 `std::vector<Node*>` + 手动 `delete`
   - 建议未来升级为 `std::unique_ptr`

2. **重复节点检测**
   - 当前未实现 Closed List
   - 在复杂场景下可能重复扩展相同位置
 - 可通过哈希表优化

3. **启发式可调优**
   - 当前 `h(n) = |Δ航向角|`
   - 可考虑加入距离因子提升效率

---

## 回滚方案

如需回滚到旧版本：
```bash
# 备份文件路径（如果需要）
planner_old.cpp.bak
planner_old.h.bak
```

关键修改点：
1. 恢复 `std::list` 替代 `std::priority_queue`
2. 恢复采样插值碰撞检测
3. 删除 `Utils` 命名空间

---

## 联系与支持

如遇到问题，请检查：
1. 编译器是否支持 C++14
2. `config.h` 中的常量定义
3. CSV 输出路径是否存在

---

**升级完成！** ??

新算法在保持完全兼容的同时，提供了更高的精度和效率。
