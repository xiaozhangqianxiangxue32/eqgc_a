# 添加安全裕度检查 - 避免路径太接近禁飞区

## 问题描述

从可视化结果看，虽然从切点2直接到终点的几何路径不穿过任何禁飞区，但是**距离NFZ 5（右下角小禁飞区）太近了**。

### 问题场景

```
切点2 (55°, 2°)  ------------>  终点 (90°, 0°)
  ↓
    距离NFZ 5太近 ?
              ↓
             实际飞行可能偏离进入NFZ
```

### 为什么会有风险？

1. **动力学约束**：实际飞行器不能完美沿着几何路径飞行
2. **转弯半径**：需要提前转弯，可能进入禁飞区
3. **制导误差**：总会有跟踪误差
4. **安全裕度不足**：几何上刚好不碰撞，实际上不安全

## 解决方案

### 核心思想

**不仅检查路径是否被阻挡，还要检查路径是否距离禁飞区太近**

### 实现方法

#### 1. 新增函数：`isTooCloseToNFZ()`

```cpp
bool AStarPlanner::isTooCloseToNFZ(
    const Point& p1, 
    const Point& p2, 
    const std::vector<NoFlyZone>& nfzs, 
    int ignore_idx, 
    double safety_factor = 1.2  // 默认1.2倍安全裕度
)
```

**功能**：检查从p1到p2的路径是否距离任何禁飞区太近

**判断标准**：
- 路径到禁飞区中心的垂直距离 < `r_safe * safety_factor`
- 端点距离禁飞区中心 < `r_safe * safety_factor`

#### 2. 修改直连检查逻辑

**修改前**：
```cpp
if (!isBlockedGlobal(next_pos, target, nfzs, i)) {
    // 可以直连 → 创建终点节点
}
```

**修改后**：
```cpp
if (!isBlockedGlobal(next_pos, target, nfzs, i) && 
    !isTooCloseToNFZ(next_pos, target, nfzs, i, 1.2)) {
  // 可以安全直连 → 创建终点节点
}
```

### 安全裕度系数说明

#### `safety_factor = 1.2` 的含义

- 原始安全半径：`r_safe = radius * (1 + omega)` （omega默认0.03）
- 额外安全半径：`r_extra_safe = r_safe * 1.2`
- **总安全裕度约为 1.2 × 1.03 ≈ 1.236 倍原始半径**

#### 为什么选择1.2？

1. **足够的安全裕度**：
   - 20%的额外缓冲
   - 考虑动力学约束和制导误差

2. **不会过度保守**：
   - 太大会导致无法找到路径
   - 太小会有安全风险

3. **可调节**：
   - 可以根据实际需求调整
   - 在函数参数中可以传入不同值

## 预期效果

### 修改前的路径

```
起点 → 切点1 → 切点2 → 终点
   (直接连接，但太接近NFZ 5) ?
```

控制台输出：
```
[Planner] Tangent point can see target! Direct connection added.
```

### 修改后的路径

```
起点 → 切点1 → 切点2 → 切点3 (NFZ 5的切点) → 终点
         (更安全的路径) ?
```

控制台输出：
```
[Planner] Tangent point can SAFELY see target! Direct connection added.
```

或者如果不够安全：
```
(不输出直连消息，继续扩展切点)
```

## 技术细节

### Cross-Track Distance (CTD) 计算

路径到禁飞区中心的最小距离：

```
CTD = asin(sin(dist_1c) * sin(az_12 - az_1c))
```

其中：
- `dist_1c`: 起点到禁飞区中心的距离
- `az_12`: 起点到终点的方位角
- `az_1c`: 起点到禁飞区中心的方位角

### Along-Track Distance (ATD) 检查

确保垂足在线段内：

```
ATD = acos(cos(dist_1c) / cos(CTD))
```

只有当 `0 < ATD < total_dist` 时，垂足才在线段内。

### 端点检查

也检查路径的两个端点是否距离禁飞区太近：

```cpp
if (Utils::calcDistRad(p1, c) < r_extra_safe || 
    Utils::calcDistRad(p2, c) < r_extra_safe) {
    return true;  // 端点太接近
}
```

## 调试和验证

### 控制台输出变化

**修改前**（可能不安全）：
```
[Planner] Iter 3: Node at (34.235°, -4.16936°) ...
[Planner] Tangent point can see target! Direct connection added. Potential cost: 10130.8 km
[Planner] Iter 4: Node at (90°, 0°) ...

[Planner] Final Path (4 waypoints):
  WP0: (0°, 0°)
  WP1: (34.235°, -4.16936°)
  WP2: (55.4307°, 1.90832°)
  WP3: (90°, 0°)
```

**修改后**（更安全）：
```
[Planner] Iter 3: Node at (34.235°, -4.16936°) ...
(不会输出"can see target"，因为太接近NFZ 5)
[Planner] Iter 4: Node at (xx°, yy°) ... (NFZ 5的切点)
[Planner] Tangent point can SAFELY see target! Direct connection added. Potential cost: XXXX km

[Planner] Final Path (5 waypoints):
  WP0: (0°, 0°)
  WP1: (34.235°, -4.16936°)
  WP2: (55.4307°, 1.90832°)
  WP3: (xx°, yy°)  ← 新增：NFZ 5的切点
  WP4: (90°, 0°)
```

### MATLAB可视化验证

在 Figure 2 (地面轨迹图) 中：
- ? 红色虚线应该绕开NFZ 5更远
- ? 增加一个航迹点（NFZ 5的切点）
- ? 路径稍微变长，但更安全

## 参数调整指南

### 如果路径过于保守（绕行太远）

降低安全裕度系数：
```cpp
!isTooCloseToNFZ(next_pos, target, nfzs, i, 1.1)  // 从1.2降低到1.1
```

### 如果路径仍然太接近禁飞区

提高安全裕度系数：
```cpp
!isTooCloseToNFZ(next_pos, target, nfzs, i, 1.5)  // 从1.2提高到1.5
```

### 针对不同禁飞区使用不同裕度

可以修改函数，根据禁飞区的大小或重要性使用不同的安全系数：

```cpp
double safety_factor = (nfzs[i].radius > 某个阈值) ? 1.1 : 1.3;
```

## 代码质量改进

### 1. 函数复用

`isTooCloseToNFZ()` 使用了与 `isPathBlocked()` 相同的几何计算，但判断标准不同：
- `isPathBlocked()`: 距离 < r_safe → 阻挡
- `isTooCloseToNFZ()`: 距离 < r_safe * safety_factor → 太接近

### 2. 可扩展性

可以轻松添加其他安全检查：
- 航向角约束
- 速度约束
- 高度约束

### 3. 性能影响

- 额外计算：O(N) 对于每个候选路径
- N = 禁飞区数量（通常较小，如5个）
- 性能影响可忽略不计

## 总结

这次修改的核心是：**安全第一**

- **问题**：几何上可行的路径，实际飞行可能不安全
- **原因**：动力学约束、制导误差、转弯半径
- **解决**：添加安全裕度检查，确保路径距离禁飞区足够远
- **效果**：路径可能稍长，但更安全、更符合实际飞行需求

修改后的规划器会生成更加**鲁棒**和**可靠**的航迹，适合高超声速飞行器这种动力学约束强的场景。
