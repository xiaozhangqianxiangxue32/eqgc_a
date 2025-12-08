# 修复节点3到终点直连问题

## 问题诊断

从可视化结果看，航迹点3（约在经度65°，纬度5°）应该可以直接连接到终点（90°, 0°），但规划器仍然生成了节点4和5，造成不必要的绕行。

## 根本原因

代码中存在一个**致命错误**：

```cpp
#ifndef PI
#define PI 3.14/4  // ? 错误！
#endif
```

### 错误分析

- **错误值**: `PI = 3.14/4 ≈ 0.785`
- **正确值**: `PI = 3.14159265358979323846`
- **误差倍数**: 约为 **4倍**！

### 影响范围

这个错误影响了所有使用PI的计算：

#### 1. **角度归一化函数** `normalizeAngle()`
```cpp
double normalizeAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;  // ? 使用了错误的PI
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}
```

**后果**：
- 角度范围变成 `[-0.785, 0.785]` 而不是 `[-π, π]`
- 导致角度比较完全错误

#### 2. **碰撞检测函数** `isPathBlocked()`
```cpp
// 如果圆心在起点后方 (|angle| > 90°)
if (std::abs(normalizeAngle(az_12 - az_1c)) > PI / 2.0) {  // ? 使用错误的PI
    atd = -atd;
}
```

**后果**：
- 90° 的判断阈值变成了约 0.393 弧度（22.5°）
- 导致碰撞检测逻辑完全错误
- **节点3到终点的路径被错误地判定为"被阻挡"**

## 解决方案

### 修复PI定义

```cpp
#ifndef PI
#define PI 3.14159265358979323846  // ? 正确的值
#endif
```

### 为什么这能解决节点3直连问题？

修复PI值后：

1. **角度归一化正确**
   - 角度范围变为正确的 `[-π, π]`
   - 方位角计算准确

2. **碰撞检测准确**
   - Along-Track Distance (ATD) 判断正确
   - Cross-Track Distance (CTD) 计算准确
   - 能正确判断节点3到终点的路径**不被NFZ 5阻挡**

3. **直连检查生效**
   ```cpp
   if (!isBlockedGlobal(next_pos, target, nfzs, i)) {
       // 现在能正确识别节点3可以直连终点
     // 直接创建终点节点，不再经过节点4和5
   }
   ```

## 预期效果

### 修复前
```
路径: 起点 → 节点1 → 节点2 → 节点3 → 节点4 → 节点5 → 终点
        ↑_____________________↑
         实际可以直连但被误判为阻挡
```

### 修复后
```
路径: 起点 → 节点1 → 节点2 → 节点3 → 终点
          ↑_______↑
     正确识别为可直连
```

## 验证步骤

### 1. 重新编译运行
```sh
# Visual Studio 中编译并运行
EQGC_A.exe
```

### 2. 检查控制台输出

应该看到类似输出：
```
[Planner] Tangent point can see target! Direct connection added. Potential cost: XXXX km
[Planner] Direct path to target found! Total cost: XXXX km, Iterations: XX
[Config] Waypoints saved to: ... (4 points)  // 点数应该减少
```

### 3. MATLAB可视化
```matlab
plot_results_with_waypoints
```

**检查 Figure 2 (地面轨迹图)**:
- 红色虚线应该在节点3后直接指向终点
- 不应该再有节点4和5
- 路径应该明显变短

## 技术细节

### 碰撞检测逻辑

修复PI后，`isPathBlocked()` 中的关键判断会正确工作：

```cpp
// 垂足方向判断（圆心在航线的前方还是后方）
double angle_diff = normalizeAngle(az_12 - az_1c);

// 90° 判断现在正确
if (std::abs(angle_diff) > PI / 2.0) {  // 现在 PI/2.0 = 1.5708 (90°)
    atd = -atd;
}
```

### 为什么之前节点3会绕行？

1. **错误的PI值**: 导致 `PI/2.0 ≈ 0.393` (约22.5°)
2. **误判阻挡**: 节点3到终点的路径与NFZ 5的角度关系被错误计算
3. **生成不必要的切点**: 继续扩展到NFZ 5的切点（节点4）
4. **继续绕行**: 从节点4再扩展到终点附近（节点5）

修复后，碰撞检测能正确判断：
- 节点3到终点的大圆航线**不经过**NFZ 5的安全区域
- Cross-Track Distance 足够大
- Along-Track Distance 判断正确
- 返回 `false` (不阻挡) → 触发直连逻辑

## 其他可能受影响的地方

虽然这个修复主要针对规划器，但PI的错误可能也影响了：

### 1. **制导律** (`guidance.cpp`)
如果制导律中也使用了相同的PI定义，可能需要检查

### 2. **动力学模型** (`dynamics.cpp`)
角度计算的准确性可能受影响

### 建议检查
在这些文件开头添加：
```cpp
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
```

并使用 `M_PI` 而不是自定义的 `PI`。

## 代码质量改进建议

### 1. **统一使用标准库常量**
```cpp
#include <cmath>
// 使用 M_PI 而不是自定义 PI
```

### 2. **添加编译时检查**
```cpp
static_assert(M_PI > 3.14 && M_PI < 3.15, "PI value seems incorrect");
```

### 3. **添加单元测试**
```cpp
// 测试角度归一化
assert(std::abs(normalizeAngle(4.0 * M_PI) - 0.0) < 1e-9);
assert(std::abs(normalizeAngle(3.5 * M_PI) - (-0.5 * M_PI)) < 1e-9);
```

## 总结

这是一个典型的**定义错误**导致的系统性bug：
- **错误源头**: `PI = 3.14/4` 而不是 `3.14159...`
- **错误传播**: 影响所有角度计算和碰撞检测
- **表面症状**: 规划器生成不必要的绕行路径
- **根本解决**: 修复PI的定义

修复后，节点3应该能够正确识别可以直连终点，不再绕行到节点4和5。

---

**重要**: 如果修复后仍然有问题，请检查：
1. `config.h` 中是否也定义了错误的PI
2. 其他头文件是否重定义了PI
3. 禁飞区的安全系数 `omega` 是否过大（当前为0.03）
