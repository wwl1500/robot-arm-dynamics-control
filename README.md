# robot-arm-dynamics-control

基于 ROS2-control 的机械臂关节空间动力学控制与任务空间阻抗控制。

## 当前阶段

第一步：2-DOF 平面机械臂运动学。

当前只实现 2-DOF 平面机械臂的正运动学和雅可比矩阵，尚未实现动力学、控制器、ROS2-control 插件或仿真。

## 模型

机械臂由两根平面连杆组成：

- `l1`：第一连杆长度
- `l2`：第二连杆长度
- `q1`：第一关节角
- `q2`：第二关节相对角

所有角度均使用弧度。

## 正运动学

末端位置为：

```text
x = l1*cos(q1) + l2*cos(q1 + q2)
y = l1*sin(q1) + l2*sin(q1 + q2)
```

## 雅可比矩阵

```text
J(q) = [ -l1*sin(q1)-l2*sin(q1+q2)   -l2*sin(q1+q2) ]
       [  l1*cos(q1)+l2*cos(q1+q2)    l2*cos(q1+q2) ]
```

该矩阵满足：

```text
xdot = J(q) * qdot
```

## 项目结构

```text
.
├── CMakeLists.txt
├── include/
│   ├── arm_kinematics.hpp
│   └── common_types.hpp
└── src/
    ├── arm_kinematics.cpp
    └── main.cpp
```

## 构建与运行

需要系统已安装 CMake、C++17 编译器和 Eigen3。

```bash
cmake -S . -B build
cmake --build build
./build/kinematics_demo
```

## 验证内容

`kinematics_demo` 会检查：

- `q = [0, 0]` 时的正运动学和雅可比矩阵
- `q = [pi/2, 0]` 时的正运动学和雅可比矩阵
- `q = [0, pi/2]` 时的正运动学和雅可比矩阵
- 解析雅可比矩阵与中心差分数值雅可比的一致性

全部通过时会输出：

```text
All kinematics checks passed.
```
