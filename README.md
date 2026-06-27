# robot-arm-dynamics-control

基于 ROS2-control 的机械臂关节空间动力学控制与任务空间阻抗控制。

## 当前阶段

第三步：2-DOF 平面机械臂运动学 + Pinocchio 刚体动力学 + 最小控制器接口。

当前已实现 2-DOF 平面机械臂的正运动学、雅可比矩阵、质量矩阵、非线性项、重力项、逆动力学和纯 C++ 控制器接口。尚未实现 ROS2-control 插件、仿真、轨迹规划、数据记录或绘图。

## 模型

机械臂由两根平面连杆组成：

- `l1`：第一连杆长度
- `l2`：第二连杆长度
- `q1`：第一关节角
- `q2`：第二关节相对角

所有角度均使用弧度。运动平面为 `XY`，两个关节均绕 `Z` 轴旋转。动力学模型默认将重力设为世界 `-Y` 方向，均匀连杆的质心位于各自连杆中点，绕质心 `Z` 轴转动惯量为 `I = m*l^2/12`。

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

## 动力学

Pinocchio 动力学模块提供：

- `M(q)`：质量矩阵
- `h(q, v)`：非线性项，包含 `C(q, v)v + g(q)`
- `g(q)`：重力项
- `tau = M(q)a + h(q, v)`：逆动力学力矩

## 控制器接口

控制器模块提供三个最小力矩控制接口：

- 关节空间 PD + 重力补偿：`tau = Kp(q_des - q) + Kd(v_des - v) + g(q)`
- 任务空间雅可比转置控制：`tau = J(q)^T * f + g(q)`，其中 `f = Kp(x_des - x) + Kd(xdot_des - xdot)`，`xdot = J(q)v`
- 计算力矩控制：`tau = inverseDynamics(q, v, qdd_cmd)`，其中 `qdd_cmd = qdd_des + Kd(v_des - v) + Kp(q_des - q)`

控制器只复用 `ArmKinematics` 和 `ArmDynamics` 的公开接口，不暴露 Pinocchio 内部模型。

## 项目结构

```text
.
├── CMakeLists.txt
├── include/
│   ├── arm_controller.hpp
│   ├── arm_dynamics.hpp
│   ├── arm_kinematics.hpp
│   └── common_types.hpp
└── src/
    ├── arm_controller.cpp
    ├── arm_dynamics.cpp
    ├── arm_kinematics.cpp
    ├── controller_demo.cpp
    ├── dynamics_demo.cpp
    └── main.cpp
```

## 构建与运行

需要系统已安装 CMake、C++17 编译器、Eigen3 和 Pinocchio。

```bash
cmake -S . -B build
cmake --build build
./build/kinematics_demo
./build/dynamics_demo
./build/controller_demo
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

`dynamics_demo` 会检查：

- 质量矩阵对称性
- 质量矩阵正定性
- 典型姿态下的解析重力项
- 逆动力学与 `M(q)a + h(q, v)` 的一致性
- 静止状态下逆动力学、非线性项和重力项的一致性

全部通过时会输出：

```text
All dynamics checks passed.
```

`controller_demo` 会检查：

- 关节空间 PD + 重力补偿控制律
- 零关节误差时输出重力补偿
- 任务空间雅可比转置控制律
- 计算力矩控制律
- 零跟踪误差、零期望加速度时计算力矩结果与动力学非线性项一致

全部通过时会输出：

```text
All controller checks passed.
```
