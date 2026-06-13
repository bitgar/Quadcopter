# Quadcopter

本项目参考了北航全权的《多旋翼飞行器设计与控制》，实现了悬停状态下的四旋翼无人机建模，并设计了控制器。

## 环境要求

- MATLAB R2019b/R2020b
- Simulink

## 文件说明

| 文件名 | 类型 | 作用 |
|--------|------|------|
| QuadcopterHover.slx | Simulink模型 | 1.建立四旋翼无人机的动力学和运动学模型<br>2.设计了位置控制器、姿态控制器、姿态角速度控制器<br>3.扫频北向位置指令—北向位置响应的传递函数，用于模型辨识<br>4.ident建模传递函数，分母4阶，分子1阶 |
| DifferentialEquation.m | MATLAB脚本 | 控制律让误差按照差分方程的解收敛 |
| PlantDynamics.m | S-Function函数 | 建立了无人机动力学模型 |
| TiltAttitudeAndForceGenerated.m | S-Function函数 | 根据控制律，生成期望的滚转角、俯仰角和拉力指令 |
