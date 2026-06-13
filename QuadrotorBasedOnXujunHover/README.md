# Quadcopter

本项目参考了北理工徐军的《多旋翼飞行器设计与控制》，实现了悬停状态下的四旋翼无人机建模，并设计了控制器。

## 环境要求

- MATLAB R2019b/R2020b
- Simulink

## 文件说明

| 文件名 | 类型 | 作用 |
|--------|------|------|
| SmallPerturbationLinearizedModelHover.slx | Simulink模型 | 1.在悬停状态下，建立四旋翼无人机的动力学和运动学模型<br>2.设计了位置控制器、姿态控制器、姿态角速度控制器<br>3.预留了扫频接口 |
| QuadcopterControlHover.slx | Simulink模型 | 1.建立四旋翼无人机的动力学和运动学模型<br>2.设计了位置控制器、姿态控制器、姿态角速度控制器 |
| PlantDynamics.m | S-Function函数 | 建立了无人机全量模型 |
| SmallPerturbationLinearizedModel.m | S-Function函数 | 建立了无人机悬停模式下小扰动线性化模型 |
| ControlAllocator.m | S-Function函数 | 进行电机的动力分配 |
| VelBodyToNED.m | S-Function函数 | 速度坐标系转NED导航坐标系 |
| DifferentialEquation.m | MATLAB脚本 | 控制律让误差按照差分方程的解收敛 |
| TrimHover.m | MATLAB脚本 | 配平脚本 |

