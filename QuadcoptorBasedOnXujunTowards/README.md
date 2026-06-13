# Quadcopter

本项目参考了北理工徐军的《多旋翼飞行器设计与控制》，实现了前飞状态下的四旋翼无人机建模，并设计了控制器。

## 环境要求

- MATLAB R2019b/R2020b
- Simulink

## 文件说明

| 文件名 | 类型 | 作用 |
|--------|------|------|
| small_perturbation_linearized_model_towards.slx | Simulink模型 | 1.在前飞状态下，建立四旋翼无人机的动力学和运动学模型<br>2.设计了位置控制器、姿态控制器、姿态角速度控制器<br>3.预留了扫频接口 |
| SmallPerturbationLinearizedModelTowards.m | S-Function函数 | 建立了无人机前飞模式下小扰动线性化模型 |
| LateralDynamics.m | S-Function函数 | 建立了无人机前飞模式下横侧向模型 |
| ControlAllocatorTowards.m | S-Function函数 | 进行电机的动力分配 |
| TrimTowards.m | MATLAB脚本 | 配平脚本 |
| QuadcopterControlTowards.slx | Simulink模型 | 前飞状态下的全量模型和控制器（待完善） |

