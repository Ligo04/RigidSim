# RigiSim

使用bevy实现基于XPBD的简单刚体模拟(CPU)

复现《Detailed Rigid Body Simulation with Extended Position Based Dynamics》， 论文解读可参考[Report](https://github.com/SZU-AdvTech-2022/197-Detailed-Rigid-Body-Simulation-with-Extended-Position-Based-Dynamics/blob/main/ResearchReport.pdf) 或者 [博客](https://www.cnblogs.com/Ligo-Z/p/16921559.html)。

**实现Positional Constraint , 基于Positional Constraints实现Distance Joint**

未实现Angular Constraints（模拟带角度的关节）以及Contact Constraints（需要碰撞检测）

## 运行说明

```powershell
cargo run --example rigid_sim_demo -- 1 1 1 1 1.5 0
```

参数：

1. `size_x`:  Cuboid在x轴的Size
2. `size_y:  Cuboid在y轴的Size
3. `size_x`:  Cuboid在z轴的Size
4. `chain_count`: 链条的数量，默认为1
5. `rest_length`:  `DistanceJoint`的`rest length`, 默认为1.5
6.  compliacne`： 柔性，默认为0（设置为零，模拟来模拟无限刚性约束
7.  `linear_vel_damping`: 线性速度阻尼，默认为零
8. `angular_vel_damping`: 角速度阻尼，默认为零

说明：

DistanceJoint的两个点设置在第一个刚体的右下角(`-0.5 * cuboid_size`)和第二个刚体的左上角(`0.5 * cuboid_size`)

通过鼠标点击可对刚体施加一个力（目前设定大小为100N,方向是`-Vec3::Z`）

## 算法流程

基于《Detailed Rigid Body Simulation with Extended Position Based Dynamics》与Smallsteps的思路提出的以下算法流程：

![image-20241104133413468](.\assets\image-20241104133413468.png)

### gyroscopic torque

- 实验中发现对于旋转使用半隐式方法进行更新会发散，结果不稳定，导致陀螺仪现象
- 使用gyroscopic torque会缓解上述问题，在[GDC 2015](https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf)上有提到。

gyroscopic 求解过程如下：

![image-20241104134729397](.\assets\image-20241104134729397.png)

![image-20241104134753220](D:\Ligo\Project\RigidSim\assets\image-20241104134753220.png)

### DistanceJoint

距离约束是最基本的约束形式，用于约束两个点之间的距离。如下图

![image-20241104141150126](.\assets\image-20241104141150126.png)

设约束距离为$l$, 约束方程为：$C=||(x_a + r_a) - (x_b+r_b)|||\ge0$ . 

根据推导，得到的约束梯度(Constraint Gradien）向量$\Delta C$方向是：$(x_a + r_a) - (x_b+r_b) / ||x_a + r_a - x_b+r_b||$, 约束梯度c的大小是$(x_a + r_a) - (x_b+r_b)-l$

根据的约束梯度(Constraint Gradien）向量$\Delta C$,  完成Positional Constraint就行模拟。

## 参考资料

- https://matthias-research.github.io/pages/publications/XPBD.pdf
- https://matthias-research.github.io/pages/tenMinutePhysics/09-xpbd.pdf
- https://matthias-research.github.io/pages/publications/PBDBodies.pdf
- https://matthias-research.github.io/pages/publications/smallsteps.pdf
- https://carmencincotti.com/2022-08-08/xpbd-extended-position-based-dynamics/
- XPBD论文解读(Extended Position-Based Dynamics) - zilch的文章 - 知乎
  https://zhuanlan.zhihu.com/p/406652407
- 2D 游戏物理引擎 - 关节约束 - ACRL的文章 - 知乎
  https://zhuanlan.zhihu.com/p/665405974
- https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf
- https://bevyengine.org/
- https://github.com/aevyrie/bevy_mod_picking
