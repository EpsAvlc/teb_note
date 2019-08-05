# TEB中的g2o问题的设置

## g2o问题的表述形式 

参照论文[^1]给出非线性优化问题的格式：
$$
V^{*}(\mathbf{b})=\min _{\mathbf{b}} \sum_{k=1}^{n-1} \Delta T_{k}^{2}
$$
subject to 
$$
\begin{array}{l}{\mathbf{s}_{1}=\mathbf{s}_{s}, \quad \mathbf{s}_{n}=\mathbf{s}_{f}, \quad \Delta T_{k}>0} \\ {\mathbf{h}_{k}\left(\mathbf{s}_{k+1}, \mathbf{s}_{k}\right)=\mathbf{0}} \\ {r_{k}\left(\mathbf{s}_{k+1}, \mathbf{s}_{k}\right) \geq 0} \\ {\mathbf{o}_{k}\left(\mathbf{s}_{k}\right) \geq \mathbf{0}} \\ {\boldsymbol{v}_{k}\left(\mathbf{s}_{k+1}, \mathbf{s}_{k}, \Delta T_{k}\right) \geq \mathbf{0}, \quad(k=1,2, \ldots, n-1)} \\ {\boldsymbol{\alpha}_{k}\left(\mathbf{s}_{k+1}, \mathbf{s}_{k+1}, \mathbf{s}_{k}, \Delta T_{k+1}, \Delta T_{k}\right) \geq \mathbf{0}, \quad(k=2,3, \ldots, n-2)} \\ {\boldsymbol{\alpha}_{1}\left(\mathbf{s}_{2}, \mathbf{s}_{1}, \Delta T_{1}\right) \geq \mathbf{0}, \quad \boldsymbol{\alpha}_{n}\left(\mathbf{s}_{n}, \mathbf{s}_{n-1}, \Delta T_{n-1}\right) \geq \mathbf{0}}\end{array}
$$
其中，$\mathbf{b}=\left[\mathbf{s}_{1}, \Delta T_{1}, \mathbf{s}_{2}, \Delta T_{2}, \mathbf{s}_{3}, \ldots, \Delta T_{n-1}, \mathbf{s}_{n}\right]^{\top}$。

## 顶点

顶点分为两类：pose与timediff

### vetex_pose

* 定义于g2o_types/vetex_pose.h
* 优化变量的数据类型定义为pose_se2，定义于pose_se2.h，由三个分量组成：x,y,theta，优化维度为三维。
* 在Teb初始化的过程中，将起点的Pose与终点的Pose所在的顶点设为fixed，使得g20不对这两个Pose进行优化。
* 初始化某一个TEB时，其路线简单的由起点和终点的连线组成。随后在这条直线上均匀采样点作为待优化的顶点。采样的步长由cfg_->trajectory.min_samples 决定。而 timediff 顶点的初始值为步长除以 cfg_->robot.max_vel_x。每有一个pose顶点就产生一个time_diff顶点。time_diff顶点实际上是每两个Pose之间所需要的时间。

### vetex_timediff

* 定义于g20_types/vetex_timediff.h
* 优化变量的数据类型为double。
* 由上所述，其生成在Teb初始化的时候。具体在timed_elastic_band.h/.cpp中的TimedElasticBand::initTrajectoryToGoal中。

## 边

### EdgeObstacle

* 定义于g2o_types/edge_obstacle.h中

* 一元边，观测值维度为1，类型为Obstacle基类，连接VertexPose顶点

* 存储了某个障碍物的中心点的三维位置，形状与顶点的位置

* 根据机器人的轮廓模型计算当前Pose与某个障碍物的距离

* $$
  error = dist > min\_obstacle\_dist + \epsilon ? 0 : (min\_obstacle\_dist + \epsilon) - dist
  $$

* 信息矩阵为cfg_->optim.weight_obstacle * weight_multiplier





```
[^1]：Integrated online trajectory planning and optimization in distinctive topologies
```