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

### 障碍物约束

在AddEdgeObstacle函数中，只将离某个Pose最近的最左边与最右边的两个Obstacle加入优化中。（因为优化路径不会使得路径相对于障碍物的位置关系发生改变）。同时，还设了一个阈值，凡是离该Pose距离低于某个距离的障碍物也一并加入考虑之中。

#### EdgeObstacle

* 定义于g2o_types/edge_obstacle.h中，当inflated=false时使用此障碍物边

* 一元边，观测值维度为1，测量值类型为Obstacle基类，连接VertexPose顶点

* 存储了某个障碍物的中心点的三维位置，形状与顶点的位置

* 根据机器人的轮廓模型计算当前Pose与某个障碍物的距离

* $$
  error = dist > min\_obstacle\_dist + \epsilon ? 0 : (min\_obstacle\_dist + \epsilon) - dist
  $$

* 信息矩阵为cfg_->optim.weight_obstacle * weight_multiplier

#### EdgeInflatedObstacle

* 定义于g2o_types/edge_obstacle.h中，当inflated=true时使用此障碍物边

* 一元边，观测值维度为2，类型为Obstacle基类，连接VertexPose顶点

* $$
  error[0] = dist > min\_obstacle\_dist + \epsilon ? 0 : (min\_obstacle\_dist + \epsilon) - dist \\
  error[1] = dist > inflation\_dist ? 0 : inflation\_dist - dist \\
  $$

* 信息矩阵为对角阵，(0,0) = weight_obstacle

### via_point约束

via_point是一类点，其规定了轨迹应当经过这些点，否则会产生相应的cost。via_point边会与原规划的路径中与其距离最近的Pose顶点相连。

#### EdgeViaPoint

* 定义于g2o_types/edge_via_point.h中
* 一元边，观测值维度为1，类型为Eigen::Vector2d*，连接VertexPose顶点
* 存储了某个via_point的位置。
* error为其连接的Pose顶点的位置到这个Viapoint的距离的模长。
* 信息矩阵为1x1的矩阵，其值为weight_viapoint

### 速度约束

#### EdgeVelocity

* 定义于g2o_types/edge_via_point.h中
* 三元边，观测值变量维度为2，类型为double,连接两个VetexPose与一个VertexTimeDiff
* 速度由两个VetexPose间的距离除以时间得到。角速度由两个VetexPose间的角度除以时间得到
* error有两项，分别是线速度与线速度线速度是否在设定好的区间内。
* 信息矩阵为3x3对角矩阵，(0,0) = weight_max_vel_x, (1,1) = weight_max_vel_y, (2,2) =weight_max_vel_theta（对于全向轮底盘来说）

###  加速度约束

####　EdgeAcceleration

* 定义于g2o_types/edge_acceleration.h中
* 五元边，观测值维度为2，类型为double，连接三个pose与两个timediff顶点
* 根据三个Pose与两个timediff做两次差分得到线加速度与角加速度。
* error有两项，分别是线加速度与角加速度是否在设定好的区间内。
* 信息矩阵为2x2对角矩阵（对于阿克曼底盘来说），(0,0) =weight_acc_lim_x, (1,1) = weight_acc_lim_theta,

### 时间最优约束

#### EdgeTimeOptimal

* 定义于g2o_types/edge_time_optimal.h中
* 一元边，观测值维度为１，数据类型为double，连接一个VertexTimeDiff
* error直接就是连接的VertexTimeDiff的dt本身
* 信息矩阵为1x1矩阵，其值为weight_optimaltime

### 动力学约束

#### EdgeKinematicsCarlike

* 定义于g2o_types/edge_time_optimal.h中
* 二元边，观测值维度为2，数据类型为double，连接两个VertexPose
* 误差由文章[^1]中的动力学约束提出。阿克曼底盘模型还增加了最小转弯半径的约束。





```
[^1]：Integrated online trajectory planning and optimization in distinctive topologies
```