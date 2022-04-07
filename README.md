# Kinodynamic Path Planning

## Simulation Results

## Build an ego-graph of the linear modeled robot
> 这里相当于完成一个在控制空间内采样的过程,一共有27个不同的动作输入,因此会采样得到27个不同的位置.这里前向采样的过程可以采取积分的形式,因为本身的状态包括位置和速度,而输入是加速度,因此通过对输入积分以及联合初值便可以得到当前时刻的速度.但是我在这里是基于**状态转移矩阵**来完成当前状态的计算,状态转移方程如下:
    $$ s(t) = e^{A(t-t_0)}s(t_0) +  \displaystyle \int^{t}_{t_0}{e^{A(t-\tau)}Bu(\tau)d\tau}$$
通过应用$ A $是一个幂零矩阵可以简化状态转移矩阵的计算,整个零状态响应的积分可以使用梯形积分公式来简单得到,最终完成前向积分过程.

[Forward integration]()

## Select the best trajectory closest to the planning target
>这里相当于解决一个OBVP问题,前面forward integration得到的状态在这里作为OBVP的起始点,目标点则作为OBVP的终止点.这里计算的目标函数实际考虑的约束条件只包括了运动学模型,没有考虑障碍物约束,因此这里计算的目标函数本质相当于一种heursitic(non-holonomic-without-obstacles).

>最终代码基于的问题是final-state is partially-free的,只给定目标位置,目标点的速度不给定.对于partially-free的final state problem 可以对协态变量使用边界条件来确定在终止时刻的值,进而完成对问题的求解.最终问题可以转换为只关于T的函数,我们需要求取该目标函数的极小值,即可完成对optimal cost的求解.

[OBVP]()



