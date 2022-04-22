## 第11讲 IMU传感器

1.1 噪声较小的情况

![](./images/low.png)


陀螺仪的Allan方差图如下

![](./images/gyr_low.jpg)

加速度的 Allan 方差图如下

![](./images/acc_low.jpg)

1.2 噪声中等的情况

![](./images/mid.png)

陀螺仪的Allan方差图如下

![](./images/gyr_mid.jpg)

加速度的 Allan 方差图如下

![](./images/acc_mid.jpg)

1.3 噪声较大的情况

![](./images/high.png)

陀螺仪的Allan方差图如下

![](./images/gyr_high.jpg)

加速度的 Allan 方差图如下

![](./images/acc_high.jpg)

2 将 IMU 仿真代码中的欧拉积分替换成中值积分，代码如下

![](./images/3.png)

欧拉法仿真结果如下

![](./images/2.png)


中值法仿真结果如下

![](./images/1.png)