## 第16讲 VINS系统构建

1、程序修改
新建 simulation_test.cpp 文件,修改 PubImuData 如下

![](./images/4.png)

修改 PubImageData 如下

![](./images/5.png)

修改 System.cpp 中 PubImageData 如下

![](./images/6.png)

2、对不同大小噪声的 IMU 数据和相机数据仿真
无噪声

![](./images/%E6%97%A0%E5%99%AA%E5%A3%B0.png)

低噪声

![](./images/1.png)

![](./images/1e-7.png)

加大噪声

![](./images/2.png)

![](./images/1e-6.png)

继续加大噪声

![](./images/3.png)

![](./images/1e-5.png)

可以看出,噪声越大,效果越差