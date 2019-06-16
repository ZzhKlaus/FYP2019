# FYP2019
A Visual RGB-D SLAM aplication in AGV with omni wheels



## Abstract

Nowadays, automated guided vehicles (AGVs) are capable of accomplishing complicated tasks in addition to simple and repetitive one. With the development of micro processors and different types of perception sensors, there are many solution plans according to requirements of cost, precision and speed. For this project, a kinect depth sensor is used for perception on a small AGV with omni wheels, to realize the function of building indoor map and localization itself, the technology of PID controller, kalman filter and Robot Operating System (ROS) as well as related packages are utilized. The AGV's motion analysis, odometry and mapping performance is discussed with comparably good precision. The validness of this project is proved in the end.
Keywords: AGV, kinect, visual RGB-D,SLAM, omni wheels, Navigation, ROS, Automation, RTAB-Map 

## 

## 电机管脚连接： Mega2560 对应.ino文件

| Mega2560连接 | ENA               | ENB   | M+(红色A) | M-(黑色B) | PWM  |
| ------------ | ----------------- | ----- | --------- | --------- | ---- |
| Motor1       | 20 白（接线颜色） | 52 蓝 | 26 紫     | 27 蓝     | 7 绿 |
| Motor2       | 19 白             | 50 紫 | 24 白     | 25 灰     | 6 黑 |
| Motor2       | 18 黄             | 48 灰 | 22 橙     | 23 红     | 5 黄 |

## 

## 电机管脚连接： UNO x3 对应.ino文件

| UNO连接 x3      | ENA              | ENB  | M+(红色A) | M-(黑色B) | PWM   |
| --------------- | ---------------- | ---- | --------- | --------- | ----- |
| Motor1 （UNO1） | 2 白（接线颜色） | 8 蓝 | 6 紫      | 7 蓝      | 9 绿  |
| Motor2（UNO2）  | 2 白             | 8 紫 | 6 白      | 7 灰      | 10 黑 |
| Motor2（UNO3）  | 2 黄             | 8 灰 | 6 橙      | 7 红      | 11 黄 |

## 

## 电机编码器管脚区分

| ENB （旋转时和ENA一起输出两种波形， 用于速度测量） | 白   |
| -------------------------------------------------- | ---- |
| ENA                                                | 黄   |
| VCC （编码器供电 +5V）                             | 蓝   |
| GND                                                | 绿   |
| M- (和M+一起控制电机正反转)                        | 黑   |
| M+                                                 | 红   |



因为编码器需要比较大的运算量，三个电机的编码器都连到一块MEGA2560上会受限，所以最后采用三块UNO来分别连接三个电机。

 

电机用的减速电机（若减速比1：37），即轮子旋转一圈，内部编码器旋转37圈。 

经过测试，轮子旋转一圈编码器速出约 53735个高电平。测速原理和代码参考report。

小车圆底盘直径约：24cm， 轮子直径约5.8cm。 

整个项目分为PC部分和小车部分， PC安装ubuntu16.04系统，里边主要装有ROS, 常用指令有：

- rosrun pkg_name node_name --args（运行node)
- rostopic list (显示ros内话题)
- rospack find package_name
- pwd （获取路径
- catkin_make (编译工作空间)
- catkin_create_pkg 包名 --args
- rqt_dep package_name (查看包依赖关系)
- rosstack find (查看元功能包)
- rosmsg show msg_name (查看消息)
- rostopic echo topic_name (输出topic在terminal上)
- rostopic type topic_name (查看topic的类型)
- rosrun rqt_plot rqt_plot data_name (第一个rqt_plot为包名，第二个为node名， 绘制数据为一维图像)
- 还有许多指令



## 推荐阅读书籍

- ROS机器人高效编程
- 视觉SLAM十四讲
- 概率机器人 Probabilistic Robotics



## 推荐博客 

- <视觉SLAM技术 http://www.sohu.com/a/214096352_468626>
- 视觉SLAM十四讲 <<https://www.bilibili.com/video/av19397094?from=search&seid=1672292231994114512>
- 实时SLAM的未来, 深度学习 vs SLAM <http://zhehua.info/2016/03/23/DeepLearning-VS-SLAM-ChineseVersion..html>
- 三轮全向移动机器人的控制方法  <https://blog.csdn.net/jyaxp/article/details/55050393>
- Ubuntu安装Kinect驱动（openni、NITE、Sensor）及遇到的问题 <https://blog.csdn.net/u013453604/article/details/48013959>
- Hooking Up a Kinect to Your Computer (Using Ubuntu) <https://www.instructables.com/id/Hooking-up-a-Kinect-to-your-Computer-Using-Ubuntu/>
- 编码器速度和方向检测，371电机方向与速度检测 <http://yfrobot.com/forum.php?mod=viewthread&tid=2411&highlight=%B1%E0%C2%EB>
- Real-Time Appearance-Based Mapping <http://introlab.github.io/rtabmap/>
- ros wiki <http://wiki.ros.org/>