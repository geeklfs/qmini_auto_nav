# Qmini 台架感知

这套程序跑在 Jetson Nano 上，用已经接好的初代 ZED 做定位、局部障碍和三维重建。开发板还没装到机器人上，所以它只给出建议速度，不驱动电机。

## 这颗摄像头实际能做什么

板上是 Stereolabs 初代 ZED（USB `2b03:f580`），不是 ZED Mini，也不是 ZED 2。SDK 是 3.5.6。

- 双目深度大约 0.4 m 到 6 m 比较可用，标称最远能到 20 m，远处噪声大，导航里不用。
- 可选分辨率：HD2K 15fps、HD1080 15/30、HD720 15/30/60、VGA（672x376）15/30/60/100。Nano 4GB 默认用 VGA 15fps、`PERFORMANCE` 深度。
- 视觉定位可用。这颗摄像头没有 IMU，快速转身、遮挡、白墙和暗光都会丢跟踪。
- 可以做网格重建，但 Nano 上只能用低分辨率、短距离。
- 没有物体识别。SDK 3.5 的检测模块只支持 ZED 2，而且 Nano 也跑不动。
- 地板检测没有重力辅助，只有地面是画面里的主平面时才可信。

## 编译

在 Jetson 上：

```bash
cd ~/code/qmini_auto_nav
mkdir -p build && cd build
cmake ..
make -j4
```

## 运行

先确认没有 ZED Explorer 或旧的 ROS 节点占着摄像头。

```bash
cd ~/code/qmini_auto_nav/build
./qmini_auto_nav probe
./qmini_auto_nav nav
./qmini_auto_nav map --seconds 60
```

`nav` 启动后，在电脑浏览器打开：

```text
http://172.16.31.154:8080
```

画面四块：左目、深度、机体坐标系下的局部障碍和转向、世界坐标系下最近约 10 秒的障碍。终端同步打印建议速度。`output/cmd_vel.csv` 是以后接到 Qmini 运动接口的数据，当前没有往电机发送。

建图模式退出时写 `output/mesh.obj`，可以用 MeshLab 打开。`output/obstacles_latest.ply` 是当前帧抽出来的障碍点。

常用参数：

```bash
./qmini_auto_nav nav --res hd720 --fps 15 --camera-height 0.30 --robot-radius 0.18
```

`--camera-height` 只在地板平面没检测到时使用。相机大致水平、光轴朝前时再设这个高度。
