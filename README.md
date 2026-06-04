# robot-learning

机器人技术学习与练习仓库 —— 覆盖**感知、多模态融合、导航规划**三大方向。
个人学习用途,跟随经典教材与开源项目动手实践。

## 目录结构

```
robot-learning/
├── perception/        机器人感知
│   └── visual_slam/   视觉 SLAM(《视觉SLAM十四讲》练习)
├── fusion/            多模态融合(IMU/视觉/激光融合、滤波、标定)
├── planning/          导航规划(路径规划、避障、Nav2)
└── reference/         参考资料与第三方代码
    ├── slambook2/     《视觉SLAM十四讲》官方配套代码
    └── FAST_LIO/      紧耦合激光-惯性里程计(git 子模块)
```

## 克隆与子模块

`reference/FAST_LIO/` 是 git 子模块,**普通克隆后该目录为空**,需初始化:

```bash
# 克隆时连同子模块一起拉取
git clone --recurse-submodules <repo-url>

# 或克隆后再初始化
git submodule update --init reference/FAST_LIO

# 拉取子模块上游更新
git submodule update --remote reference/FAST_LIO
```

## 三大方向

| 方向 | 内容 | 状态 |
|---|---|---|
| **perception** 感知 | 视觉/激光 SLAM、特征提取、传感器模型 | 进行中(ch4) |
| **fusion** 融合 | 多传感器融合、卡尔曼/粒子滤波、外参标定 | 待开始 |
| **planning** 规划 | 全局/局部路径规划、避障、运动控制 | 待开始 |

## 环境

- macOS (Apple Silicon) / Ubuntu 22.04
- C++17,Eigen3;Python 3.10+
- 构建:CMake;编辑器:VSCode + clangd

各子方向的构建方式见其各自的 README。

## 致谢

本仓库的学习与实践参考了以下教材与开源项目,版权归原作者所有,仅作个人学习用途:

- **《视觉SLAM十四讲》**(高翔)—— 配套开源代码见 `reference/slambook2/`,原仓库 <https://github.com/gaoxiang12/slambook2>
- **《State Estimation for Robotics》**(Timothy D. Barfoot)—— 机器人状态估计经典教材,
  电子版见 `reference/barfoot_state_estimation.pdf`,由作者主页免费发布 <http://asrl.utias.utoronto.ca/~tdb/>
- **FAST-LIO**(HKU-MARS)—— 紧耦合激光-惯性里程计,见子模块 `reference/FAST_LIO/`,原仓库 <https://github.com/hku-mars/FAST_LIO>
