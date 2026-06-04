# 第 4 章:Sophus 与李群李代数

《视觉SLAM十四讲》第 4 章练习 —— 李群 $SO(3)/SE(3)$ 与李代数 $\mathfrak{so}(3)/\mathfrak{se}(3)$、指数/对数映射、BCH 近似、**扰动模型**(后端优化求雅可比的基础)。

官方配套代码:[../../../reference/slambook2/ch4/](../../../reference/slambook2/ch4/)(`useSophus.cpp` + `example/`)。

## 练习代码(骨架,按 TODO 填充)

| 文件 | 内容 |
|---|---|
| [ex01_sophus_basics.cpp](ex01_sophus_basics.cpp) | Sophus 基础:SO3/SE3 构造、`exp`/`log`、`hat`/`vee`、**扰动模型更新位姿** |
| [ex02_trajectory_error.cpp](ex02_trajectory_error.cpp) | 评估轨迹误差 ATE:读 TUM 轨迹,用 SE(3) 李代数范数算 RMSE |
| [ex03_adjoint.cpp](ex03_adjoint.cpp) | 习题:数值验证伴随性质 $R\,\exp(p^\wedge)R^\top=\exp((Rp)^\wedge)$、左右扰动 |

## 笔记

| 文件 | 内容 |
|---|---|
| [ch4_lie_algebra.md](ch4_lie_algebra.md) | 4.1–4.2 推导主线:$\mathfrak{so}(3)$ 怎么来、矩阵指数、罗德里格斯、李括号、$\mathfrak{se}(3)$、指数映射作用 |
| [notes_perturbation_model.md](notes_perturbation_model.md) | 4.3 补充:BCH、导数 vs 扰动模型、左右扰动、伴随性质 |

> 理论题进 `*.md` 笔记,代码题进 `exNN_*.cpp`(沿用 ch3 约定)。

## 依赖:Sophus(已装,header-only)

本章练习依赖 **Sophus**(header-only)。本机已装到 `/opt/homebrew/include/sophus`,无需重复安装。

> ⚠️ 本机 Eigen 为 5.0.1,而 Sophus 的 `find_package(Eigen3 3.4.0)` 按「同主版本」判定会拒绝 Eigen5,
> 故**不走** `cmake --install` + `find_package(Sophus)`,改为 header-only 手动安装
> (Sophus 头与 Eigen5 已实测兼容)。CMake 端用 `find_path` 定位头 + 定义 `SOPHUS_USE_BASIC_LOGGING`(不依赖 fmt)。

换机器 / 重装时:

```bash
git clone --depth 1 https://github.com/strasdat/Sophus.git /tmp/Sophus
cp -R /tmp/Sophus/sophus /opt/homebrew/include/sophus
```

## 构建运行

```bash
# 在 perception/visual_slam/ 下统一构建
cd .. && cmake -B build && cmake --build build

# ex02 读取相对路径数据,需在 perception/visual_slam/ 目录下运行:
./build/ex01_sophus_basics
./build/ex02_trajectory_error
./build/ex03_adjoint
```

## 知识要点

- **为什么要李代数**:$SO(3)/SE(3)$ 是流形不是向量空间,不能直接做加法/求导 → 映射到李代数(切空间)里优化。
- **指数映射** $\exp(\phi^\wedge)$ 的闭式 = 罗德里格斯公式;$\mathfrak{so}(3)$ 的物理意义就是**轴角**。
- **$\mathfrak{se}(3)$** 的 $\rho$ 不是真实平移,$t=J\rho$($J$ 为左雅可比)。
- **扰动模型**(优化主用,避开 $J_l$):
  - 左扰动 $R\leftarrow\exp(\delta\phi^\wedge)R$,$\dfrac{\partial(Rp)}{\partial\delta\phi}=-(Rp)^\wedge$(世界系,高翔默认)
  - 右扰动 $R\leftarrow R\exp(\delta\phi^\wedge)$,$\dfrac{\partial(Rp)}{\partial\delta\phi}=-R\,p^\wedge$(体坐标系,**FAST-LIO/IESKF** 用,对应 ⊞/⊟)
- **伴随**:$R\exp(p^\wedge)R^\top=\exp((Rp)^\wedge)$,即 $\mathrm{Ad}_R=R$ —— 左右扰动互转的桥梁。
