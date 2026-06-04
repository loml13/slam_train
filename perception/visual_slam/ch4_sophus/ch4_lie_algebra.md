# ch4 李群李代数:从旋转矩阵的导数到指数映射

> 来源:高翔《视觉 SLAM 十四讲》第二版 第 4 讲
> 整理日期:2026-05-22
> 配套代码:`../ch3_eigen/ex02_rotation.cpp`

---

## 0. 这部分在解决什么

旋转矩阵 $R$ 带约束 $R^TR=I$,**不能随便加减**(两个旋转矩阵相加不是旋转矩阵)。
但优化需要"加增量" $x\leftarrow x+\Delta x$。矛盾。

这一章通过**求导**,引出李代数 $\mathfrak{so}(3)$ —— 一个**平直、可加减**的空间,
并用**指数映射**把它和旋转矩阵 $SO(3)$ 连起来,为后续优化旋转/位姿铺路。

---

## 1. 前置知识(速查)

| 符号 | 含义 |
|------|------|
| $\mathbf{n}$ | 单位向量(旋转轴),3 维向量 |
| $\boldsymbol{\phi}$ | 旋转向量 = $\theta\mathbf{n}$;物理上是角速度方向 |
| $\mathbf{n}^\wedge$ | 帽子算子:把向量升级成 **3×3 反对称矩阵**(叉乘矩阵) |
| $SO(3)$ | 特殊正交群 = 所有旋转矩阵的集合,**弯曲流形** |
| $\mathfrak{so}(3)$ | 李代数 = $SO(3)$ 在单位元处的切空间,**平直空间** |
| $SE(3)$ | 特殊欧氏群 = 所有 4×4 齐次变换矩阵的集合,**弯曲流形** |
| $\mathfrak{se}(3)$ | $SE(3)$ 的李代数 = 6 维向量空间 |

反对称矩阵:

$$
\mathbf{n}^\wedge=\begin{bmatrix}0&-n_3&n_2\\ n_3&0&-n_1\\ -n_2&n_1&0\end{bmatrix},
\qquad \mathbf{n}^\wedge\mathbf{v}=\mathbf{n}\times\mathbf{v}
$$

**$\mathbf{n}^\wedge$ 的几何作用**:对向量 $\mathbf{v}$
- 平行于轴的分量 → **清零**(`n^ n = n×n = 0`)
- 垂直于轴的分量 → 在垂直平面里**转 90°**,长度不变

即 $\mathbf{n}^\wedge$ = "矩阵版的虚数 $i$"(在垂直平面里就是 90° 旋转)。

**群的定义**:集合 $G$ + 运算 $\cdot$ 满足四公理:封闭、结合、单位元、逆元(不要求交换律)。

---

## 2. 推导主线:$\mathfrak{so}(3)$ 怎么冒出来的(式 4.8–4.10)

### Step 1 — 旋转矩阵的导数(式 4.8)

旋转矩阵任意时刻满足 $R(t)R(t)^T=I$。两边对 $t$ 求导:

$$
\dot{R}R^T+R\dot{R}^T=0\;\Rightarrow\;\dot{R}R^T=-(\dot{R}R^T)^T
$$

一个矩阵等于自己转置的相反数 → 它是**反对称矩阵**,记作 $\boldsymbol{\phi}^\wedge$:

$$
\boxed{\dot{R}(t)=\boldsymbol{\phi}(t)^\wedge R(t)}\tag{4.8}
$$

**结论**:旋转矩阵求导 = 左乘一个反对称矩阵 $\boldsymbol{\phi}^\wedge$。
反对称矩阵不是硬塞的,是约束 $R^TR=I$ **逼出来的**。$\boldsymbol{\phi}$ 物理上是角速度。

### Step 2 — 切空间与李代数(式 4.9)

取 $t_0=0$,$R(0)=I$。一阶泰勒展开:

$$
R(t)\approx I+\boldsymbol{\phi}(0)^\wedge t\tag{4.9}
$$

**含义**:在单位元 $I$ 附近,旋转矩阵 = 单位阵 + 反对称矩阵。
所有反对称矩阵 $\boldsymbol{\phi}^\wedge$ 张成的空间 = $SO(3)$ 在 $I$ 处的
**切空间(Tangent Space)= 李代数 $\mathfrak{so}(3)$**。

> 切空间直觉:地球表面是弯的,但脚下那块平地是它的局部近似。
> $SO(3)$ 是弯曲流形,$\mathfrak{so}(3)$ 是贴在 $I$ 处的平面。
> 弯空间不能加减,平的切空间可以 —— 这就是引入李代数的意义。

### Step 3 — 指数映射(式 4.10)

设角速度恒定 $\boldsymbol{\phi}(t)\equiv\boldsymbol{\phi}_0$,式 4.8 变成线性微分方程:

$$
\dot{R}(t)=\boldsymbol{\phi}_0^\wedge R(t)\qquad\text{对照标量版 }\dot x=ax
$$

初值 $R(0)=I$,解得:

$$
\boxed{R(t)=\exp(\boldsymbol{\phi}_0^\wedge t)}\tag{4.10}
$$

这就是**指数映射**:把李代数元素 $\boldsymbol{\phi}^\wedge$ 映成李群元素 $R$。

---

## 3. 矩阵指数详解

### 3.1 定义:级数配方喂给矩阵

普通指数的泰勒级数只用到加法/自乘/除标量 —— 矩阵都有,直接套用:

$$
\exp(A)=\sum_{k=0}^{\infty}\frac{A^k}{k!} = I+A+\frac{A^2}{2!}+\frac{A^3}{3!}+\cdots
$$

### 3.2 为什么 exp 能解微分方程

逐项求导后**自我复制**:

$$
\frac{d}{dt}\exp(\boldsymbol{\phi}^\wedge t)=\boldsymbol{\phi}^\wedge\exp(\boldsymbol{\phi}^\wedge t)
$$

正好满足方程 $\dot R=\boldsymbol{\phi}^\wedge R$。

### 3.3 几何直觉:无穷小步累积

$$
\exp(\boldsymbol{\phi}^\wedge t)=\lim_{N\to\infty}\Big(I+\tfrac{\boldsymbol{\phi}^\wedge t}{N}\Big)^N
$$

- 式 4.9 是"沿切线走一大步"(近似,偏离 $SO(3)$)
- 式 4.10 是"切成无穷小步,每步贴回 $SO(3)$"(精确)

---

## 4. 级数如何收拢成罗德里格斯公式

### 4.1 幂次循环 $(\mathbf{n}^\wedge)^3=-\mathbf{n}^\wedge$

几何解释:$\mathbf{n}^\wedge$ 在垂直平面里转 90°。
- $(\mathbf{n}^\wedge)^2$:转 180°
- $(\mathbf{n}^\wedge)^3$:转 270° = $-\mathbf{n}^\wedge$

(类比虚数:$i^3=-i$,$\mathbf{n}^\wedge$ 就是矩阵版的 $i$)

无穷多幂次只有两种本质形式:$\pm\mathbf{n}^\wedge$ 和 $\pm(\mathbf{n}^\wedge)^2$。

### 4.2 归并 → sin 与 1−cos

$$
\exp(\theta\mathbf{n}^\wedge)=I
+\underbrace{\Big(\theta-\tfrac{\theta^3}{3!}+\tfrac{\theta^5}{5!}-\cdots\Big)}_{=\,\sin\theta}\mathbf{n}^\wedge
+\underbrace{\Big(\tfrac{\theta^2}{2!}-\tfrac{\theta^4}{4!}+\cdots\Big)}_{=\,1-\cos\theta}(\mathbf{n}^\wedge)^2
$$

得到**罗德里格斯公式**:

$$
\boxed{R=\exp(\theta\mathbf{n}^\wedge)=I+\sin\theta\,\mathbf{n}^\wedge+(1-\cos\theta)(\mathbf{n}^\wedge)^2}
$$

或等价形式:

$$
R=\cos\theta\,I+(1-\cos\theta)\,\mathbf{n}\mathbf{n}^T+\sin\theta\,\mathbf{n}^\wedge
$$

---

## 5. 李括号:李代数的"乘法"

李代数 ≠ 单纯的向量空间。完整定义是**向量空间 + 李括号运算**。

### 5.1 定义

对矩阵李代数,李括号就是**换位子**:

$$
\boxed{[A,B] = AB - BA}
$$

### 5.2 几何意义:衡量非交换性

- $AB=BA$ ⇒ $[A,B]=0$ —— 没有非交换性
- $AB\ne BA$ ⇒ $[A,B]\ne 0$,差值告诉你"两个操作交换顺序差多少"

旋转就是经典的非交换例子:**先绕 x 转再绕 y ≠ 先 y 后 x**。李括号给这种顺序敏感性一个代数刻画。

### 5.3 $\mathfrak{so}(3)$ 的李括号 = 叉乘(关键!)

$$
\boxed{[\boldsymbol{\phi}_1^\wedge,\boldsymbol{\phi}_2^\wedge]=(\boldsymbol{\phi}_1\times\boldsymbol{\phi}_2)^\wedge}
$$

**$\mathfrak{so}(3)$ 的李括号 = 三维向量叉乘**。这就是"叉乘"和"旋转"在物理里总是粘一起的根源。

### 5.4 三条性质

| 性质 | 公式 |
|------|------|
| 双线性 | $[aA+bB,C]=a[A,C]+b[B,C]$ |
| 反对称 | $[A,B]=-[B,A]$ |
| 雅可比恒等式 | $[A,[B,C]]+[B,[C,A]]+[C,[A,B]]=0$ |

⚠️ **不满足结合律!** 雅可比恒等式取代了结合律的角色。

### 5.5 直观小实验

两个小转动,角度 $\epsilon$:

$$
R_y(\epsilon)R_x(\epsilon)-R_x(\epsilon)R_y(\epsilon)\approx \epsilon^2[\boldsymbol{\phi}_x^\wedge,\boldsymbol{\phi}_y^\wedge]=\epsilon^2\boldsymbol{\phi}_z^\wedge
$$

"先 x 后 y" 比 "先 y 后 x" **多冒出一个 $\epsilon^2$ 量级的 z 轴旋转** —— 陀螺仪进动的几何根源。

### 5.6 BCH 公式(对 SLAM 至关重要)

两个李群元素相乘对应的李代数:

$$
\exp(\boldsymbol{\phi}_1^\wedge)\exp(\boldsymbol{\phi}_2^\wedge)
=\exp\Big(\boldsymbol{\phi}_1^\wedge+\boldsymbol{\phi}_2^\wedge+\tfrac{1}{2}[\boldsymbol{\phi}_1^\wedge,\boldsymbol{\phi}_2^\wedge]+\cdots\Big)
$$

- 只有当 $[\boldsymbol{\phi}_1^\wedge,\boldsymbol{\phi}_2^\wedge]=0$(可交换),才简化为 $\exp(A+B)$
- 旋转不交换,所以**不能直接在李代数上做加法**,需要 BCH 修正项
- SLAM 里用其线性化形式(左/右雅可比),后续做扰动模型时会用到

---

## 6. $\mathfrak{se}(3)$ 与 $SE(3)$:推广到完整位姿

旋转只能描述"朝向",完整位姿还需要平移。对应的群是 $SE(3)$。

### 6.1 $\mathfrak{se}(3)$ 元素的结构

李代数 $\mathfrak{se}(3)$ 元素是一个 **6 维向量** $\boldsymbol{\xi}$:

$$
\boldsymbol{\xi}=\begin{bmatrix}\boldsymbol{\rho}\\ \boldsymbol{\phi}\end{bmatrix}\in\mathbb{R}^6,\qquad
\boldsymbol{\rho}\in\mathbb{R}^3\text{ (类平移)},\;\;\boldsymbol{\phi}\in\mathbb{R}^3\text{ (旋转)}
$$

> ⚠️ **$\boldsymbol{\rho}$ 不是真平移!** 真平移 $\mathbf{t}=\mathbf{J}\boldsymbol{\rho}$,差一个左雅可比 $\mathbf{J}$。这是 ch4 的经典陷阱。

对应的 ^ 形式是 **4×4 矩阵**:

$$
\boldsymbol{\xi}^\wedge=\begin{bmatrix}\boldsymbol{\phi}^\wedge & \boldsymbol{\rho}\\ \mathbf{0}^T & 0\end{bmatrix}\in\mathbb{R}^{4\times 4}
$$

### 6.2 $\mathfrak{se}(3)$ 的指数映射

$$
\boxed{\exp(\boldsymbol{\xi}^\wedge)=\begin{bmatrix}R & \mathbf{J}\boldsymbol{\rho}\\ \mathbf{0}^T & 1\end{bmatrix}=T}
$$

其中:
- $R=\exp(\boldsymbol{\phi}^\wedge)$ 还是罗德里格斯
- 真正的平移 $\mathbf{t}=\mathbf{J}\boldsymbol{\rho}$
- $\mathbf{J}$ 是**左雅可比矩阵**:

$$
\mathbf{J}=\frac{\sin\theta}{\theta}I+\Big(1-\frac{\sin\theta}{\theta}\Big)\mathbf{n}\mathbf{n}^T+\frac{1-\cos\theta}{\theta}\mathbf{n}^\wedge
$$

### 6.3 几何解读:螺旋运动(Chasles 定理)

> **任何刚体运动 = 一个螺旋**:绕某轴旋转 + 沿同一轴平移。

- $\boldsymbol{\xi}\in\mathfrak{se}(3)$ 编码一个螺旋:旋转轴 + 转速 + 螺距
- $\exp(\boldsymbol{\xi}^\wedge t)$ = 沿这个螺旋运动 $t$ 时间后的位姿
- 拧螺丝、走螺旋楼梯都是这种运动

### 6.4 $\mathfrak{se}(3)$ 的李括号

$$
[\boldsymbol{\xi}_1^\wedge,\boldsymbol{\xi}_2^\wedge]
=\begin{bmatrix}\boldsymbol{\phi}_1^\wedge & \boldsymbol{\rho}_1^\wedge\\ \mathbf{0} & \boldsymbol{\phi}_1^\wedge\end{bmatrix}
\begin{bmatrix}\boldsymbol{\phi}_2\\ \boldsymbol{\rho}_2\end{bmatrix}\text{ 的反对称化}
$$

(精确表达式繁琐,SLAM 里直接调库即可。Sophus 提供 `SE3::Adj()` 等工具。)

---

## 7. 指数映射的作用

### 7.1 作用 1:桥接平直空间和弯曲空间

| | 李群 $SO(3)/SE(3)$ | 李代数 $\mathfrak{so}(3)/\mathfrak{se}(3)$ |
|---|---|---|
| 形状 | 弯曲流形 | 平直向量空间 |
| 能否加减 | ❌ | ✅ |
| 能否求导 | 困难 | 简单 |

$\exp$ 把"在平的空间里算出的东西"还原成"在弯的空间里的姿态/位姿";$\log$ 反过来。

### 7.2 作用 2:SLAM 优化的入口(**最重要!**)

位姿优化时,**不能写 $T_{\text{new}}=T+\Delta T$**(SE(3) 不能加)。解决方案:

$$
\boxed{T_{\text{new}}=\exp(\Delta\boldsymbol{\xi}^\wedge)\cdot T}
$$

- $\Delta\boldsymbol{\xi}\in\mathbb{R}^6$ 是李代数里的增量,**可以自由优化**
- 在李代数里求梯度、解牛顿步 → 得到 $\Delta\boldsymbol{\xi}$
- $\exp$ 卷回 $SE(3)$,乘到原位姿

```
平直 se(3)  ─── exp ───►  弯曲 SE(3)
     ↑                       │
     │  在这里算梯度/加增量   │  在这里表达真实位姿
     │  Δξ ← Δξ + α·∇         │  T_new = exp(Δξ^) · T
     └──────── log ───────────┘
```

**ch6 之后所有位姿优化(BA、位姿图、滑窗、FAST-LIO 状态更新)的核心循环都是这个**。

### 7.3 作用 3:匀速运动的精确解

| 方程 | 精确解 |
|------|--------|
| $\dot R=\boldsymbol{\phi}^\wedge R$(角速度恒定) | $R(t)=\exp(\boldsymbol{\phi}^\wedge t)$ |
| $\dot T=\boldsymbol{\xi}^\wedge T$(旋量恒定) | $T(t)=\exp(\boldsymbol{\xi}^\wedge t)$ |

IMU 预积分、连续时间 SLAM 的数学基础。

### 7.4 对数映射(逆映射)

| 输入 | 输出 | 算法 |
|------|------|------|
| $R\in SO(3)$ | $\boldsymbol{\phi}\in\mathfrak{so}(3)$ | $\theta=\arccos\frac{\text{tr}(R)-1}{2}$,轴从 $R-R^T$ 提取 |
| $T\in SE(3)$ | $\boldsymbol{\xi}\in\mathfrak{se}(3)$ | 先求 $\boldsymbol{\phi}=\log(R)$,再 $\boldsymbol{\rho}=\mathbf{J}^{-1}\mathbf{t}$ |

$\exp$ 和 $\log$ 互逆(局部),让李群和李代数能**双向自由切换**。

---

## 8. 逻辑链总览

```
RRᵀ=I 求导 ──► 反对称矩阵 ──► φ^      ........... 式4.8
                                  │
                       原点一阶展开 │
                                  ▼
                  I + φ^t = 切空间 = 李代数 so(3) .. 式4.9
                                  │
                     解微分方程    │
                                  ▼
                  R = exp(φ^t)  指数映射 ........... 式4.10
                                  │
                 幂次循环 (n^)³=-n^│
                                  ▼
              R = I + sinθ·n^ + (1-cosθ)(n^)²
                  = 罗德里格斯公式
                                  │
                            推广位姿 │
                                  ▼
              ξ ∈ se(3) ──exp──► T ∈ SE(3)
                              = [R  Jρ; 0 1]
                                  │
                            李括号  │
                                  ▼
              [φ₁^,φ₂^] = (φ₁×φ₂)^ ── BCH ── 扰动模型 ── 优化
```

---

## 9. 关键公式速查

| 项 | $\mathfrak{so}(3)\leftrightarrow SO(3)$ | $\mathfrak{se}(3)\leftrightarrow SE(3)$ |
|----|---|---|
| 元素维度 | 3 | 6 |
| 元素形式 | $\boldsymbol{\phi}$ 或 $\boldsymbol{\phi}^\wedge$ (3×3 反对称) | $\boldsymbol{\xi}=[\boldsymbol{\rho};\boldsymbol{\phi}]$ 或 $\boldsymbol{\xi}^\wedge$ (4×4) |
| 指数映射 | 罗德里格斯公式 | $[R\;\mathbf{J}\boldsymbol{\rho};\,0\;1]$ |
| 对数映射 | $\theta=\arccos\frac{\text{tr}(R)-1}{2}$ | 先 $\log(R)$,再 $\mathbf{J}^{-1}\mathbf{t}$ |
| 李括号 | $(\boldsymbol{\phi}_1\times\boldsymbol{\phi}_2)^\wedge$ | 4×4 矩阵换位子 |
| 优化更新 | $R_{\text{new}}=\exp(\Delta\boldsymbol{\phi}^\wedge)R$ | $T_{\text{new}}=\exp(\Delta\boldsymbol{\xi}^\wedge)T$ |

**核心关系**:

$$
\exp(\boldsymbol{\phi}^\wedge) = R \quad\Leftrightarrow\quad \log(R)=\boldsymbol{\phi}^\wedge
$$

$$
[\boldsymbol{\phi}_1^\wedge,\boldsymbol{\phi}_2^\wedge]=(\boldsymbol{\phi}_1\times\boldsymbol{\phi}_2)^\wedge
$$

$$
\text{tr}(R)=1+2\cos\theta\quad\text{(从 R 反解角度)}
$$

---

## 10. 待动手验证(TODO)

- [x] `ex02_rotation`:罗德里格斯公式手算 vs Eigen
- [x] `ex03_gimbal_lock`:欧拉角的奇异
- [x] `ex04_exercises`:已完成
- [ ] 用 Eigen 累加矩阵指数级数前 3/6/10 项,观察收敛到旋转矩阵
- [ ] 验证 $[\boldsymbol{\phi}_1^\wedge,\boldsymbol{\phi}_2^\wedge]=(\boldsymbol{\phi}_1\times\boldsymbol{\phi}_2)^\wedge$:左右两边数值计算
- [ ] 装 Sophus 后,用 `Sophus::SE3d::exp/log` 验证 $\mathbf{J}\boldsymbol{\rho}=\mathbf{t}$
- [ ] 对照 Barfoot《State Estimation for Robotics》Ch7-8:同一套推导的严格版
- [ ] BCH 公式的左/右雅可比形式(高翔 4.3.3,扰动模型基础)
