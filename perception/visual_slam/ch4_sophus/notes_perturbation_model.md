# 第4讲 4.3 — BCH 与扰动模型(补充笔记)

> 本笔记接续同目录 [`ch4_lie_algebra.md`](ch4_lie_algebra.md)(4.1–4.2 推导主线:so(3) 怎么来、罗德里格斯、李括号、se(3)、指数映射作用),专补 **4.3 李代数求导与扰动模型** —— 旧笔记停在 4.2,未覆盖这部分。
>
> Obsidian vault(`10.Books/高翔SLAM系列/`)有完整 4.1–4.3 合订版,此处为 GitHub 存档。

> **为什么 4.3 是目的**:4.1–4.2 把工具(李代数、exp/log)搭好,4.3 才回答最初的问题 —— 求 $\dfrac{\partial(Rp)}{\partial R}$,为优化(BA、IESKF)铺路。

配套练习:同目录 `ex01_sophus_basics.cpp`(扰动模型更新)、`ex03_adjoint.cpp`(伴随性质)。

---

## 4.3.1 BCH 公式与近似

为什么需要它:$\exp(\phi_1^\wedge)\exp(\phi_2^\wedge)\neq\exp\big((\phi_1+\phi_2)^\wedge\big)$(矩阵不对易)。BCH 给出 $\ln(\exp A\exp B)$ 的展开,含李括号项。

**一阶近似(重点)** —— 当其中一个量是小量时:

$$\ln\big(\exp(\phi_1^\wedge)\exp(\phi_2^\wedge)\big)^\vee\approx
\begin{cases}
J_l(\phi_2)^{-1}\,\phi_1+\phi_2 & \phi_1\text{ 为小量(左乘)}\\
J_r(\phi_1)^{-1}\,\phi_2+\phi_1 & \phi_2\text{ 为小量(右乘)}
\end{cases}$$

直观:**在李群上左乘一个微小旋转 $\exp(\delta\phi^\wedge)$,等价于在李代数上加一项 $J_l^{-1}\delta\phi$**。$J_l$ 即 4.2.3 那个左雅可比 $J$,$J_r(\phi)=J_l(-\phi)$。

## 4.3.2 导数模型 vs 扰动模型

- **导数模型**:直接对 $\phi$ 求 $\dfrac{\partial(\exp(\phi^\wedge)p)}{\partial\phi}$ → 结果**带 $J_l$**,难算。
- **扰动模型**:对 $R$ 左乘(或右乘)微小扰动 $\exp(\delta\phi^\wedge)$,对 $\delta\phi$ 求导 → **不含 $J_l$**,干净。**优化里几乎只用扰动模型。**

**核心结论(务必背 + 区分左右)**:

$$\text{左扰动}:\ \frac{\partial(Rp)}{\partial\delta\phi}=-(Rp)^\wedge\qquad\text{右扰动}:\ \frac{\partial(Rp)}{\partial\delta\phi}=-R\,p^\wedge$$

> ⚠️ **左/右扰动别搞混**:
> - **左扰动** $R\leftarrow\exp(\delta\phi^\wedge)R$,扰动在**世界系** —— 高翔本书默认。
> - **右扰动** $R\leftarrow R\exp(\delta\phi^\wedge)$,扰动在**体坐标系** —— **FAST-LIO2 / IESKF 用这个**(⊞/⊟ 算子)。
> - 读代码先确认扰动放哪边,否则雅可比差一个 $R$、符号也可能错。

## 4.3.3 SE(3) 扰动模型

$$\frac{\partial(Tp)}{\partial\delta\xi}=\begin{bmatrix}I & -(Rp+t)^\wedge\\ 0^\top & 0^\top\end{bmatrix}\quad(\text{左扰动})$$

这个分块直通第6/7讲 BA 的雅可比、以及 FAST-LIO 点面残差对位姿的求导。

## 伴随性质(连接左右扰动的桥梁)

$$R\,\exp(p^\wedge)R^\top=\exp\big((Rp)^\wedge\big)\quad\Longrightarrow\quad \mathrm{Ad}_R=R$$

含义:把切空间向量 $p$ 用 $R$ 旋转,等价于先把 $p$ 本身用 $R$ 转过去。这是左↔右扰动互转、$SE(3)$ 伴随矩阵的基础。`ex03_adjoint.cpp` 数值验证它。

---

## 4.1–4.2 回顾自查(详见 `ch4_lie_algebra.md`)

- [ ] 为什么旋转矩阵不能直接做加法/梯度下降?李代数解决了什么?
- [ ] 默写罗德里格斯公式 $\exp(\theta a^\wedge)=\cos\theta\,I+(1-\cos\theta)aa^\top+\sin\theta\,a^\wedge$,说出 $\mathfrak{so}(3)$ 物理意义
- [ ] $\mathfrak{se}(3)$ 里 $\rho$ 和真实平移 $t$ 什么关系?($t=J\rho$)
- [ ] 指数映射为什么是「满射非单射」?

## 4.3 自查

- [ ] 为什么 $\exp(\phi_1^\wedge)\exp(\phi_2^\wedge)\neq\exp((\phi_1+\phi_2)^\wedge)$?BCH 一阶近似形式?
- [ ] 左扰动与右扰动的导数分别是什么?FAST-LIO 用哪个?
- [ ] 伴随性质 $\mathrm{Ad}_R=R$ 怎么用来在左右扰动间转换?
