// ch4 练习 1:Sophus 基础 —— SO(3) / SE(3) 的李群-李代数操作
//
// 对应官方代码:reference/slambook2/ch4/useSophus.cpp
// 目标:掌握 exp / log、hat / vee,以及【扰动模型】更新位姿(优化的基础)。
// 依赖:Sophus(header-only,新版依赖 fmt)、Eigen3。Sophus 安装见本目录 README.md。
//
// 编译:在 perception/visual_slam/ 下 cmake -B build && cmake --build build
//       运行 ./build/ex01_sophus_basics

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main() {
    // ============================================================
    // 1. 构造 SO(3):沿 Z 轴旋转 90°
    //    SO3 可由旋转矩阵 R 或单位四元数 q 构造,两者应等价
    // ------------------------------------------------------------
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    Quaterniond q(R);
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);
    // TODO 1: 打印 SO3_R.matrix() 与 SO3_q.matrix(),确认两种构造结果一致

    // ============================================================
    // 2. 对数映射 log:SO(3) -> so(3)
    //    得到的 3 维向量就是李代数(物理意义 = 轴角 θ·a)
    // ------------------------------------------------------------
    // 提示:Vector3d phi = SO3_R.log();
    //       反对称化 Sophus::SO3d::hat(phi) -> 3x3 矩阵
    //       还原    Sophus::SO3d::vee(Phi) -> 3 维向量
    // TODO 2: 求 phi 并打印;验证 vee(hat(phi)) == phi(hat/vee 互逆)
    Vector3d phi = Vector3d::Zero();  // TODO: 替换为 SO3_R.log()

    // ============================================================
    // 3. 指数映射 exp:so(3) -> SO(3)
    // ------------------------------------------------------------
    // TODO 3: 用 Sophus::SO3d::exp(phi) 映回,确认 .matrix() == R

    // ============================================================
    // 4. 【扰动模型】更新 SO(3) —— 优化里"位姿加增量"的正确方式
    //    左扰动:R <- exp(dphi^) * R  (扰动在世界系,高翔本书默认)
    //    右扰动:R <- R * exp(dphi^)  (扰动在体坐标系,FAST-LIO/IESKF 用)
    // ------------------------------------------------------------
    Vector3d dphi(1e-4, 0, 0);  // 一个微小扰动
    // TODO 4a: 左扰动更新 SO3_left  = Sophus::SO3d::exp(dphi) * SO3_R;
    // TODO 4b: 右扰动更新 SO3_right = SO3_R * Sophus::SO3d::exp(dphi);
    // TODO 4c: 打印两者 .matrix() 的差异,体会"扰动放左/放右结果不同"

    // ============================================================
    // 5. SE(3) 与 se(3)
    //    se(3) 是 6 维:Sophus 约定【前 3 维平移 rho,后 3 维旋转 phi】
    //    注意 rho != t,需经左雅可比 J:t = J * rho(见 notes)
    // ------------------------------------------------------------
    Vector3d t(1, 0, 0);
    Sophus::SE3d SE3_Rt(R, t);
    // TODO 5a: Vector6d xi = SE3_Rt.log(); 打印 xi,观察前 3 维 rho 是否等于 t
    // TODO 5b: 用 Sophus::SE3d::exp(xi) 映回,确认 == SE3_Rt
    // TODO 5c: 给 6 维扰动 dxi(很小),左乘更新:Sophus::SE3d::exp(dxi) * SE3_Rt
    Vector6d xi = Vector6d::Zero();  // TODO: 替换为 SE3_Rt.log()

    (void)phi; (void)xi;  // 占位,避免未使用告警;实现 TODO 后可删
    cout << "[ex01] 骨架已就绪,按 TODO 逐步填充。\n";
    return 0;
}
