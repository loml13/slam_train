// ch4 练习 2:评估轨迹误差(ATE,Absolute Trajectory Error)
//
// 对应官方代码:reference/slambook2/ch4/example/trajectoryError.cpp
// 思路:用 Sophus::SE3d 表示每个位姿;第 i 帧误差
//         e_i = || log( T_gt,i^{-1} * T_esti,i ) ||_2     (6 维李代数范数)
//       绝对轨迹误差(均方根):
//         RMSE = sqrt( (1/N) * sum_i e_i^2 )
//
// 数据(TUM 格式,每行:time tx ty tz qx qy qz qw):
//   groundtruth: ../../reference/slambook2/ch4/example/groundtruth.txt
//   estimated  : ../../reference/slambook2/ch4/example/estimated.txt
// 路径是相对 perception/visual_slam/ 的(从该目录运行 ./build/ex02_trajectory_error)。

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// 读取 TUM 格式轨迹文件 -> SE3 位姿序列
TrajectoryType ReadTrajectory(const string &path) {
    TrajectoryType traj;
    ifstream fin(path);
    if (!fin) {
        cerr << "无法打开轨迹文件: " << path << endl;
        return traj;  // 返回空,main 中需判空
    }
    // TODO A: 逐行读取 time tx ty tz qx qy qz qw,构造 SE3 压入 traj
    //   double time, tx, ty, tz, qx, qy, qz, qw;
    //   while (fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
    //       Sophus::SE3d p(Quaterniond(qw, qx, qy, qz), Vector3d(tx, ty, tz));
    //       traj.push_back(p);
    //   }
    //   注意:Quaterniond 构造顺序是 (w, x, y, z),而文件里是 qx qy qz qw,别填反
    return traj;
}

int main() {
    TrajectoryType gt   = ReadTrajectory("../../reference/slambook2/ch4/example/groundtruth.txt");
    TrajectoryType esti = ReadTrajectory("../../reference/slambook2/ch4/example/estimated.txt");

    // TODO B: 检查 gt、esti 非空且 size() 相等,否则报错退出

    // TODO C: 遍历两条轨迹,累加平方误差
    //   for (size_t i = 0; i < esti.size(); i++) {
    //       Vector6d e = (gt[i].inverse() * esti[i]).log();
    //       rmse += e.squaredNorm();
    //   }
    //   rmse = sqrt(rmse / esti.size());
    double rmse = 0.0;

    cout << "[ex02] RMSE (ATE) = " << rmse << endl;
    // 思考:为什么用 SE(3) 李代数范数,而不是只算平移的欧氏距离?
    //       (答:李代数范数同时包含旋转误差,平移距离会漏掉姿态偏差)
    return 0;
}
