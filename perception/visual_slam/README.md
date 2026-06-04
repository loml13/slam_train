# visual_slam 视觉 SLAM

跟随《视觉SLAM十四讲》(高翔)的练习代码与笔记。
官方配套代码见 [../../reference/slambook2/](../../reference/slambook2/)。

## 章节

| 目录 | 内容 | 状态 |
|---|---|---|
| [ch3_eigen/](ch3_eigen/) | 第3章:Eigen 几何模块 —— 旋转与变换表示 | 完成 |
| [ch4_sophus/](ch4_sophus/) | 第4章:Sophus 李群李代数 —— exp/log、BCH、扰动模型 | 进行中(骨架) |

## 构建

用 CMake 管理(`CMakeLists.txt` 依赖 Eigen3;ch4 另需 Sophus,见 [ch4_sophus/README.md](ch4_sophus/README.md)):

```bash
cmake -B build
cmake --build build
./build/ex01_basics
```

也可在 VSCode 中用「运行」按钮编译单个文件。
