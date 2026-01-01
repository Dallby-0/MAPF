# MAPF 算法课程期末作业

本项目为算法课的期末作业，针对 MAPF（多智能体路径规划）问题，实现了基础的 **CBS + 时空 A*** 算法。

## 实现说明
- 代码为 **100% 纯手工编写**，约 **90% 依赖人工调试**（好比印度手作咖喱饭）。
- 出于方便考虑，所有核心实现均集中在 `main` 函数内（其他头文件仅为空壳）。

## 测试数据
测试地图文件位于 `maps/` 目录中，来源于标准的 MAPF 测试数据集（选取了Berlin_1_256和Maze32-32-2）：  
[movingai](https://movingai.com/benchmarks/mapf/index.html)

## 运行结果
- 结果保存在 `results/` 目录中。
- 输出格式与 [Tracker](https://tracker.pathfinding.ai/) 保持一致。
- 目前导出的结果与 Tracker 中的一致。

## 可视化
导出的路径解可在以下网站进行可视化演示：  
[justinshetty](https://justinshetty.com/mapf-visualizer/)
