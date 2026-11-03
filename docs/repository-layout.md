# 2026-11-03 仓库分类迁移

| 原路径 | 新路径 |
|---|---|
| chassis/ | platforms/differential_f103/firmware/ |
| upper_computer/chapt6_ws/ | platforms/differential_f103/ros2_ws/ |
| data/ | platforms/differential_f103/calibration/ |
| chassis_f407/ | platforms/mecanum_f407/firmware/ |
| upper_computer/mecanumws/ | platforms/mecanum_f407/ros2_ws/ |
| upper_computer/其余资料 | platforms/differential_f103/legacy_upper_computer/ |

旧上位机资料按原工程归档关系保留，不据此判定其所有实验均已与F103实车集成。根docs保留跨车型开发历史。历史文档中的旧路径不批量改写，以免改变当时记录。

本次没有修改产品代码或运行项目。启动文件中的绝对路径依然需要部署时配置，迁移未验证构建成功。F407下的新仿真与巡检架构属于后续计划。
