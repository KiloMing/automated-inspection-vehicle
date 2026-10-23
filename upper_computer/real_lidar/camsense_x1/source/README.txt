基于此次粘贴的可运行源码修改。默认 frame_id=laser_link，包名和 include 路径不改。
替换 src/camsense_x1.cpp，入口 camsense_x1_node.cpp 无需修改。
随包附原接口头文件；如果此前换过上一版头文件，请恢复本包 camsense_x1.hpp 到 include/camsense_x1/。
不需要 scan_core.hpp，不需要使用上一版 CMakeLists.txt 或 package.xml。

参数：port=/dev/ttyUSB0，baud=115200，angle_offset=0，clockwise=true。
clockwise=true 延续旧驱动反向排列方向；false 对应可视化程序的角度方向。
角度范围 -pi 到 pi-2pi/400。缺测为 +inf，丢弃 quality=0、距离0、0x8000及80..8000mm以外数据。
完整36字节帧检查03 08和55 AA；保留/8插值，不实现未知校验算法。
按变换后的角度边界逐点切圈，丢弃启动时残圈。
scan_time 根据连续边界估算，初始0.19s仅用于首圈点时刻估计。
时间戳受串口、USB、系统调度误差影响，不是硬件时间。
clockwise=true 时 time_increment 为负，因为角度排序与采集顺序相反；实际RF2O版本需验证。
当前仅提供源码，尚未通过ROS2 Jazzy编译或硬件验收；不能声称已解决RF2O或SLAM问题。

Ubuntu中在~/ros_ladar运行colcon build --symlink-install，然后source install/setup.bash。
运行时包名以你当前package.xml为准，节点程序名以CMakeLists.txt为准。
验收顺序：编译 -> 静止/转动时scan质量 -> RF2O odom -> SLAM map。
