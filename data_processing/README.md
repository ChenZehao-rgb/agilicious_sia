# 三维轨迹绘图

默认读取同级 `build/loop50.csv`，输出到本目录的 `output/`。

```bash
# 在项目根目录运行，使用系统包避开本机用户 NumPy 与系统 Matplotlib 的版本冲突
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/loop50-matplotlib python3 data_processing/plot_trajectory.py
```

在兼容的 Python 环境中安装 `requirements.txt` 后，也可以直接使用 `python3`。

输出：

- `output/loop50_trajectory.png`：高分辨率图。
- `output/loop50_trajectory.pdf`：可缩放 PDF。

蓝色实线为实际位置 `p_x/y/z`，灰色虚线为参考位置 `ref_p_x/y/z`。
蓝色圆点 Start 标记所选时间范围内第一条实际位置，橙色方点 End 标记最后一条实际位置。
三轴单位为米，并保持相同物理尺度。不显示无人机姿态、方向箭头或速度色标。
脚本仅需要时间与位置字段，不再读取姿态或速度，也不再生成欧拉角文件；此前导出的姿态 CSV 是旧版本结果。

数据处理保留全日志时间范围，包括起飞前和悬停阶段。日志中的 22 个重复时间戳保留最后一条记录；反向时间和必需字段非有限值会报错。原日志不会修改。日志写入器在 `has_reference=0` 时将实际状态写入参考字段，因此该阶段曲线重合不代表跟踪性能。

可选参数示例：

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/loop50-matplotlib python3 data_processing/plot_trajectory.py build/loop50.csv --start 9 --end 21 --elev 30 --azim -45 --output data_processing/output/flight_detail
```

`--start` 和 `--end` 的单位为日志秒数；`--output` 不含扩展名。
