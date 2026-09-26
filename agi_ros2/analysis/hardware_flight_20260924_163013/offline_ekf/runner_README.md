# 原生 EKF 离线回放接口

`offline_ekf.cpp` 链接本仓库 `agi::EkfImu`，只读输入、写 CSV，不连接 ROS、不发送控制命令、不修改硬件配置。它检验给定初值/时间修正/高度重建假设下的数值行为，不能证明真实定位精度或满足实机初始化条件。

```bash
./build_runner.sh
./offline_ekf events.csv run 24.322 0.025 ../../../config/hardware.yaml
```

最后一个参数应为实际 `hardware.yaml` 路径；在当前目录执行时，相对路径为 `../../../config/hardware.yaml`。构建需要已有 `build/agi_ros2`；可通过 `AGILIB_BUILD_DIR` 指向另一已配置的 ROS 构建目录。构建脚本先更新 `agilib` 目标，再使用相同的 C++17/Eigen `-march=native` ABI 编译离线程序。`max_nis` 可用 `inf`，仅作为去除创新门限的敏感性对照。

输入是逗号分隔的数字行，无表头；`#` 开头的行可作注释。所有时间单位为秒，使用提取脚本明确修正后的统一时间；所有字段必须有限。IMU 使用 FLU 机体系，比力单位 m/s²、角速度 rad/s；位置/速度使用 ENU 米/m/s，航向为东向零、逆时针正的 rad。方差单位为相应物理量单位的平方。

```text
S,t,segment,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bgx,bgy,bgz,bax,bay,baz,pvarx,pvary,pvarz,vvarx,vvary,vvarz
I,t,ax,ay,az,gx,gy,gz
G,t,px,py,pz,vx,vy,vz,yaw,pvarx,pvary,pvarz,vvarx,vvary,vvarz,yawvar
```

`S` 显式创建新段，四元数表示 FLU 到 ENU 的旋转且须归一化。位置/速度初始方差由 `S` 给出。IMU 噪声、过程噪声、初始姿态和偏置方差从 `hardware.yaml` 的 `fusion` 配置读取；GPS 每次观测方差由 `G` 给出。回放不使用 RTK 的默认厘米级观测方差代替 GNSS 实际报告方差。

每段 IMU 严格递增，`S` 后首个 IMU 距初始化也不能超过 `max_imu_gap`。GPS 按采样时刻排序，可在当前 IMU 尚未覆盖时先入队；下一 IMU 到达后按原始 GPS 时刻进行延迟更新。真实 IMU 缺口超过门限或时间倒退会停止该段，直到遇到新的 `S` 才恢复。提取脚本也必须在源会话变化处插入 `S`，不可只靠修正时戳隐藏会话切换。

输出：

- `run_states.csv`：每次 IMU 预测和 GPS 更新后的状态、四元数、陀螺仪/加速度偏置。
- `run_updates.csv`：观测接受/拒绝、七维 NIS、更新前创新（观测减预测）、GPS 原输入、更新后状态、最近后验的方差和累计计数。
- `run_segments.csv`：明确的初始化和因 IMU 缺口停止事件。

`covariance_t` 是方差实际对应的**最近后验时间**。状态预测不会推进公开方差；不得把同一行的方差当成当前高频预测时刻的传播协方差。`rejected` 表示 EKF 实际尝试了更新但拒绝；`not_evaluated`、`segment_inactive` 等状态表示没有有效更新尝试，其 NIS 不可用于统计。初始时刻本身的 GPS 已用于 `S`，不要再次将其当成独立观测更新。

高度重建、时间修正、初始姿态/偏置来源、GPS 真值缺失等限制由提取脚本及主分析报告记录，不由本程序猜测或修复。

## 可复现验证

```bash
python3 verify_runner.py
python3 review_runner.py
```

`verify_runner.py` 仅依赖 Python 标准库和已构建程序，在 `unitcases/` 保留合成输入、原生输出、日志及 `verification.json`。它检查静止重力传播、GPS 延迟覆盖、NIS 拒绝、无门限对照、IMU 缺口停段、显式重启及 CSV 完整性。变化 IMU 序列中的即时/延迟 20 ms GPS 对照有 9 次更新，状态、NIS 和协方差输出完全一致。

`review_runner.py` 需要 NumPy 和主分析生成的 `inputs/`、`results/`。它独立将本包 GPS 交付延迟 20 ms，核对全部 847 次更新的状态、NIS、协方差及接受/拒绝，写入 `runner_review.json`。所有 NIS、公开协方差和更新决策相同；拒绝后的纯预测积分因缓存分段时间不同，最大位置差约 `1.4e-7 m`，远小于本次米级差异。
