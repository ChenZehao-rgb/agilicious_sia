# 20261029 motor[0] 工程初版

该入口使用真实 `1.BFL/1.ulg` 为主、`2.BFL/2.ulg` 为辅助。先拟合单电机内部 motor 指令域，再按目标 profile 名义转换为原始 MSP RC—整机总推力表。原来的 `analyze_thrust_logs.py` 保留用于具备原始RC证据的完整标定流程；本入口复用其日志适配器、单调回归和运行时文件契约。

产物位于 [`../../config/thrust_motor0_20261029/`](../../config/thrust_motor0_20261029/REPORT.zh-CN.md)，选用配置是 [`../../config/hardware_reference_trial.yaml`](../../config/hardware_reference_trial.yaml)。`hardware.yaml` 保持原样，`hardware_reference_trial.yaml` 保留它的 shadow、授权、接管、控制器和限幅参数，仅选择新推力模型及文件路径。

## 离线复现

从仓库根目录执行；只读原日志。依赖同目录 `requirements.txt`，无ROS或串口依赖。decoder 使用官方 [betaflight/blackbox-tools](https://github.com/betaflight/blackbox-tools)，不能使用 `--raw`（那会禁用预测恢复）。本次官方源码及二进制在 `build/motor0_trial_decoder`，具体commit、二进制hash、完整argv/帮助/统计保存于产物 `decode_run*.json`。

若需重新构建 decoder（只需要解码器，无需 Cairo 图像渲染依赖）：

```bash
git clone https://github.com/betaflight/blackbox-tools.git build/motor0_trial_decoder
make -C build/motor0_trial_decoder ./obj/blackbox_decode -j4 \
  CFLAGS='-O2 -std=gnu99 -Isrc -pthread' LDFLAGS='-lm -pthread'
```

已有目录时不用再 clone；复算相同解码器时 checkout `decode_run1.json` 中的 `decoder_source_commit`。本任务未修改外部 decoder 源码。

```bash
python3 agi_ros2/tools/thrust_calibration/motor0_trial.py inspect \
  --input-dir /home/sia/bag/20261029test \
  --cache-dir build/motor0_trial \
  --decoder build/motor0_trial_decoder/obj/blackbox_decode

python3 agi_ros2/tools/thrust_calibration/motor0_trial.py fit \
  --input-dir /home/sia/bag/20261029test \
  --cache-dir build/motor0_trial \
  --output-dir agi_ros2/config/thrust_motor0_20261029 \
  --target agi_ros2/config/thrust_motor0_20261029/target_fc_mapping.yaml \
  --hardware-profile agi_ros2/config/hardware.yaml
```

`fit` 会生成数值表、配置、统计、中文报告及图；会更新 `hardware_reference_trial.yaml`，不改基础 `hardware.yaml`。`--vref` 默认24，`--exponent` 可显式覆盖选参结果（仍保留原p对比）。解码缓存与原日志hash绑定；不匹配或CSV被改动会拒绝，换数据请换缓存目录。输出目录是生成物目录，复算会覆盖其中同名产物；不要把它指向原始日志目录。

目标配置变化时只重建RC表：

```bash
python3 agi_ros2/tools/thrust_calibration/motor0_trial.py export \
  --output-dir agi_ros2/config/thrust_motor0_20261029 \
  --target agi_ros2/config/thrust_motor0_20261029/target_fc_mapping.yaml \
  --hardware-profile agi_ros2/config/hardware.yaml
```

选择 static 时在目标yaml同时设 `selected_target_profile: static` 和 `dyn_idle_min_rpm: 0`。当前主文件明确选 `dynamic_nominal`、历史profile1的40；无运行时猜测。两份对照表总会生成，但运行时仅加载 `calibration.yaml.table_file`。该转换器只实现已声明的线性、无限制、非3D目标条件；遇到非线性/限幅参数会明确拒绝，不能套用同一公式。

## 产物及数值含义

- `motor_reference_thrust.csv`：`motor_command,single_thrust_n_at_vref`。正的已测idle点保留，不增加零推力点。
- `motor_fit_points.csv`：单调处理前后数值、修改量、相同motor候选窗口数及使用数。
- `reference_thrust.csv`：当前唯一所选profile的整数原始RC—整机N表。
- `reference_thrust_static.csv` / `reference_thrust_dynamic_nominal.csv`：两种端点映射对照。
- `steady_windows.csv`：原BF/PX4时钟、核心边界、motor真实范围、带载电压、去皮后force均值/标准差/半段差、样本数和用途。
- `calibration.yaml`：运行时契约、来源、provisional状态、映射假设、电压域、低端策略、选参与辅助误差。
- `fit_report.json`、`REPORT.zh-CN.md`、`curves.png`：复算结果、对齐阶跃证据、±80ms敏感性和限制。

分箱跨度≤4和时长阈值不按拟合残差调节。枚举每个bin起点的最大稳定区间，再取最长互不重叠窗口；这避免“第一个settling bin越界导致整个后续平台丢失”。近零力只报告绝对N误差；p比较使用净力≥50gf。同组下扫参与p选择，不能再称独立测试。第二组去皮142gf情景使用与196gf完全相同的高于50gf样本集合。

单电机乘4只发生在导出函数 `export` 中。运行时接受 `ControlCommand.total_thrust` 的整机N，执行 `T_lookup=T_desired*(Vref/V)^p` 再反解；不乘质量/4、不改body_rates。以原MSP报告身份/请求时间计龄，Health重复发布不刷新样本年龄；τ=0，沿用硬件配置的电池超时。

`provisional` 文件可在 `shadow=False` 的加载检查中通过，无新审批开关。`thrust_calibrated=false` 如实保留，`thrust_mapping_ready=true` 仅表示模型成功加载；电压越域/过期仍在命令与watchdog路径独立拒绝。`OutputStatus` 展示 `calibration_status`、provenance、`reference_lower_saturated` 和 `reference_idle_requested`。正低端返回表内最小RC；0N复用输出层idle通道，均不宣称精确产生目标力。

## 构建及测试（离线，不连接硬件）

本次使用隔离安装目录，不覆盖原生产安装：

```bash
source /opt/ros/humble/setup.bash
agi_ros2/scripts/build.sh --build-base build/thrust_reference \
  --install-base build/thrust_reference_validation/install \
  --cmake-args -DAGI_ROS2_GAZEBO=OFF
python3 -m unittest discover -s agi_ros2/tools/thrust_calibration/tests
source build/thrust_reference_validation/install/setup.bash
AGI_ROS2_TEST_INSTALL="$PWD/build/thrust_reference_validation/install" \
  python3 -m unittest discover -s agi_ros2/test -p test_reference_runtime.py
```

测试构建纯C++ probe并与Python对同一真实YAML/CSV逐点比较，覆盖电压补偿方向、p=0、正低端、0N、域外/NaN拒绝、单调平台、MSP真实样本年龄/滤波旁路、motor530映射、gap裁剪、gf→N及只乘4一次。启动配置测试只构造launch动作；synthetic拒绝测试只使用不存在的临时UART路径，拒绝发生在打开设备前。

## 用户后续选择profile的命令（本次未执行）

先使用上述隔离安装环境。shadow命令会启动传感器/MSP读取，因此不是纯离线测试，但保持既有串口控制帧禁用：

```bash
ros2 launch "$PWD/agi_ros2/launch/shadow.launch.py" mode:=hardware \
  runtime_config:="$PWD/agi_ros2/config/hardware_reference_trial.yaml"
```

用户自行选择正常硬件模式时使用同一profile；默认仍为 `shadow_only: true`。下面显式关闭shadow的命令只供用户后续选用，本次未运行；既有ARM/AUTO/接管/健康/超时授权条件仍须满足：

```bash
ros2 launch "$PWD/agi_ros2/launch/flight.launch.py" mode:=hardware \
  runtime_config:="$PWD/agi_ros2/config/hardware_reference_trial.yaml" shadow_only:=false
```

无需追加标定批准参数，也没有自动写FC、关闭动态怠速、修改PID或解锁电机的动作。四电机一致性、名义零力矩映射、动态怠速增量和实际飞行效果尚未实测验证。
