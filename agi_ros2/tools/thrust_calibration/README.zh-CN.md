# 离线手动推力标定与 reference_table

这是文件分析和项目映射实现。没有 MSP 扫描器、串口驱动移植或电机控制入口。现有 `hardware.yaml` 仍为 `quadratic`，旧二维 `table` 格式不变。20261029两组真实日志的 motor[0] 工程初版现已实现，见 [MOTOR0_TRIAL_README.zh-CN.md](MOTOR0_TRIAL_README.zh-CN.md)。该入口使用显式目标profile名义转换，允许provisional加载；以下保留原始RC证据充分时的完整标定流程及其严格验收规则。飞行资格不由离线拟合建立。

## 安装与入口

Python 3.10+，依赖见 `requirements.txt`。建议独立环境安装；C++ 测试另需 g++、yaml-cpp 开发包。无需 ROS 运行离线流程。

```bash
python3 -m pip install -r agi_ros2/tools/thrust_calibration/requirements.txt
python3 agi_ros2/tools/thrust_calibration/analyze_thrust_logs.py inspect --file /absolute/log.BFL --out /tmp/bf-inspect
python3 agi_ros2/tools/thrust_calibration/analyze_thrust_logs.py inspect --file /absolute/log.ulg --out /tmp/px4-inspect
```

`inspect` 按内容识别格式，列出 Blackbox 各 log 头或 ULog 所有 topic/instance/字段类型、数值范围、时间异常、dropout。范围不能证明单位；自定义 force topic 不预设。重置、重复时刻不静默排序或拼接。

Blackbox 原始数据需官方 `blackbox_decode`。本机未发现已安装 decoder，因此没有宣称定制固件原始日志兼容。工具先执行 `--help`，要求 index/stdout/明确时间与电压单位/output-dir 选项，用 argv 数组调用，拒绝不存在的 log index。保存完整命令、可执行文件 SHA256、帮助/版本输出、原始头、frame statistics；必须人工核实坏帧统计后才确认 `decoder_quality_reviewed`。所有输出写分析目录，不写原日志目录。`--raw` 是预测差分，不能用于物理标定；本工具不启用它或模拟电流/IMU。

```bash
python3 agi_ros2/tools/thrust_calibration/analyze_thrust_logs.py decode \
  --file /absolute/log.BFL --decoder /absolute/blackbox_decode --log-index 1 --out /tmp/bf-decoded
```

官方源码：[blackbox-tools](https://github.com/betaflight/blackbox-tools/blob/d89f96b983d0f5c8f053acf003ee2998b02b5b0f/src/blackbox_decode.c)、[pyulog](https://github.com/PX4/pyulog)。读取 ULog 使用官方 pyulog；合成二进制 ULog 测试覆盖多实例/dropout，不能替代真实自定义驱动日志验证。

## 先核实轴，再配对

复制 `analysis_config.example.yaml` 和 `sessions.example.yaml`，填写 null，不要直接把示例阈值当台架验收限值。

- 首选 `debug[3]`，要求已确认 `RX_SIGNAL_LOSS`、DEBUG 解码未额外缩放、固件与日志匹配、手动和 MSP 经过同一 RX range、无 PWM/PPM 状态平均。R→C 为 `rx_min+(R-1000)*(rx_max-rx_min)/1000`。返回量化不确定度，排除限幅边界和域外 C；`valid_when` 必须描述接收、failsafe、保持旧值等有效性证据。若现有字段不足以证明有效性，应保持 unresolved。
- `rcCommand[3]` 是经过油门曲线/可能 RC smoothing 的 Q，`setpoint[3]` 是 mixer 油门，`motor[]` 是内部输出。它们不可改名为原始 RC。当前未实现未知完整 G(C) 的稳态反解；只有这些轴时保留 inspect 诊断，禁止部署表。
- 电压单位只能显式 V/mV/centivolt；高分辨率 rcCommand 的 ×10 与 debug 无关。不根据数值猜单位。BF vbatLatest 与 MSP130 displayFiltered 仅按稳态兼容处理。
- 力支持 N/kgf/gf/有实际系数的 counts、符号、零点、可选向量单位方向投影。零点须来自独立无推力段；不从解锁怠速去皮，不对漂移任意去趋势。
- 配对由 run_id、battery_id、block_id、arm_cycle、BF log_index、PX4 topic/instance、各自原时钟区间明确指定。每个会话可选 `sample_slice: [start,end]` 切分重启前后原始行；不允许默默合并时钟重置。CSV 也必须记录原文件、hash、解码器命令/版本/单位证据。实测会话另要求人工控制来源和原日志来源证据。
- 可用 `diagnostic_fields` 保留 eRPM、温度或其他字段均值；含义/单位仍须记录。eRPM 不自动当机械 RPM。`boundary_fields` 指定可见模式/profile/arm 边界；不可见边界由明确会话分段承担。BOXMSPOVERRIDE=44 超过候选低32位记录范围，缺位只能记 unknown。

已核对本地 Betaflight `6b92b586eeb53b2783c58348810b2b9af091e25b` 的 `src/main/rx/rx.c`：`readRxChannelsApplyRanges` 中手动/override 取样后共同 range 转换，`detectAndApplySignalLossBehaviour` 有无效值保持/替换及脉宽限幅，末尾记录 debug[3]。因此 R 不是未经处理的原始 C。提供的 CLI 是 diff，记录 debug_mode=MAVLINK_TELEMETRY，没有完整 rxrange 等证据，且固件 norevision 不能与源码 commit 自动等同。实际烧录语义仍待短日志核验。

## 可审查的阶段命令

每步都读同一 config/sessions；更改源文件或配置后须从 inspect 重跑，产物身份包含所有源文件和配置 hash。输出目录不能包含输入文件。

```bash
python3 agi_ros2/tools/thrust_calibration/analyze_thrust_logs.py inspect  --sessions sessions.yaml --config analysis_config.yaml --out results
python3 agi_ros2/tools/thrust_calibration/analyze_thrust_logs.py align    --sessions sessions.yaml --config analysis_config.yaml --out results
python3 agi_ros2/tools/thrust_calibration/analyze_thrust_logs.py extract  --sessions sessions.yaml --config analysis_config.yaml --out results
python3 agi_ros2/tools/thrust_calibration/analyze_thrust_logs.py fit      --sessions sessions.yaml --config analysis_config.yaml --out results
python3 agi_ros2/tools/thrust_calibration/analyze_thrust_logs.py validate --sessions sessions.yaml --config analysis_config.yaml --out results
python3 agi_ros2/tools/thrust_calibration/analyze_thrust_logs.py report   --sessions sessions.yaml --config analysis_config.yaml --out results
```

当前 `align` 使用人工审查后的同向共同阶跃锚点，检查真实信号方向/幅度、稳健拟合、首尾覆盖和独立边缘残差；不自动在周期信号里猜唯一对应。没有确定对应时必须拒绝/补充证据。`a` 可固定为1；自由漂移受 ppm 约束。`b` 混入动力与传感器响应延迟，不能叫硬件同步。锚点不够、错序、多解、验证残差过大均拒绝。

`extract` 用 RC/motor 阶跃和显式边界识别候选，收缩过渡及时间不确定裕量；分别在原始样本上检验力/RC/电压/四电机斜率和方差，拒绝 gap/dropout/无效窗口。对 ±时间不确定度重新取样并检查力统计和 p 敏感性。sample_seq 可用时单列真实独立样本数；否则 unknown，采样点数不冒充独立更新率。

`fit` 只用训练块估计 p/参考曲线；按窄 RC bin 建截距并控制局部 RC 偏差，要求跨电压、跨完整块重复，检查设计矩阵可辨识性、分箱宽度敏感性、各 RC 区间 p、块 bootstrap 不确定性。验证块选择 p=0/1/2/p_fit；最终测试不影响选择。单次升扫同时掉压会失败。有效电压域取各 RC bin 的公共支持，最终测试需覆盖每个导出 RC 节点的电压域。统一 p 不成立时应明确缩小配置/数据域后重新划分实验，不自动提高模型自由度。

参考表只在离线将单电机乘4一次。PAVA允许平台，不补 (1000,0)。正怠速时0N也会域外拒绝。验证有前后零漂、独立点数、平均/最大相对误差及覆盖检查；分母为超过明确噪声下限的实测单电机 N。示例5%/10%是初步离线目标。实测还要求有效配置、FC电压补偿关闭、稳态电压兼容证据，才可导出 validated。

产物：`inspection.json`、`alignment.yaml/json`、每会话对齐图、`windows.json`/`steady_windows.csv`（含所有拒绝）、`model.json`、单电机参考表、`reference_thrust.csv`、`calibration.yaml`、`validation.json`、`model_validation.png`、`report.html`。未通过 inspect 不导出伪原始 RC 表。失败会返回非零状态；先前阶段产物可继续审查。

## 项目契约与运行时

CSV 精确表头为 `rc_throttle,total_thrust_n_at_vref`；RC 为整数严格递增、总 N 非递减且至少两个不同值。首尾必须等于有效 RC 域。平台采用广义逆：0.8/0.8/2.8/6.8N 对应1000/1100/1200/1300，则 inverse(0.8)=1000、inverse(1.8)=1150；裁到1100后 inverse(0.8)=1100。全平台、负数、NaN、重复 RC、错单位/schema均拒绝，不外推/clip。

`flight.thrust_calibration` 相对 runtime profile 解析，CSV 相对 calibration.yaml 解析。参见 `../../config/reference_table.example.yaml`，在原 hardware.yaml 的副本中使用，保持 `shadow_only: true`。Vref/p/电压域/滤波只存在 calibration.yaml。Python launch 与 C++ 直接节点加载分别严格校验，同一合成/无效契约语料做双端测试。synthetic 或 unvalidated 只允许完全禁止控制输出的 shadow/离线路径，不能用于实机输出。

```text
ControlCommand.total_thrust [vehicle N]
  -> fresh MSP130 report -> V_filtered
  -> scale=(Vref/V_filtered)^p
  -> Tlookup=total_thrust*scale -> inverse table -> raw RC
```

运行时不再乘质量或4、不缩放 body_rates、不重复逆 RX range、不做 I*R 修正。V>Vref 可得到 scale<1；p=0仍需有效电压。raw/filtered 电压、总推力超域及溢出走既有输出拒绝路径。既有最小油门/撤销授权路径仍是命令语义，不声称0N物理推力；没有新增自动降落/disarm策略。

MspEvent 现有 request/receive monotonic 时刻经 MspEvidence→MspState/Health 传递，增加报告序号和会话（含本机 clock epoch）。滤波只在新报告更新，alpha=1-exp(-dt/tau)，tau=0旁路；重复 Health 不刷新 age。年龄从 request 起保守计算，不声称是 ADC 硬件采样时刻。时钟/可观测会话/FC导航重置清空滤波，等待新报告；无 FC boot counter 时不能检测所有复位。现有2Hz轮询及门禁没有放宽。输出 watchdog 独立检查 freshness；static thrust_mapping_ready 不依赖异步电压回授。

OutputStatus 增加校准身份、来源、raw/filtered/age、Vref/p/scale、目标N/查表N、实际取整RC、能力域和拒绝原因。`thrust_calibrated` 表示已验证的单电机乘4离线模型，不表示四动力一致性/实机补偿/飞行验证。

## 可复现合成验证

```bash
python3 agi_ros2/tools/thrust_calibration/tests/generate_synthetic.py /tmp/thrust-synthetic
# 用上述7个阶段，sessions/config分别取 /tmp/thrust-synthetic/{sessions,config}.yaml
python3 -m unittest discover -s agi_ros2/tools/thrust_calibration/tests -p 'test_*.py' -v
```

夹具带明确 synthetic 标签，p=1.6、非恒等 RX range、RC抖动、100/60Hz不同采样率、偏移、小漂移、物理响应延迟、噪声、首尾同步标记、前后零点和训练/验证/最终测试块。C++探针编译后直接测试真实模型/滤波器，不启动ROS或串口。

正常构建脚本可隔离构建和安装：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 ./agi_ros2/scripts/build.sh -DAGI_ROS2_GAZEBO=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/build/thrust_reference_validation/install" \
  --build-base build/thrust_reference \
  --install-base "$PWD/build/thrust_reference_validation/install" --cmake-force-configure
```

ROS message有新增字段，部署时须一致重建相关发布/订阅节点，不能混用旧安装消息。此次只进行了离线/合成测试和软件构建，没有部署。具体测试证据见 `VALIDATION.zh-CN.md`。
