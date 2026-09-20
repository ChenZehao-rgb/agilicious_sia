# 本机 Docker Jazzy SITL + CM5 Jazzy 控制

宿主机保持 Ubuntu 22.04，容器使用 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo
Harmonic。此目录独立于仓库根目录的旧 ROS 1 Dockerfile。

## 1. 安装 Docker Engine（宿主机只需一次）

以下命令用于尚未安装 Docker 的 Ubuntu 22.04/24.04：

```bash
sudo apt update
sudo apt install -y ca-certificates curl xauth
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo docker run --rm hello-world
```

来源：[Docker 官方 Ubuntu 安装文档](https://docs.docker.com/engine/install/ubuntu/)。
若已有其他发行版的 Docker/containerd，先按该文档处理包冲突。
脚本会在当前用户不能访问 Docker 时使用 `sudo docker`，无需加入 docker 组。

## 2. 构建和启动

在普通桌面终端执行，不要对整个 run.sh 使用 sudo：

```bash
cd /home/sia/agilicious_internal-main
bash betaflight_sitl/docker/run.sh build
bash betaflight_sitl/docker/run.sh
```

脚本读取本机 `~/betaloop/config.txt`，将 Betaloop、Aeroloop 和 Betaflight ELF
所在目录按原绝对路径只读挂载。当前本机已有这些依赖；镜像不下载或重建 Betaflight。
换机器需先准备同架构的 SITL ELF、EEPROM、Aeroloop 源码/资源以及 Betaloop。
Betaloop 还检查 Aeroloop 原有 build 目录是否存在。
自定义 Betaloop 路径可设置 `BETALOOP_HOME=/absolute/path`。

每次启动将当前源码（包含未提交修改）同步到名为 `agilicious-sitl-jazzy` 的 Docker
卷中，在容器内重新构建/增量构建 acados、ROS 包和 Gazebo 插件。原仓库只读挂载，
不复用 Humble 的 build/install。容器删除后卷仍保留。
源码修改应在宿主机进行，然后重启容器同步；容器内对同步源码的编辑会被覆盖。

仅运行仿真服务、不显示 Gazebo 窗口：

```bash
bash betaflight_sitl/docker/run.sh --no-gazebo
```

首次构建成功后可跳过 run.py 的插件/ROS 包编译；改源码后不要加此参数：

```bash
bash betaflight_sitl/docker/run.sh --no-build
```

进入容器终端（仍先同步源码和构建 acados）：

```bash
bash betaflight_sitl/docker/run.sh bash
# 容器终端中：
python3 betaflight_sitl/run.py --betaloop-home "$BETALOOP_HOME"
```

Ctrl-C 停止；另一个终端也可执行：

```bash
sudo docker stop --time 30 agilicious-sitl-jazzy
```

同时只能运行一个此名称的容器；宿主机原有 SITL 应先退出以释放端口。

## 3. Gazebo 显示与 GPU

默认使用 X11/XWayland，脚本传递 Xauthority cookie，不需要 `xhost +`。
Intel/AMD 的 `/dev/dri` 存在时自动传入。

NVIDIA 需先按 [NVIDIA Container Toolkit 文档](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
安装工具并配置 Docker runtime，然后运行：

```bash
SITL_NVIDIA=1 bash betaflight_sitl/docker/run.sh
```

只想先验证窗口可打开，可尝试软件渲染（仿真速度可能明显下降）：

```bash
SITL_SOFTWARE_GL=1 bash betaflight_sitl/docker/run.sh
```

## 4. CM5 配置

两端使用同一版本源码和消息定义，各自编译。CM5 已有 Jazzy/acados 时：

```bash
source /opt/ros/jazzy/setup.bash
ACADOS_ROOT=/path/to/arm64/acados ./agi_ros2/scripts/build.sh -DAGI_ROS2_GAZEBO=OFF
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

容器默认使用相同的域号和 RMW。若 CM5 缺少该 RMW，安装
`ros-jazzy-rmw-fastrtps-cpp`。两端优先接同一有线局域网，允许 DDS 通信和本机 UDP 9004。
Docker 使用 host 网络，CM5 应使用**本机局域网 IP**，不是 Docker 网桥 IP。

在 CM5 编辑 `agilib/params/betaflight_udp.yaml`，仅把 `host` 改为本机 IP，
保留 `port: 9004` 和其余标定参数。然后用显式参数目录启动，确保读取修改后的源文件：

```bash
./agi_ros2/scripts/launch.sh mode:=sitl params_dir:="$PWD/agilib/params"
```

若希望后续只运行 `./agi_ros2/scripts/launch.sh`，将 CM5 的
`agi_ros2/launch/flight.launch.py` 中 `FLIGHT_CONFIG.params_dir` 设为该目录绝对路径；
或重新构建，使修改后的 YAML 安装到默认 share/agi_ros2/params 中。
launch 中轨迹和 bag 路径也必须在 CM5 上存在或可写。

本机 run.py 不启动控制器；CM5 保持 `mode='sitl'`，从本机 `/clock` 使用仿真时间。
仍需唯一遥控授权来源，例如在 CM5 另开终端、设置上述 ROS 环境并执行：

```bash
source install/agi_ros2/local_setup.bash
ros2 run agi_ros2 sim_rc.py --ros-args -p use_sim_time:=true
```

启动这两个脚本不会自动 ARM 或起飞；遥控操作参见 `agi_ros2/README.md`。

## 5. 验证

若旧镜像构建时报 `Target "CPPZMQ::CPPZMQ" not found`，重新执行
`bash betaflight_sitl/docker/run.sh build`，然后正常启动（不加 `--no-build`）。
Ubuntu 24.04 的 `cppzmq-dev` 是 `libzmq3-dev` 的推荐依赖，使用
`--no-install-recommends` 时必须显式安装；Dockerfile 已补上。
不需要删除 Docker 卷或手动修改 Gazebo 的 CMake 文件。

GCC 13 编译 Eigen 时可能在 AVX 分块赋值及 Quaternion/JacobiSVD 中报告
`array-bounds` / `maybe-uninitialized`。`agilib` 的 CMake 对 GCC 13 将这两类
诊断保留为警告，不再由 `-Werror` 升级为错误；其他编译器及其他诊断不变。
同步此修复只需重新启动容器，不加 `--no-build`，无需重建镜像。
这是构建兼容处理，不是对潜在越界或未初始化访问的运行时验证。

本机启动后，在 CM5 已设置相同 ROS 环境的终端检查：

```bash
source install/agi_ros2/local_setup.bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic hz /sensors/imu
```

若没有远端话题，检查域号、RMW、网卡选择、防火墙和局域网组播；修改 ROS 环境后
可执行 `ros2 daemon stop` 再检查。本机可用 `ss -lunp | grep ':9004'` 检查接收端；
必须监听 `0.0.0.0:9004` 或本机局域网地址，不能只监听回环。

本配置需要在目标机器实际构建和联调；仅通过脚本静态检查不代表闭环已验证。
