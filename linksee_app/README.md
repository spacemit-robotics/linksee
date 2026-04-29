# Native Linksee 应用

`linksee_app` 封装了一个原生 C++ 应用，用于通过
mlink device → mlink gateway → MCP → Hermes 暴露移动抓取相关工具，并将
Linksee 的 host 控制与策略推理封装为可独立启停的本地能力。

## 功能特性

- 提供 `linksee_native` 命令行入口
- 提供 `linksee_device` 设备进程，动态注册 4 个 MCP 工具
- 将移动抓取流程拆分为 host 端与推理端，分别可独立启动与停止
- 支持应用内模型目录 `models/` 和推理数据集目录 `datasets/`
- 支持通过仓库构建系统 `mm` 构建，也支持本地 `cmake` 独立构建

## 前置准备

### 硬件准备与连接

运行 Linksee 前，请先确认机械臂、底盘与相机已经正确上电并连接到当前设备。

- 机械臂：确认电源、通信线缆和控制接口连接正常，并确认 host 控制链路可正常工作。
- 底盘：确认底盘串口、驱动板与控制接口连接正常，并根据实际设备记录串口号。
- 相机：确认相机已正确接入，能够满足 Linksee 推理流程采集图像的要求。

### 模型准备

Linksee 推理依赖本地策略模型。首次运行前，请先准备好推理所需模型文件。

- 请根据当前 Linksee 抓取流程准备对应模型和相关配置文件。
- 建议将模型放置到应用约定目录：
    `models/mars_act_pick_place_move_v2/checkpoints/100000/pretrained_model`
- 若后续脚本参数或模型版本发生变化，请以实际脚本配置和运行日志为准。

模型准备完成后，建议先检查：

- 模型文件是否完整可读
- 模型路径是否与 `scripts/evaluate.py` 中的实际配置一致
- 当前运行环境是否满足推理依赖

### Python 虚拟环境准备

请先在仓库根目录完成整体 SDK 环境准备，并确保以下依赖已可用：

- `cmake`
- `python3`
- 已执行项目根目录环境脚本：`source build/envsetup.sh`
- `mlink gateway` 已可正常启动
- 可通过 `mlink gateway start` 启动或重启网关，并通过 `mlink gateway tools` 查看当前网关支持的工具

由于 Linksee host 与推理流程依赖 Python 环境，首次运行前需要准备当前应用的虚拟环境：

```bash
m_env_build application/ros2/linksee/linksee_app
```

或者进入应用目录安装：

```bash
cd application/ros2/linksee/linksee_app
bash scripts/setup_env.sh
```

脚本会在仓库根目录下准备虚拟环境：

```bash
output/envs/linksee_app
```

### 编译 mlink device 依赖

`linksee_device` 在编译时依赖 `mlink` 提供的头文件和动态库。在编译 `linksee` 相关程序之前，先完成
`mlink device` 的构建：

```bash
cd components/agent_tools/mlink/device
mm
```

构建完成后，通常可在以下位置看到关键产物：

```bash
output/staging/include/mlink.h
output/staging/lib/libmlink_device.so
output/staging/bin/mlink_device_test
```

其中：

- `output/staging/include/mlink.h`：供 `linksee_device` 编译时引用的头文件。
- `output/staging/lib/libmlink_device.so`：供 `linksee_device` 链接及运行时加载的动态库。
- `output/staging/bin/mlink_device_test`：`mlink device` 组件自带的测试程序，可用于基础功能验证。

## 快速开始

### 构建编译

仓库内构建：

```bash
cd spacemit_robot
source build/envsetup.sh
cd application/ros2/linksee/linksee_app
mm
```

本地独立构建：

```bash
cd application/ros2/linksee/linksee_app
cmake -B build -S .
cmake --build build
```

构建产物：

- `output/staging/bin/linksee_native`
- `output/staging/bin/linksee_device`

### 运行示例

直接命令行调用：

```bash
linksee_native tool-start-host
linksee_native tool-start-inference
linksee_native tool-stop-host
linksee_native tool-stop-inference
```

启动 mlink device：

```bash
linksee_device unix linksee
```

注册成功后，可在 gateway 中看到：

- `linksee.start_host`
- `linksee.start_inference`
- `linksee.stop_host`
- `linksee.stop_inference`

## 详细使用

### 目录结构

```text
linksee_app/
├── CMakeLists.txt              # CMake 构建与安装入口
├── LICENSE                     # Apache-2.0 许可证文本
├── NOTICE                      # 第三方依赖与版权声明
├── README.md                   # 组件使用说明与发布文档
├── package.xml                 # 包元数据与依赖声明
├── pyproject.toml              # Python 依赖与虚拟环境配置
├── datasets/                   # 推理阶段临时评测数据集目录
├── models/                     # Linksee 模型权重与配置目录
├── include/linksee_app/        # 对外暴露的 C++ 头文件目录
│   └── runner.hpp              # Runner 类声明，封装脚本路径与命令执行接口
├── scripts/                    # Host/推理启动停止及 Python 业务脚本
│   ├── common.py               # Python 公共路径、日志、PID 等辅助函数
│   ├── evaluate.py             # Linksee 推理入口脚本
│   ├── run_host.py             # Linksee host 启动与信号处理脚本
│   ├── setup_env.sh            # 创建并校验 `output/envs/linksee_app` 虚拟环境
│   ├── start_host.sh           # 启动 host 进程
│   ├── start_inference.sh      # 启动推理进程
│   ├── stop_host.sh            # 停止 host 进程
│   └── stop_inference.sh       # 停止推理进程
└── src/cpp/
    ├── main.cpp                # `linksee_native` 命令行入口
    ├── linksee_device.cpp      # mlink device 入口与工具注册实现
    └── runner.cpp              # Runner 类实现，负责调用 shell 脚本
```

### 工具说明

- `linksee.start_host`：启动 Linksee host 进程
- `linksee.start_inference`：启动 Linksee 推理进程
- `linksee.stop_host`：停止 Linksee host 进程
- `linksee.stop_inference`：停止 Linksee 推理进程

### 整体链路

完整执行链路如下：

1. 编译 `application/ros2/linksee/linksee_app`
2. 启动 `linksee_device`
3. `linksee_device` 作为 mlink 设备连接到 `mlink gateway`
4. gateway 对设备执行 `initialize` 和 `tools/list`
5. gateway 将设备工具动态注册为 MCP tool
6. Hermes 连接 gateway 后看到 `linksee.start_host`、`linksee.start_inference`、`linksee.stop_host`、`linksee.stop_inference`
7. Hermes 发起对应工具调用
8. `linksee_device` 回调 `linksee_native` 对应命令
9. `linksee_native` 调用 `scripts/start_host.sh`、`scripts/start_inference.sh`、`scripts/stop_host.sh` 或 `scripts/stop_inference.sh`
10. Shell 脚本进一步调用 `run_host.py` 或 `evaluate.py` 执行真实 host 与推理流程

### tool 注册说明

`linksee_device` 启动时会在设备侧调用：

- `mlink_tool_create("start_host", ...)`
- `mlink_tool_create("start_inference", ...)`
- `mlink_tool_create("stop_host", ...)`
- `mlink_tool_create("stop_inference", ...)`
- `mlink_server_add_tool(server, tool)`

因此无需修改 gateway 内建工具代码。只要设备成功连接 gateway，tool 就会通过运行时发现机制自动
暴露为：

```text
<device_id>.<tool_name>
```

例如设备名为 `linksee` 时，最终 tool 名为：

```text
linksee.start_host
linksee.start_inference
linksee.stop_host
linksee.stop_inference
```

### Hermes 调用说明

Spacemit 平台已适配 Hermes，可直接安装并完成基础配置。

安装 Hermes：

```bash
sudo apt-get update
sudo apt-get install --reinstall hermes-agent
```

配置模型：

```bash
hermes model
```

随后按照命令行向导完成秘钥和模型配置。

启动交互式 CLI：

```bash
hermes
```

当 gateway 已运行且 `linksee_device` 已连接后，Hermes 就能通过 MCP 看到 Linksee 相关工具。

若需要将本机 gateway 的 HTTP MCP endpoint 写入 Hermes 配置，可参考如下方式维护
`~/.hermes/config.yaml`，使其指向当前运行中的 mlink gateway。

示例配置如下：

```yaml
mcp_servers:
    mlink-gateway:
        transport: http
        url: http://127.0.0.1:18765/mcp
        enabled: true
```

当 Hermes 绑定该 MCP 服务后，即可在 Hermes CLI 中使用自然语言发起指令，例如“启动 Linksee host”
或“停止 Linksee 推理”。Hermes 会将该请求路由到 `linksee.start_host`、`linksee.stop_inference` 等 tool。

### 模型与数据集约定

- 推理模型目录：
    `models/linksee_act_pick_place_move_v2/checkpoints/100000/pretrained_model`
- 推理数据集目录：
    `datasets/linksee_pick_place_move_v2_eval`
- 每次启动推理前，会删除已有的推理数据集目录，避免旧数据混入新结果

### 日志与 PID

- host 日志：`/tmp/linksee_host.log`
- host pid：`/tmp/linksee_host.pid`
- inference 日志：`/tmp/linksee_inference.log`
- inference pid：`/tmp/linksee_inference.pid`

## 常见问题

### 1. `linksee_device` 启动后 gateway 看不到工具

请确认：

- `mlink gateway` 已正常启动
- 使用的设备名为 `linksee`
- 启动命令为 `linksee_device unix linksee`

### 2. 推理启动失败，提示模型目录不存在

请确认模型已放入：

`models/linksee_act_pick_place_move_v2/checkpoints/100000/pretrained_model`

### 3. 停止 host 后机械臂未释放

当前 host 通过 `run_host.py` 接管信号处理，优先发送 `SIGINT` 触发 Python 清理逻辑；
如超时未退出，再回退到强制停止。

### 4. Hermes 中看不到 Linksee 工具

请依次检查：

- `mlink gateway` 是否已启动
- `linksee_device` 是否已运行
- Hermes 配置中的 MCP 地址是否指向 `http://127.0.0.1:18765/mcp`
- `/tmp/mlink-gateway/gateway.log` 中是否出现设备注册成功日志

## 版本与发布

版本以本组件文档或仓库 tag 为准。

| 版本  | 说明                                                   |
|-------|--------------------------------------------------------|
| 1.0.0 | 初始版本，提供 `linksee_native`、`linksee_device` 与 Linksee host/推理工具链 |

## 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

1. C/C++ 代码遵循 [Google C++ 风格](https://google.github.io/styleguide/cppguide.html)
2. Python 代码遵循 [PEP 8](https://peps.python.org/pep-0008/)
3. Git commit 遵循 [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。