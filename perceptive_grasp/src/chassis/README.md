# 内置底盘驱动

本目录是 `spacemit-robotics/base`（`components/control/base`）中底盘控制核心与
UART 驱动源码的副本，随 `perceptive_grasp` 一起编译，因此部署时不再需要
`output/staging/lib/libchassis.so`。

| 文件 | 上游路径 |
|---|---|
| `chassis_core.c` / `chassis_core.h` | `src/chassis_core.c` / `src/chassis_core.h` |
| `drivers/drv_uart_esp32.c` | `src/drivers/drv_uart_esp32.c` |
| `../../include/chassis/chassis.h` | `include/chassis.h` |

## 为什么要内置

上游组件库长期只注册 `drv_rpmsg_esos`，且 `struct chassis_config` 没有
`left_wheel_gain`。Linksee 真机底盘走 UART，需要 `drv_uart_esp32` 和该补偿系数，
因此这里直接内置包含这两者的新版源码。驱动依赖 `chassis_core.h` 的私有结构
（`struct chassis_dev`、驱动注册表），无法只拷单个驱动文件。

## 与上游的差异

只改了一处 include 路径，使其适配本仓库的目录布局：

- `chassis_core.h`：`#include "../include/chassis.h"` → `"../../include/chassis/chassis.h"`

其余内容与上游保持一致。同步上游更新时，先比对这三处源文件，
再确认 include 路径改动是否需要保留。

## 许可

上游与本副本均为 Apache-2.0，各文件头部保留原始版权声明。
