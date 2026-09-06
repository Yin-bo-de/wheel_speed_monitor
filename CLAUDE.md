# wheel_speed_monitor 项目规则

## 构建与测试（IDF 规范速查）

```bash
get_idf                                   # 每次新终端先加载 ESP-IDF 环境
idf.py set-target esp32s3                 # 首次或换目标芯片
idf.py build                              # 日常迭代
idf.py -p /dev/cu.usbmodemXXX flash       # 烧录（端口先 ls /dev/cu.*）

# 宿主机单测（TDD 循环入口）
cmake -S tests/native -B tests/native/build
cmake --build tests/native/build
ctest --test-dir tests/native/build --output-on-failure
```

- sdkconfig 不进 git；固化配置写 `sdkconfig.defaults`，每项注释原因。
  改默认值后删 sdkconfig 重生成（set-target 再 build）才完全生效。
- 增量构建经常看不出 Kconfig 变更差异。

## 代码组织纪律

- `main/CMakeLists.txt` 的 `SRCS`/`REQUIRES` 显式清单；新增 .c 必须加进 SRCS
  （漏了最常见的报错是 `undefined reference`）。EMBED_FILES 嵌入 www 资源。
- 纯逻辑层组件（telemetry/wheel_speed/strategy）**不依赖 IDF**（可宿主机测），
  头文件 include `esp_err.h`（宿主命中 tests/native/include 的 shim，
  IDF 命中 esp_common 的真实 esp_err.h）。硬件层（wheel_sensor）依赖
  esp_driver_pcnt/esp_driver_gpio，不宿主机测。
- 宏约定：`ESP_RETURN_ON_ERROR(x, TAG, "msg")`（4 参 IDF 签名，TAG 每文件定义）；
  函数小写蛇形、返回 esp_err_t、错误通路宏统一。
- 注释解释"为什么"而非"做什么"。中文或英文保持一致即可（当前用中文）。

## 任务表（改任务配置前先查此表）

| 任务 | 核心 | 优先级 | 栈 | 说明 |
|---|---|---|---|---|
| sampler_task | Core0 | 22 | 3072 | 50ms 周期采样：读 4 路 PCNT + GPIO 电平 → 采集器运算 |
| ws_push_task | Core0 | 15 | 4096 | 100ms 推送遥测帧（单任务 WS 发送） |
| httpd server | Core1 |  5 | 8192 | 静态页/API/WS 命令解析 |

- 双核分工：重逻辑/分发留 Core0；httpd 挂 Core1（httpd_config_t.core_id）。
- 队列/锁都在 main/ 静态共享（bench_http_server 经 ctx 引用）：
  - `s_cfg_mutex`（配置）、`s_frame_mutex`（最新帧）、`s_clients_mutex`（WS 客户端表）
  - 共享结构见 `main/wheel_bench.h`（bench_server_ctx_t）

## 引脚表（动硬件前核对）

| 用途 | GPIO | 备注 |
|---|---|---|
| 4 路霍尔输入 | 1 / 14 / 21 / 47 | 摄像机占 4-18、SD 38-40、PSRAM 35-37、USB 19/20、
  UART 43/44、WS2812=48、GPIO2 板载 LED、strapping 0/3/45/46 → 仅剩这 4 个空闲 |
| 二期舵机候选 | 41/42 | 默认 JTAG，启用需关 JTAG 重配 |
| 二期 RC 输入 | 未定 | 二期再定 |

## 组件职责图

```
main（编排/任务/WiFi/httpd）
 ├─ telemetry        采集框架：注册表、采样泵、帧聚合（纯 C）
 │  └─ wheel_speed   采集器：数学链/回绕判定/模拟源/JSON 渲染（纯 C）
 ├─ wheel_sensor     PCNT 硬件层（IDF 依赖）
 └─ strategy         差速策略桩（二期：PWM 输入/舵机输出）
```

新增采集器步骤见 README「扩展」。
