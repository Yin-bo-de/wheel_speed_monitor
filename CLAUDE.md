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
- `main/idf_component.yml` 钉住 `espressif/esp32-camera`；首次构建需要联网拉
  `managed_components/`（已 gitignore）。PSRAM（octal/80MHz）与 `CAMERA_CORE1`
  固化在 `sdkconfig.defaults`，相机缓冲走 PSRAM。

## 代码组织纪律

**新增模块（采集器、驱动、配置项、任务）必须先对齐下面的分层，不要另起一套。**
偏离需有明确理由，并在改动说明里写清为什么。

- **纯算的与碰硬件的分开。** 逻辑/数学/状态机写纯 C 组件——**不依赖 IDF**（可宿主机测），
  头文件 include `esp_err.h`（宿主命中 tests/native/include 的 shim，
  IDF 命中 esp_common 的真实 esp_err.h）；硬件驱动单独成组件（依赖 esp_driver_*，
  不宿主机测）。现有对照：`wheel_speed`（纯逻辑）↔ `wheel_sensor`（硬件层），
  `servo_act`（纯逻辑）↔ `servo_drv`（硬件层）。
- **新数据走 telemetry 采集器框架**：按 README「扩展」四步接
  （`telem_type_t` 加枚举 → 实现 ops → `app_main` 注册 → 前端 `applyFrame` 渲染），
  不绕过框架自己推数据。
- **新配置进 `cfg_params`**：纯 C 模型 + 逐字段校验 + blob pack/unpack；blob 字段
  变更必须升版本号并兼容旧版。NVS 读写只经 `config_store` 薄封装，别处不直接调
  nvs API。
- **不轻易加任务**：先看上面任务表能否挂载（重逻辑 Core0 / httpd Core1）；
  确实要加则同步更新本文件的任务表。
- `main/CMakeLists.txt` 的 `SRCS`/`REQUIRES` 显式清单；新增 .c 必须加进 SRCS
  （漏了最常见的报错是 `undefined reference`）。EMBED_FILES 嵌入 www 资源。
  宿主机可测的 .c 还要进 `tests/native/CMakeLists.txt` 的对应 SRCS 分组。
- 宏约定：`ESP_RETURN_ON_ERROR(x, TAG, "msg")`（4 参 IDF 签名，TAG 每文件定义）；
  函数小写蛇形、返回 esp_err_t、错误通路宏统一。
- 注释解释"为什么"而非"做什么"。中文或英文保持一致即可（当前用中文）。

## 任务表（改任务配置前先查此表）

| 任务 | 核心 | 优先级 | 栈 | 说明 |
|---|---|---|---|---|
| sampler_task | Core0 | 22 | 3072 | 50ms 周期采样：读 4 路 PCNT + GPIO 电平 → 采集器运算 → 遥控仲裁 → 舵机控制链 |
| ws_push_task | Core0 | 15 | 8192 | 100ms 推送遥测帧（单任务 WS 发送） |
| httpd server | Core1 |  5 | 8192 | 静态页/API/WS 命令解析；不跑 MJPEG 循环 |
| cam_publish_task | Core1 | 6 | 6144 | 相机 init/deinit/取帧/发布缓冲/预览计时；唯一碰相机句柄的任务 |
| cam_stream_task | Core1 | 5 | 4096 | 接管 `/cam/stream` 异步请求，按帧率上限发 MJPEG |
| esp32-camera 内部任务 | Core1 | 23（库固定） | 4096 | DMA 取帧；必须由 `CONFIG_CAMERA_CORE1=y` 钉到 Core1，否则抢 Core0 采样 |

- 双核分工：重逻辑/分发留 Core0；httpd 挂 Core1（httpd_config_t.core_id）。
- **舵机控制链（strategy → servo_act/servo_drv）只跑在 sampler_task 里**，
  httpd 只改配置数据、不碰执行器句柄；执行器在模拟与 LEDC 之间的切换也在这里做
  （`servo_act_select`），这样 LEDC 的初始化与寄存器写全落在单任务上下文，不用加锁。
- **遥控捕获（rc_capture）同理**：GPIO 配置与中断挂载在采样任务里惰性做
  （`rc_capture_select`），httpd 只改 `rc_sim` / `rc_sim_us` 这两个配置项。
- **相机句柄只在 `cam_publish_task` 里碰**：`esp_camera_init/deinit/fb_get/fb_return`
  全在这个任务；httpd 和 `cam_stream_task` 只发请求、读发布缓冲。发布缓冲由短锁保护，
  网络发送不持锁。MJPEG 发送循环不能放进 httpd handler（httpd 单任务，会把页面/API
  一起堵死）；`/cam/stream` 用 `httpd_req_async_handler_begin()` 交给流任务。
  采集生命周期（on_demand/always/off、切 tab 保持 30 秒、断流宽限）见
  `docs/CAMERA_DESIGN.md` 4.5.1，改之前先读。相机不是采集器：像素走自己的
  HTTP 端点，只有状态和配置回显进 `telemetry` 的 `sys.cam`。
- 队列/锁都在 main/ 静态共享（bench_http_server 经 ctx 引用）：
  - `s_cfg_mutex`（配置）、`s_frame_mutex`（最新帧）、`s_clients_mutex`（WS 客户端表）
  - 共享结构见 `main/wheel_bench.h`（bench_server_ctx_t）

## 引脚表（动硬件前核对）

| 用途 | GPIO | 备注 |
|---|---|---|
| 4 路霍尔输入 | 1 / 14 / 21 / 47 | 摄像机占 4-18、PSRAM 35-37、USB 19/20、
  UART 43/44、WS2812=48、GPIO2 板载 LED、strapping 0/3/45/46 → 仅剩这 4 个空闲
  （SD 的 38-40 后来让给了遥控输入） |
| 差速舵机输出 | 41 / 42 | 前 / 后差速，LEDC 50Hz。是 S3 的 JTAG 脚（MTDI/MTMS），
  但默认调试通道是芯片内置 USB-Serial-JTAG，走内部 TAP、不占引脚
  （IDF 文档 jtag-debugging/tips-and-quirks），所以直接用；只有烧了
  DIS_USB_JTAG efuse 把 JTAG 引到引脚上才会冲突 |
| 遥控 PWM 输入 | 38 / 39 / 40 | 模式 / 前差速 / 后差速，GPIO 双沿中断测脉宽。
  原属板载 microSD 卡座，本项目不用 SD，**代价是卡座不再可用**；
  39/40 兼 JTAG 脚（MTCK/MTDO），与 41/42 同理。没用 3/45/46 三个净空脚：
  45 上电被采到高会把 flash 电压切到 1.8V 直接起不来，46 与 BOOT 键的
  组合影响下载模式 |
| 板载摄像头 DVP | 4,5,6,7,8,9,10,11,12,13,15,16,17,18 | SCCB 4/5、VSYNC 6、HREF 7、
  PCLK 13、XCLK 15、Y2..Y9 = 11/9/8/10/12/18/17/16。**14 是霍尔，不属于摄像头**。
  摄像头启用后 4/5 被 SCCB 占用；J13/J14 只是 40 针 1:1 扩展，不是额外空闲脚 |

## 组件职责图

```
main（编排/任务/WiFi/httpd）
 ├─ telemetry        采集框架：注册表、采样泵、帧聚合（纯 C）
 │  └─ wheel_speed   轮速采集器：数学链/回绕判定/模拟源/JSON 渲染（纯 C）
 ├─ wheel_sensor     PCNT 硬件层（IDF 依赖）
 ├─ strategy         差速状态机：打滑判定 / 锁定保持 / 试探退避 / 脉宽映射（纯 C）
 │  └─ strategy_render  舵机 JSON 块渲染（纯 C）
 ├─ servo_act        舵机执行器接口 + 模拟模型（纯 C）
 ├─ servo_drv        LEDC 硬件输出（IDF 依赖）
 │                   └─ 与 servo_act 共用 servo_act_ops_t，按配置二选一
 ├─ rc_input         遥控判定与仲裁：通道有效性 / 模式判定 / 透传回落（纯 C）
 │  └─ rc_render        遥控 JSON 块渲染（纯 C）
 ├─ rc_capture       GPIO 双沿中断测遥控脉宽（IDF 依赖）
 │                   └─ 与验证台模拟注入共用同一个 rc_input 模型，按配置二选一
 ├─ camera_drv       板载 DVP 相机的 esp32-camera 薄封装（IDF 依赖）
 └─ bench_cam        相机生命周期 / 发布缓冲 / cam_publish_task（IDF 依赖）
    └─ bench_cam_stream  /cam/stream 异步 MJPEG 与 /cam/snapshot（IDF 依赖）
```

新增采集器步骤见 README「扩展」。
