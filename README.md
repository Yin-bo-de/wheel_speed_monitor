# wheel_speed_monitor

RC 攀爬车四轮转速监测验证台（v1）。

A3144 霍尔传感器 + 小铷磁铁随车轮转动触发脉冲，ESP32-S3 用 PCNT 硬件计数，
经 WebSocket 推送遥测帧到内嵌网页验证台。采集器框架可扩展（后续加 IMU 姿态等）。

## 硬件与接线

板卡：Freenove ESP32-S3 WROOM CAM（FNK0085）。摄像头已接线但 v1 不启用。

| 轮 | 霍尔 GPIO | 说明 |
|---|---|---|
| 左前 | GPIO1 | |
| 右前 | GPIO14 | 闪光灯焊盘可选接口，不用 |
| 左后 | GPIO21 | |
| 右后 | GPIO47 | |

A3144 接线：VCC ← ESP32 3.3V（**注意：数据手册标称 4.5–24V，3.3V 低于规格下限，
可能工作但不保证**）、GND ← GND、OUT → GPIO 并 10kΩ 上拉到 3.3V（面板已加）。
磁铁南极靠近 → OUT 拉低（下降沿 = 一次触发）。

### 3.3V 供电排查预案

1. 打开验证台「原始 IO 电平监视」：
   - 静止时 4 路全"高" = 上拉有效；某路非高 → 查该路接线/上拉电阻。
   - 磁铁靠近时对应 GPIO 闪"低" = 触发正常。
2. 静止高、磁铁靠近不闪低 → 万用表量 A3144 VCC 对 GND 电压：
   - 不足 3.1V 且无动作 → **改 5V 供电**：VCC 改接 ESP32 5V 引脚；
     **输出上拉保持接 3.3V**（A3144 是开集电极，上拉电压决定 GPIO 高电平，安全）。
3. 翻转但 RPM 乱跳 → 磁铁南极面朝向传感器、间距 < 5mm；网页「去抖」填 20 开软件去抖。

## 构建与烧录

```bash
# 环境加载（每次新终端）
get_idf

# 首次
idf.py set-target esp32s3
idf.py build

# 烧录（端口先 ls /dev/cu.*）
idf.py -p /dev/cu.usbmodemXXX flash monitor
```

## 使用

1. 烧录后手机/电脑连 WiFi 热点 `wheel-bench`（开放，无密码）。
2. 浏览器打开 `http://192.168.4.1`。
3. 页面显示 4 轮 RPM/线速度/频率/累计脉冲 + 原始 IO 电平监视 + 轮速曲线。
4. 配置（磁铁数/轮径/EMA/每轮启用）自动持久化到 NVS；「模拟模式」用合成
   数据验证显示与计算链路（走与真实霍尔完全相同的数据链）。

## 宿主机单元测试

```bash
cmake -S tests/native -B tests/native/build
cmake --build tests/native/build
ctest --test-dir tests/native/build --output-on-failure
```

纯逻辑层（数学、回绕判定、模拟源、采集框架、JSON 渲染、配置校验、策略桩）
在 macOS 用 clang + Unity 跑；硬件层（PCNT/NVS/WiFi）与页面是集成分层，
由板上验证台验证。

## 目录结构

```
main/           入口、配置、Web 服务、内嵌验证台页
components/
  telemetry/    采集框架（注册表/采样泵/帧聚合），可扩展采集器
  wheel_speed/  轮速采集器实现（数学链/回绕/模拟源/JSON 渲染）
  wheel_sensor/ PCNT 硬件薄封装（A3144 4 路）
  strategy/     差速策略接口桩（二期：PWM 输入/舵机输出）
tests/native/   宿主机单测（clang + Unified + ctest）
```

## 扩展：新增采集器（如 IMU）

1. `telemetry.h` 的 `telem_type_t` 末尾加枚举值；
2. 实现 `telem_collector_ops_t`（init/sample/render/reset_baseline）；
3. `app_main` 里 `telem_register` 注册；
4. 前端 `applyFrame` 的 collectors 遍历加一个渲染函数。

协议约定：`sample` 每采样窗口调用一次（50ms），计算量须小；`render` 输出本类型
JSON 对象（不含外层包裹），字段名即前端协议，变更需前后端同步。

## 二期规划（v1 未实现）

- 遥控接收机 PWM 输入（RMT 捕获）→ 手动模式直控舵机；
- 前后差速策略（依据四轮 + 姿态判定差速驱动 GPIO41/42 舵机输出）；
- 姿态（IMU）采集器。

## v1 范围外

摄像头 OV5640、OTA、STA 联网、密码认证、移动端专项适配。
