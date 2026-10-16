# BionicFish

基于 ESP32-C5 与 ESP-IDF 的仿生鱼控制固件。项目使用 BMI270 获取姿态，驱动三关节尾鳍，并采集模拟 pH/温度传感器数据；支持 Wi-Fi 云端数据上报与指令轮询，以及本地 HTTP 控制接口。

## 功能

- 通过三路 PWM 控制尾鳍舵机，使用 CPG 行波算法生成游动动作，并对速度、转向和动作过渡进行平滑处理。
- 读取 BMI270 姿态数据，根据姿态状态控制游动；BMI270 初始化失败时进入演示模式。
- 读取 pH 电压、温度电压和数字输入，带有截尾平均与 EMA 平滑处理，并支持通过代码接口校准。
- 通过 Wi-Fi 周期性向云端上报数据并轮询控制命令。
- 启动本地 HTTP 服务，提供状态查询和方向控制接口。

## 硬件与引脚

| 模块 | ESP32-C5 GPIO | 说明 |
| --- | ---: | --- |
| BMI270 SCL | 9 | 软件模拟 I2C，100 kHz |
| BMI270 SDA | 8 | 默认 I2C 地址 `0x68` |
| BMI270 INT | 14 | 中断引脚配置 |
| 尾鳍舵机关节 0 | 4 | LEDC 通道 0 |
| 尾鳍舵机关节 1 | 5 | LEDC 通道 1 |
| 尾鳍舵机关节 2 | 6 | LEDC 通道 2 |
| pH 模拟输入 | 1 | ADC1 通道 1 |
| 温度模拟输入 | 2 | ADC1 通道 2 |
| pH 数字输入 | 7 | GPIO 输入 |

舵机 PWM 配置为 50 Hz，脉宽范围 500–2500 μs，角度范围按配置限制为 ±90°。接线前请确认传感器模块电平、供电和 ESP32-C5 板卡引脚映射；舵机应使用能满足其电流需求的独立电源，并与控制板共地。

## 软件要求

- ESP-IDF 6.0.2（`sdkconfig.defaults` 中记录的版本）
- 支持 ESP32-C5 的 ESP-IDF 工具链
- USB 串口连接，用于烧录和查看日志

## 配置与烧录

先在 `main/fish_config.h` 中配置 Wi-Fi、服务器和硬件参数：

- `WIFI_SSID`、`WIFI_PASSWORD`：设备连接的 Wi-Fi。
- `CLOUD_SERVER_IP`、`CLOUD_SERVER_PORT`：云端服务地址。
- BMI270、舵机和 pH 传感器的 GPIO、校准及运动参数。

不要将真实 Wi-Fi 密码或私有服务器信息提交到公开仓库。可先使用占位值，再按本地网络环境配置。

在已配置 ESP-IDF 环境的终端中，从项目根目录运行：

```bash
idf.py set-target esp32c5
idf.py build
idf.py -p PORT flash monitor
```

将 `PORT` 替换为开发板对应的串口，例如 Windows 下的 `COM5`。如果 `sdkconfig` 已配置目标芯片，通常无需重复执行 `set-target`。退出串口监视器可按 `Ctrl+]`。

## 本地 HTTP API

设备连接 Wi-Fi 后，串口日志会打印设备 IP。使用该 IP 访问以下接口，默认 HTTP 端口为 80。

| 方法与路径 | 说明 |
| --- | --- |
| `GET /api/status` | 返回当前状态 JSON，例如 `{"do":2.5,"ph":7.00,"roll":0.0,"pitch":0.0,"state":"LEVEL"}`。 |
| `GET /action?go=F` | 前进 |
| `GET /action?go=L` | 左转 |
| `GET /action?go=R` | 右转 |
| `GET /action?go=U` | 上浮 |
| `GET /action?go=D` | 下潜 |
| `GET /action?go=AUTO` | 解除手动覆盖，恢复姿态自动控制 |
| `GET /action?go=S` | 清除本地手动覆盖，恢复姿态自动控制 |

本地接口使用表中的命令码，命令区分大小写。例如：

```text
http://<设备IP>/action?go=F
http://<设备IP>/api/status
```

本地命令优先于云端命令。注意，当前 `/action?go=S` 的实现是清除本地覆盖并返回自动控制，并不等同于强制停止舵机。

## 云端协议

固件默认访问 `http://<CLOUD_SERVER_IP>:<CLOUD_SERVER_PORT>`：

- 每隔 `CLOUD_UPLOAD_INTERVAL_MS` 向 `POST /api/upload` 发送 JSON：`{"do":2.5,"ph":7.00,"roll":0.0,"pitch":0.0}`。
- 每隔 `CLOUD_POLL_INTERVAL_MS` 请求 `GET /api/get_cmd`，从响应 JSON 的 `cmd` 字段读取命令，例如 `{"cmd":"FORWARD"}`。
- 支持的命令包括 `F`/`FORWARD`、`L`/`LEFT`、`R`/`RIGHT`、`S`/`STOP`、`AUTO`、`U`/`UP`、`D`/`DOWN`。

云端服务需自行实现上述 HTTP 接口。当前固件中上传数据里的 `do` 字段固定为 `2.5`，并非来自溶解氧传感器；本地状态接口中的该字段同样是固定值。

## 代码结构

| 文件 | 职责 |
| --- | --- |
| `main/main.c` | 初始化外设、Wi-Fi/HTTP 服务及 FreeRTOS 任务 |
| `main/fish_control.c`、`main/fish_motion.c` | 控制状态机、外部命令仲裁和 CPG 尾鳍运动 |
| `main/fish_servo.c` | 三路舵机 PWM 输出 |
| `main/bmi270.c`、`main/bmi270_driver.c`、`main/bmi2.c` | BMI270 驱动与姿态传感器访问 |
| `main/software_i2c.c` | 软件模拟 I2C |
| `main/ph_sensor.c` | ADC 采样、滤波、pH/温度换算与校准 |
| `main/wifi_connector.c` | Wi-Fi STA 连接 |
| `main/cloud_client.c` | 云端 HTTP 上报与命令轮询 |
| `main/local_server.c` | 本地 HTTP 控制与状态查询 |
| `main/fish_config.h` | 引脚、网络、校准和运动参数 |

## 当前实现说明

- 未检测到 BMI270 时，控制任务切换为演示模式；这不代表姿态传感器功能正常。
- pH 换算参数及温度传感器换算参数是 `fish_config.h` 中的默认校准值，实际使用前应使用已知标准液/温度源校准。
- `PH_TEMP_COEFF` 当前为 `0.0`，因此 pH 计算暂未启用温度补偿。
- Wi-Fi 连不上时本地 HTTP 服务仍会尝试启动，但云端上报和命令轮询不可用。
- 云端使用 HTTP 明文传输，且 `sdkconfig.defaults` 禁用了 HTTPS。