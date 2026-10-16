#ifndef FISH_CONFIG_H
#define FISH_CONFIG_H

/* ================================================================
 *   BionicFish - 硬件引脚配置 (ESP32-C5 + BMI270 + pH传感器)
 *   硬件: 3x DS-SERVO RDS3125 多关节尾鳍 + BMI270 姿态传感器 + pH检测
 * ================================================================ */

/* --- BMI270 姿态传感器 (软件模拟I2C) --- */
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_FREQ_HZ        100000
#define BMI270_I2C_ADDR            0x68
#define BMI270_INT_PIN              14

/* --- 舵机 PWM 通道 (3舵机多关节尾鳍: 近端→远端) --- */
#define SERVO_JOINT_0_PIN           4
#define SERVO_JOINT_1_PIN           5
#define SERVO_JOINT_2_PIN           6

/* --- LEDC 定时器配置 --- */
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ            50
#define LEDC_RESOLUTION         LEDC_TIMER_16_BIT

#define LEDC_CH_JOINT_0         LEDC_CHANNEL_0
#define LEDC_CH_JOINT_1         LEDC_CHANNEL_1
#define LEDC_CH_JOINT_2         LEDC_CHANNEL_2

/* --- 舵机角度范围 (DS-SERVO RDS3125, 35Kg) --- */
#define SERVO_MIN_PULSE_US      500
#define SERVO_MAX_PULSE_US      2500
#define SERVO_MAX_ANGLE_DEG     90
#define SERVO_CENTER_US         1500

/* 舵机微调 (度): 补偿机械安装偏差, 正=右偏 负=左偏 */
#define SERVO_TRIM_J0            0.0f
#define SERVO_TRIM_J1            0.0f
#define SERVO_TRIM_J2            0.0f

/* ================================================================
 *   CPG 行波参数 (多关节尾鳍行波推进)
 * ================================================================ */

/* 关节数量 */
#define TAIL_JOINT_COUNT         3

/* 各关节基础振幅 (度) - 从近端到远端递增 */
#define AMP_JOINT_0             20.0f
#define AMP_JOINT_1             30.0f
#define AMP_JOINT_2             35.0f

/* 相位延迟系数: 从头到尾的相位滞后 (弧度/关节) */
#define PHASE_LAG_PER_JOINT     0.7f

/* 游泳频率 (Hz) */
#define CPG_FREQ_CRUISE          1.5f

/* 转向偏移量 (度): 通过整体偏移实现转向 */
#define TURN_OFFSET_DEG          15.0f

/* 运动过渡平滑速度 (单位/秒): 值越大过渡越快 */
#define MOTION_TRANSITION_SPEED   8.0f

/* 转向过渡平滑速度 (单位/秒): 比振幅过渡更慢, 转弯更柔和 */
#define TURN_TRANSITION_SPEED     3.0f

/* 速度斜坡速率 (单位/秒): 加速/减速的平滑速率 */
#define SPEED_RAMP_RATE           2.0f

/* 姿态死区 (度): 在此范围内视为 LEVEL, 防止噪声触发状态切换 */
#define ATTITUDE_DEAD_ZONE        3.0f

/* 舵机最大速度 (度/秒): RDS3125 约 500°/s, 留余量 */
#define SERVO_MAX_SPEED_DEG_S     300.0f

/* 舵机滞后阈值 (度): 估算位置落后目标超过此值则降低幅度 */
#define SERVO_LAG_THRESHOLD_DEG   5.0f

/* 幅度柔顺系数: 舵机滞后时幅度缩放的下限 */
#define SERVO_COMPLIANCE_MIN      0.5f

/* ================================================================
 *   系统参数
 * ================================================================ */

/* 控制循环频率 (Hz) */
#define CONTROL_LOOP_FREQ_HZ    100
#define CONTROL_LOOP_PERIOD_MS  (1000 / CONTROL_LOOP_FREQ_HZ)

/* 姿态阈值 */
#define PITCH_DIVE_THRESHOLD    15.0f
#define PITCH_SURFACE_THRESHOLD (-15.0f)
#define ROLL_LEFT_THRESHOLD     20.0f
#define ROLL_RIGHT_THRESHOLD    (-20.0f)

/* 状态机迟滞裕度: 退出状态需要的回退量, 防止阈值边界振荡 */
#define HYSTERESIS_MARGIN        5.0f

/* 状态去抖动次数: 连续多少次采样确认后才切换状态 */
#define STATE_DEBOUNCE_COUNT     8

/* 任务栈大小 */
#define TASK_STACK_CONTROL      4096
#define TASK_STACK_SERVO        3072
#define TASK_STACK_CLOUD        4096
#define TASK_STACK_LOCAL_SERVER 4096

/* --- pH 检测模块 (模拟pH传感器) --- */
#define PH_ADC_UNIT             ADC_UNIT_1
#define PH_ADC_CHANNEL          ADC_CHANNEL_1
#define PH_ADC_ATTEN            ADC_ATTEN_DB_12
#define PH_ADC_PIN              1

#define PH_TEMP_ADC_UNIT        ADC_UNIT_1
#define PH_TEMP_ADC_CHANNEL     ADC_CHANNEL_2
#define PH_TEMP_ADC_ATTEN       ADC_ATTEN_DB_12
#define PH_TEMP_ADC_PIN         2

#define PH_DIGITAL_PIN          7

/* pH 校准参数 (pH = slope * voltage + offset) */
#define PH_SLOPE                -5.70f
#define PH_OFFSET               21.34f

/* pH 采样次数 (中值+截尾平均滤波) */
#define PH_SAMPLE_COUNT         20

/* EMA 平滑系数 (0~1, 越小越平滑, 0.15=约6次采样平滑) */
#define PH_EMA_ALPHA            0.15f

/* 温度补偿系数 (pH/°C): 0=不补偿, 校准后设为0.03 */
#define PH_TEMP_COEFF           0.0f

/* 温度传感器校准 (temp_c = slope * voltage + offset) */
#define PH_TEMP_SLOPE           100.0f
#define PH_TEMP_OFFSET          (-60.0f)

/* pH 读取间隔 (ms) */
#define PH_READ_PERIOD_MS       500

/* --- WiFi 配置 --- */
#define WIFI_SSID               "BTWIFI_20F4"
#define WIFI_PASSWORD           "22226666"
#define WIFI_MAX_RETRY          5
#define WIFI_CONNECT_TIMEOUT_MS 15000

/* --- 云端服务器配置 --- */
#define CLOUD_SERVER_IP         "192.168.1.5"
#define CLOUD_SERVER_PORT       5000
#define CLOUD_UPLOAD_INTERVAL_MS 3000
#define CLOUD_POLL_INTERVAL_MS  1000

/* --- 本地 HTTP 控制服务 --- */
#define LOCAL_WEB_SERVER_PORT   80

/* --- 任务优先级 --- */
#define TASK_PRIO_CONTROL       4
#define TASK_PRIO_SERVO         3
#define TASK_PRIO_PH_SENSOR     2
#define TASK_PRIO_CLOUD_UPLOAD  1
#define TASK_PRIO_CLOUD_POLL    1

#endif