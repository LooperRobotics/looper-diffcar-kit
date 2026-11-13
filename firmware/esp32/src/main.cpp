// ESP32-S3-DevKitC-1 (WROOM-1 N16R8) + TB6612(D153C) 差速小车
// 双轮正交编码器(PCNT 硬件解码)闭环调速 + 差速运动学 + 里程计，串口敲命令，输入 ? 看帮助
#include <Arduino.h>
#include "esp_heap_caps.h"
#include <Preferences.h>
#include <Update.h>
#include "driver/pcnt.h"
#include "esp32-hal-rmt.h"
#include "esp_system.h"

// ---- 引脚 ----
// 禁用：26-32(SPI flash)、33-37(八线 PSRAM，右排丝印上 35/36/37 看着能用其实会崩)、
//       45(VDD_SPI 选压，复位时为高则 flash 起不来)、22-25(芯片上不存在)
// 占用：19/20(原生 USB)、43/44(UART0 → CH343)
constexpr int PIN_E2B = 4, PIN_E2A = 5;   // 左轮编码器
constexpr int PIN_E1B = 6, PIN_E1A = 7;   // 右轮编码器
constexpr int PIN_PWMB = 15, PIN_BIN2 = 16, PIN_BIN1 = 17;  // B 通道 = 左轮
// A 通道故意排在 11-13：它们属于 ADC2，而 ADC2 开 WiFi 就废，当数字口用是零损失；
// 换来的是把 GPIO8/9 这两个 ADC1 脚腾出来给模拟量(如电机电流采样)。三个孔在排针上还是连着的
constexpr int PIN_AIN1 = 11, PIN_AIN2 = 12, PIN_PWMA = 13;  // A 通道 = 右轮
// STBY 放 GPIO46：它是 strapping 脚，复位期间芯片内部挂弱下拉 → 带电复位时驱动自动断电。
// 但掉压塌电时芯片失去供电，内部下拉一起消失、GPIO46 真悬空，而 TB6612 可能还活着 ——
// 所以外面仍要接一个 10kΩ 到 GND，否则会"重启后突然窜出去"。实测过
constexpr int PIN_STBY = 46;
constexpr int PIN_VBAT = 10;                    // ADC1_CH9，接 D153C 的 ADC 脚
// UART1 → X5，按实际焊接：X5 的 TX 焊在 GPIO40、RX 焊在 GPIO41。
// 也就是 GPIO40 是我们的 RX、GPIO41 是我们的 TX —— 角色是交叉的，别按引脚号顺手对应
constexpr int PIN_X5_RX = 40, PIN_X5_TX = 41;
// DevKitC-1 的板载 WS2812 在不同版本上接 48 或 38，两个都驱动，省得分辨版本
constexpr int PIN_RGB_A = 48, PIN_RGB_B = 38;

// ---- 机械参数 ----
// 两个都是卷尺量的标称值，落地后要按"走直线 2m"和"原地转 10 圈"重标，
// 轮胎压扁会让实际周长偏小，轮胎接地是一个面而不是线会让实际轮距偏大
//
// 2026-08-21 前驱改装后重量：轮径 65->70mm，轮距 110->320mm。轮距那一项差 2.91 倍，
// 而它同时进 setTargets 和里程计，所以旧值让偏航指令只实现 1/2.91、里程计的 theta
// 又放大 2.91 倍 —— 之前"原地转角度不足"是这个，不是脚轮拖曳。
//
// 2026-08-21 落地定标。轮周长是卷尺定的：开 2.1560m(里程计读数)实测 2.290m，
// k=1.0622，反推轮径 69.2mm —— 比标称 70 小 1.2%，就是轮胎压扁。
// 轮距用"有效"值：无量纲的 theta_固件/theta_VIO = 1.0226（正反转、两个转速三次一致，
// 散布 0.35%）定的是 C/B 这个比值，再由卷尺定死的 C 推出 B=0.3234，比物理轮距 320mm
// 大 1.1% —— 接地是面不是线、原地转有打滑，这个方向才是物理上对的。
// ⚠️ 两个常量耦合：B_有效 与 C 成正比，重标周长后轮距必须按同比例跟着改。
//
// 2026-09-20 bag 复核距离尺度(近直线窗口，轮速/VIO)：旧车 08-26 是 +8.00%±0.38，而改装后的
// 09-07 只有 +1.70%±0.38 —— 而 VIO 本就少报平移，所以现在 C 在 2% 以内，够用。
// ⇒ 上面"新轮 Ø65~66、距离差 5%"那句是没量过的猜测，已作废。
//
// 🔴 别再用 VIO 定绝对距离。先前按 VIO 把周长改到 0.2046（轮径 65.1mm）是错的，偏了 5.9%，
// 而且它推出的轮距比物理轮距还小 4.8% —— 物理上讲不通。VIO 少报平移，量级几个百分点，
// 而且静止后会在 TRACKING_STATIC 里滞留数秒吞掉开头的运动。旋转比值无量纲，不受影响。
constexpr float WHEEL_CIRC = 0.2173f;      // 等效轮径 69.2mm（标称 70，轮胎压扁 1.2%）
// 2026-09-20 VIO 标定四组(原地左/右@0.8、原地@0.3、弧线 r=0.25@0.8)：theta_固件/theta_VIO
// = 1.024 ± 0.004，与方向/转速/转弯半径都无关 ⇒ 就是这个常数偏小，0.2035 -> 0.2083。
// 🔴 别拿卷尺量的物理轮距来"修"它：同日实测物理轮距约 0.210，但标定定的是无量纲比值
// C/B=1.0432，单独动 B 会立刻把偏航改坏 0.8%。那 1.7mm 的差小于一个胎宽(轮距的定义是
// 两条接地中心线，量外沿/内沿各差一个胎宽)，而且有效轮距本就该【大于】物理值(接地是面、
// 原地转打滑)，方向也对不上 —— 更像读数口径问题。要改就 B 和 C 同比例一起改。
constexpr float WHEEL_BASE = 0.2083f;

constexpr int CH_A = 0, CH_B = 1;
constexpr int PWM_FREQ = 10000;  // TB6612 手册推荐值
constexpr int PWM_BITS = 8;
constexpr int PWM_MAX = 255;

constexpr int DEFAULT_SPEED = 180;
constexpr uint32_t CMD_TIMEOUT_MS = 2000;  // 失联保护：超时自动停车
constexpr uint32_t CTRL_PERIOD_MS = 50;    // 闭环控制周期 20Hz

// PCNT 硬件计数器只有 16 位。不用溢出中断——实测 pcnt_isr_handler_add 的回调里
// pcnt_get_event_status() 拿不到事件，计数一到限位归零就凭空丢一整个 PCNT_LIM。
// 改成轮询累加：读到攒够 PCNT_RESET_AT 就清零并加进 32 位软件计数。
// 限位拉满 32767，离 12000 还有 20767 的余量 = 满速 2.5 秒，正常轮询间隔远小于它
constexpr int16_t PCNT_LIM = 32767;
constexpr int16_t PCNT_RESET_AT = 12000;
// 毛刺滤波 400 个 APB 周期 = 5us。满速 1.2m/s 时边沿间隔约 116us，留了 20 倍余量
constexpr uint16_t PCNT_FILTER = 400;

// 堵转判据：给了这么大 PWM 却几乎不动，持续这么久就断驱动
constexpr int STALL_PWM = 60;
constexpr float STALL_V = 0.02f, STALL_SEC = 0.6f;
// 静摩擦击穿。原地转要拖着两条胎横向擦地，2026-09-01 地砖实测击穿门限 24~28 PWM，而
// w=0.15 时前馈只给到 7 —— 差额靠积分爬要 1~2 s，规划周期只有 0.36 s，指令早换了。
// 判据必须比 STALL_V 松一档：STALL_V 是"该动却不动"的堵转判据，这里是"还没起步"。
constexpr float BRK_V = 0.008f, BRK_HOLD = 0.10f;

// ---- 控制台 ----
// 原生 USB 口改当 NCM 网卡后 CDC 就关了(两者的 IN 端点加起来超过 S3 的 TX FIFO，
// 症状是主机报 can't set config #1, error -32)。此时 Serial 会退化成 UART0，
// 和 Serial0 是同一个口 —— 再相加就会把 available 读成两倍。
#if ARDUINO_USB_CDC_ON_BOOT
#define CONSOLE_HAS_USB 1
#else
#define CONSOLE_HAS_USB 0
// CDC 关掉后 Serial 本身就是 UART0，Arduino 不再定义 Serial0 这个名字
static HardwareSerial &Serial0 = Serial;
#endif
class Console : public Print {
 public:
  bool toUart = false;  // 处理 Looper 发来的命令时置位，让回复原路发回去
  String *cap = nullptr;  // 非空时把输出抄一份，远程命令靠它取回复
  size_t write(uint8_t c) override {
    if (cap) *cap += (char)c;
#if CONSOLE_HAS_USB
    if (Serial) Serial.write(c);  // 没插时 operator bool 为假，跳过，否则会卡住
#endif
    Serial0.write(c);
    if (toUart) Serial1.write(c);
    return 1;
  }
  size_t write(const uint8_t *b, size_t n) override {
    if (cap) cap->concat((const char *)b, n);
#if CONSOLE_HAS_USB
    if (Serial) Serial.write(b, n);
#endif
    Serial0.write(b, n);
    if (toUart) Serial1.write(b, n);
    return n;
  }
#if CONSOLE_HAS_USB
  int available() { return Serial.available() + Serial0.available(); }
  int read() { return Serial.available() ? Serial.read() : Serial0.read(); }
#else
  int available() { return Serial0.available(); }
  int read() { return Serial0.read(); }
#endif
};
static Console io;

struct Wheel {
  const char *name;
  int pinPwm, pinIn1, pinIn2, ch, pinA, pinB;
  pcnt_unit_t unit;
  bool invert;      // 两侧电机镜像安装，都要翻向才是同向前进
  int8_t encSign;   // 编码器 AB 相顺序，镜像安装必然有一侧是反的，用 j 自动标定
  volatile int32_t accum;  // PCNT 限位中断累加值，真实计数 = accum + 硬件 16 位计数
  int32_t lastCount;
  float ppr, kff, vMeas, integ, target, tRamp;
  int deadband, pwm;
  float dS;      // 本控制周期走过的距离 m，里程计用
  float stallT;  // 堵转累计时间 s
  float brkT;    // 目标非零但还没动起来的累计时间 s
};

// ppr 手转 20 圈实测(C3 版)，由编码器线数 x 减速比决定，换主控不变
// 1468.8 / (13 x 4) = 28.2 → 13 线编码器 + 减速比 28，型号 MG513XP28 里的 P28 就是减速比
// kff/截距是空载(轮子架空)拟合的，落地后必须重标
//
// 2026-08-21：驱动组从车后搬到车前，等于绕竖轴转了 180°，于是前后和左右同时翻转 ——
// 两个翻转在偏航上正好抵消，所以症状是"前进后退反了、左右转正常"，而里程计的 x 反、
// theta 正。用 `m 100 0` / `m 0 100` 单独驱动一路验过：B 路(原 g_wl 槽)带的是物理右轮，
// A 路带物理左轮，两路的正向 PWM 都把车往后推。
// 修法是把硬件字段和标定值整组对调、两个 invert 都改成 false，这样 g_wl 才真是物理左轮、
// 正向 PWM 真是物理前进，里程计跟着一起对。
// ⚠️ ppr/kff/截距/编码器符号存在 NVS 里、键是按槽位("l"/"r")而不是物理电机，所以烧完
// 必须重标：先 `j` 定编码器符号，再 calib.py dist / spin 重算周长和轮距。
//
// 2026-09-01：换圆盘底盘重装轮子，同一个 bug 又出现一次 —— 症状完全一致(前后反、左右转正常)，
// 因为"绕竖轴转 180°"同时翻转前后和左右，两个翻转在偏航上抵消。按上面记的修法再对调一次：
// 硬件字段和标定值整组换回来，两个 invert 从 false 改回 true。CFG_REV 2 -> 3。
static Wheel g_wl{"左",  PIN_PWMB, PIN_BIN1, PIN_BIN2, CH_B, PIN_E2A, PIN_E2B,
                  PCNT_UNIT_1, true, +1, 0, 0, 1468.6f, 199.9f, 0, 0, 0, 0, 4, 0};
static Wheel g_wr{"右",  PIN_PWMA, PIN_AIN1, PIN_AIN2, CH_A, PIN_E1A, PIN_E1B,
                  PCNT_UNIT_0, true, +1, 0, 0, 1469.0f, 197.5f, 0, 0, 0, 0, 4, 0};

// PID 和斜坡两轮共用：被控对象特性相近，增益量级一样
static float g_kp = 200, g_ki = 400;  // 自动调参，稳定边界在 kp=800~1600
static int g_brk = 45;                // 静摩擦击穿 PWM，0=关闭；`F` 可在线改，见 BRK_V
// 45 是地毯上扫出来的（40 → 增益 0.85，45 → 0.96，50 → 1.08，59 → 超发到 2.09）。
// 起转时刻从"永不"变成 0.1~0.2 s；顺带把直行 0.02 m/s 的增益从 0.56 提到 0.71~0.88。
static float g_accel = 1.5f;          // 目标斜坡加速度上限 m/s^2
// 减速单独一个限幅。安全只关心刹车：0.40→0 在 1.5 下要 267 ms，是全链 673 ms 里最大
// 的一块。加速方向保持 1.5，所以打滑风险只在刹车时暴露。`A` 可在线改。
static float g_decel = 3.0f;          // 目标斜坡减速度上限 m/s^2

// 里程计位姿，ROS 惯例：x 向前 y 向左 theta 逆时针为正
static float g_ox = 0, g_oy = 0, g_oth = 0;

static int g_speed = DEFAULT_SPEED;
static uint32_t g_lastCmd = 0;
// 闭环开关。目标 0 不等于关环：要减速到零再主动顶住，松手滑行是 s 的事
static bool g_closed = false;
static bool g_guard = true, g_telem = false;
constexpr uint32_t UART_SILENT_MS = 5000;  // app 跑着时 diffcar_control 以 ~19Hz 发 p，5s 很宽松
constexpr uint32_t PC_STALE_MS = 8000;     // K 报告的保质期，超了就当没报过
static uint32_t g_lastUart = 0;            // 最近一次从 UART1 收到字节的时刻
static uint32_t g_pcStamp = 0;             // 最近一次收到 K 的时刻
static bool g_pcOk = true;
// 闩住并计数，理由和启动计数一样：掉线是过后才被发现的，当时没人盯着灯
static uint32_t g_silentCnt = 0, g_silentAt = 0, g_pcBadCnt = 0, g_pcBadAt = 0;
// 遥测在 loop() 里发，而 io.toUart 只在 handleLine 期间为真 —— 所以上位机一行都收不到。
// 记住是谁开的，发的时候临时置位，让串口那边也能收到流。
static bool g_telemUart = false;
// 终端手敲的运动指令自动续期失联保护(人在盯着)；串口来的必须持续重发——
// 上位机一挂，车就该停，不能一直跑下去
static bool g_holdCmd = true;
// D153C 的 ADC 脚是 VIN 的 11:1 分压(10k+1k)。这里是端到端定标，不是纯分压比：
// 万用表 电池11.40V / ADC脚1.03V = 11.07 是真实分压比，但芯片自己读成 1.0147V(低 1.5%)，
// 所以取 11.40/1.0147，把 ADC 的校准误差一起吸收进来
static float g_vbatK = 11.24f;
// 标定 kff 时的电池电压，扫 o 时自动记下。0 = 不做掉压补偿
static float g_vNom = 0;
static float g_vFilt = 0;  // 慢低通，滤掉换向纹波但跟得上放电和负载变化
// 未滤波的最低值，抓电机启动那一瞬的跌落。欠压复位查到哪一环全靠它：
// 电池本身就塌了 vs 电池挺住了但板子 5V/3.3V 塌了，修法完全不同
static float g_vMin = 99;
// 低压保护阈值。默认按 3S 锂电 3.2V/节 取，比 3.0V/节 的截止值留了余量。0 = 关
constexpr uint32_t UART_BAUD = 115200;  // 与 Looper /dev/ttyS3 对齐，两边必须一致

#include "wifitest.h"
#include "usbncm.h"
#include "ncm_policy.h"
#include "ncm_tx_policy.h"

// 把提交失败那一刻的硬件状态解成人话。busy=1 而软件层 transferring=false，
// 就是"软件以为空闲、硬件还卡着"——那种情况在 NCM 层重试多少次都没用。
#define NCM_DEAD_MS 60000   // 收发双双停这么久 = 网桥死了，只剩重启一条路
volatile uint32_t ncm_usb_reconn = 0;   // USB 设备掉线后重新连接的次数
static const char *epStateStr(uint32_t v) {
  static char b[96];
  if (v == 0xFFFFFFFF) return "(从未失败过)";
  snprintf(b, sizeof(b), "busy=%d stall=%d mounted=%d susp=%d alt1=%d",
           (int)!!(v & 1), (int)!!(v & 2), (int)!!(v & 4), (int)!!(v & 8), (int)!!(v & 16));
  return b;
}
#include "USB.h"
static void saveNcmFault(const char *why);   // 定义在 ncmAutoTick 附近，命令处理里要先用
static void ncmEpRegs(char *b, size_t n);
#include "soc/usb_struct.h"
#include "soc/usb_reg.h"

// Looper 的 C 口不供 VBUS：X5 没有 CC 检测硬件(无 typec 子系统)，做 host 也不会主动开
// 5V，而 USB 外设默认要检到 VBUS 才起会话。三个 valid 全 override 成 1，这样靠 5V/3V3
// 引脚供电也能被枚举，C-to-C 直连不再需要那个外部供电转接头。
static void usbForceSessionValid() {
  USB0.gotgctl |= USB_VBVALIDOVEN | USB_VBVALIDOVVAL |
                  USB_AVALIDOVEN  | USB_AVALIDOVVAL  |
                  USB_BVALIDOVEN  | USB_BVALIDOVVAL;
}

static bool g_ncmAuto = true;   // 开机自动拉起 NCM 网桥，掉电保持
static void ncmSetAuto(bool on);
static void ctrlTask(void *arg);
static float g_vLow = 9.5f;
static bool g_lowBat = false;  // 触发后闭锁，等电压回升 0.5V 才解除
static Preferences g_prefs;

// 改了轮子接线/invert 就把 CFG_REV 加一：NVS 里的编码器符号是按槽位存的，接线一动就张冠李戴，
// 而 PID 拿反了符号会变成正反馈，PWM 直接顶到 255 往错方向狂转(堵转保护判的是转不动，抓不到)。
// 版本不匹配时丢掉存的每轮参数并锁住闭环，必须先跑 j 重标编码器方向。
constexpr int32_t CFG_REV = 3;   // 1 = 后驱 invert=true, 2 = 2026-08-21 前驱 invert=false,
                                 // 3 = 2026-09-01 圆盘底盘重装，驱动组又转了 180°
static bool g_encOk = false;     // 编码器符号是否对当前接线有效
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- 复位诊断 ----
// RTC_NOINIT 这块内存 bootloader 从不初始化，靠 magic 认出首启。
// 注意计数只在软件重启/看门狗/掉压复位时保留：按 RST 或烧录会拉低 EN，RTC 域跟着掉电，计数归 1。
// 所以判断"有没有偷偷重启"要看 Looper 有没有收到 BOOT 那一行，别只盯计数
RTC_NOINIT_ATTR static uint32_t g_bootMagic, g_bootCnt;
// 最低电压也放进这块内存：欠压复位会保留 RTC 域，于是重启后能报出"塌掉之前掉到了多少"，
// 这是唯一能抓到复位那一瞬电压的办法(普通变量在复位时就没了)
RTC_NOINIT_ATTR static float g_vMinRtc;
// 电压塌陷环形记录。Looper 掉电重启时它收不到任何日志，而 ESP32 不跟着重启(2026-09-16
// 实测它比 Looper 多跑 325 秒) —— 它是唯一能观测 Looper 复位那一瞬的东西。存 RTC 域，
// Looper 起来后读 N 就能看到重启前后有没有塌陷、塌到多少、电机当时在不在动。
#define SAG_N  8
// 🔴 原来 11.0f 是错的：这是 12V 电池，10~12V 都是健康区间，静置就在 11.x，结果把正常
// 纹波全记成"塌陷"——2026-09-22 一晚攒了 3088 条，深度全是 10.98~10.99（只比门限低 0.02V），
// 我还据此误判成"电池没电"。真正的塌陷是 9.48/9.72/9.85/9.98 那一档。
// 取 10.0V：这是 12V 电池，静置就在 11.x，常态纹波不会碰到它，又能在真掉电前记上。
// ⚠️ 「Looper 低于 9.4V 起不来」是一次掉电事件里的相关观测，**没有做过扫描验证**，
// 不要当成门限来源引用。10.0 是留裕量，不是从 9.4 推出来的。
#define SAG_TH 10.0f
RTC_NOINIT_ATTR static struct { uint32_t ms; float v; int16_t pl, pr; } g_sag[SAG_N];
RTC_NOINIT_ATTR static uint32_t g_sagCnt;
// 主循环卡死定位：每进一个阶段写个编号，卡死重启后直接报出卡在哪一段。
// 2026-09-18 实测 TCP 下行压测会让固件静默挂死——无 panic、无重启、串口不应答，只能人工断电。
RTC_NOINIT_ATTR static volatile uint32_t g_stage;   // 主循环写、看门狗任务读 ⇒ 必须 volatile
RTC_NOINIT_ATTR static uint32_t g_stuckAt, g_stuckMagic;
RTC_NOINIT_ATTR static uint32_t g_stuckBeat;   // 卡死那一刻的心跳值：极小=循环压根没跑起来
RTC_NOINIT_ATTR static uint32_t g_stuckWant;   // 卡死那一刻谁正在等锁(owner=0 时唯一能分辨的判据)
RTC_NOINIT_ATTR static uint32_t g_stuckSub;    // 卡在 stage 内部的第几句
RTC_NOINIT_ATTR static char g_stuckHolder[20]; // 卡死那一刻 NCM 锁在谁手里(问内核，不是我们记的)
// 🔑 分辨"卡在代码里"和"没被调度"。此前所有字段对这两者都是同一副样子：锁空着、没人等、
// g_sub 停在某处 —— 而那正是被更高优先级任务踢下去之后的样子。任务状态是唯一的判据。
RTC_NOINIT_ATTR static uint32_t g_stuckLoopState;
RTC_NOINIT_ATTR static char g_stuckCore1[20];   // 卡死那一刻 core1(主循环所在核)上真正在跑的是谁
RTC_NOINIT_ATTR static uint32_t g_subHist[8];   // 看门狗每秒采一次 g_sub：是"一直停在这"还是"刚停"
RTC_NOINIT_ATTR static uint32_t g_subHistIdx;
// 唯一位置编号。卡点只到 stage 这一级时分不出是哪一句 —— 每个可能阻塞的位置
// 给一个唯一号，卡死记录直接指到行。编号规则：stage*10+序号，跨文件的用 stage*100+序号。
volatile uint32_t g_sub;
RTC_NOINIT_ATTR static volatile uint32_t g_stageAux;   // 同上
RTC_NOINIT_ATTR static uint32_t g_stuckAux, g_stuckHeap;
RTC_NOINIT_ATTR static uint32_t g_stuckOwner, g_stuckLockSeq, g_stuckRecStep;
extern volatile uint8_t ncm_lock_owner;
extern volatile uint32_t ncm_lock_want;
extern "C" const char *ncm_mutex_holder_name(void);
extern volatile uint8_t ncm_recover_step;   // 卡在 recover 函数体的哪一句    // 1=recover 2=xfer_cb 3=can_xmit 4=xmit 5=alt切换
extern volatile uint32_t ncm_lock_seq;
#define STAGE(n) (g_stage = (n))
#define STUCK_MAGIC 0xDEAD5A5B
#define LOOP_STUCK_S 25     // 主循环停跳这么久=卡死。常态每秒 2~3 万圈，而标定/自检可能阻塞十几秒
static volatile uint32_t g_beat;   // 主循环心跳，看门狗任务靠它判活
// 对照实验用：这个任务每秒唤醒一次，怀疑正是它改变了 core 0 的调度从而治好了挂死。
// 调大到 30 秒 ≈ 等价于没有这个任务，用来验证因果；看门狗仍在，只是变粗（Nz 切换）。
static volatile uint32_t g_wdtPeriodMs = 1000;
static esp_reset_reason_t g_rst;
static float g_vMinLast;  // 上次复位前的最低值，供 BOOT 行和 ? 报出来
static char g_stuckNote[480];   // 🔴 160 曾不够，现场行被截断丢了整个字段。中文一字 3 字节，往宽了给   // 上次若是卡死重启，留一句给 N；magic 在 setup 就被清掉了

uint32_t wt_boot_cnt(void) { return g_bootCnt; }
float wt_vmin_last(void) { return g_vMinLast > 90 ? 0.0f : g_vMinLast; }
const char *wt_stuck_note(void) { return g_stuckNote; }
static const char *resetName(esp_reset_reason_t r);
const char *wt_reset_name(void) { return resetName(g_rst); }

static const char *resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "上电";
    case ESP_RST_EXT:       return "RST 按键";
    case ESP_RST_SW:        return "软件重启";
    case ESP_RST_PANIC:     return "崩溃(异常)";
    case ESP_RST_INT_WDT:   return "中断看门狗";
    case ESP_RST_TASK_WDT:  return "任务看门狗";
    case ESP_RST_WDT:       return "看门狗";
    case ESP_RST_BROWNOUT:  return "掉压复位 << 电机把电压拽下去了";
    case ESP_RST_DEEPSLEEP: return "深睡唤醒";
    default:                return "未知";
  }
}

// ---- 状态灯 ----
// 上位机链路的两个状态是分开的，因为发现者不同：ESP32 自己能看出 UART1 静默，
// 但「看车的网页断开了」只有 Looper 的网页后端知道，必须靠 K 命令报进来。
// ⚠️ 别拿 LED_LINKLOST 当这个用 —— 它的含义是"2s 无运动指令"，没人开车时本来就该亮。
enum LedState { LED_OK, LED_CAL, LED_LINKLOST, LED_STALL, LED_LOWBAT,
                LED_UPSTREAM_SILENT, LED_PC_UNREACHABLE };
static LedState g_ledState = LED_OK;

// 灯的颜色是判断上位机状态最快的依据，但人得在车旁边看。把它做进状态报告，远程也能判。
static const char *ledName(LedState s) {
  static const char *n[] = {"绿常亮(正常)", "黄(标定自检)", "红慢闪(失联停车)",
                            "红快闪(堵转)", "紫闪(低压)", "橙慢闪(ttyS3静默)",
                            "青慢闪(网页断开)"};
  return (s >= LED_OK && s <= LED_PC_UNREACHABLE) ? n[s] : "?";
}
static uint8_t g_ledLvl = 12;  // 0-255，默认压得很暗，调试时不刺眼
static rmt_obj_t *g_rmt[2];

static void ledRaw(uint8_t r, uint8_t g, uint8_t b) {
  static uint32_t last = 0xFFFFFFFF;
  static uint32_t lastMs = 0;
  uint32_t now = (uint32_t)r << 16 | (uint32_t)g << 8 | b;
  uint32_t t = millis();
  // 颜色没变就不发，省得每周期占着 RMT；但每 500ms 强制重发一次——
  // WS2812 是锁存的，一旦被电机噪声干扰锁错颜色(常见是全白)，不重发就会一直错下去
  if (now == last && t - lastMs < 500) return;
  last = now;
  lastMs = t;
  rmt_data_t d[24];
  uint8_t c[3] = {g, r, b};  // WS2812 线序是 GRB
  for (int i = 0; i < 24; i++) {
    bool one = c[i / 8] & (0x80 >> (i % 8));
    d[i].level0 = 1;
    d[i].duration0 = one ? 8 : 4;  // tick = 100ns
    d[i].level1 = 0;
    d[i].duration1 = one ? 4 : 8;
  }
  for (auto *h : g_rmt)
    if (h) rmtWriteBlocking(h, d, 24);
}

static void ledUpdate() {
  uint8_t L = g_ledLvl;
  uint32_t t = millis();
  switch (g_ledState) {
    case LED_OK:       ledRaw(0, L, 0); break;  // 绿常亮: 一切正常
    case LED_CAL:      ledRaw(L, L, 0); break;  // 黄: 标定/自检中
    case LED_LINKLOST: ledRaw((t / 500) % 2 ? L : 0, 0, 0); break;  // 红慢闪: 失联停车
    case LED_STALL:    ledRaw((t / 100) % 2 ? L : 0, 0, 0); break;  // 红快闪: 堵转
    case LED_LOWBAT:   ledRaw((t / 400) % 2 ? L : 0, 0, (t / 400) % 2 ? L : 0); break;  // 紫闪: 低压
    // 橙慢闪: UART1 静默，上位机(Looper/app)没在跟我说话
    case LED_UPSTREAM_SILENT:
      ledRaw((t / 700) % 2 ? L : 0, (t / 700) % 2 ? max<uint8_t>(1, L / 3) : 0, 0); break;
    // 青慢闪: Looper 活着，但正在看车的网页异常断开了（断网/锁屏）
    case LED_PC_UNREACHABLE:
      ledRaw(0, (t / 700) % 2 ? L : 0, (t / 700) % 2 ? L : 0); break;
  }
}

static void ledFlash(uint8_t r, uint8_t g, uint8_t b, int ms) {
  ledRaw(r, g, b);
  delay(ms);
  ledRaw(0, 0, 0);
}

// ---- 编码器 ----
static void encInit(Wheel &w) {
  pcnt_config_t c = {};
  c.unit = w.unit;
  c.counter_h_lim = PCNT_LIM;
  c.counter_l_lim = -PCNT_LIM;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;
  // 通道0 数 A 相两个边沿、B 相定方向；通道1 反过来。两通道合起来就是四倍频
  c.pulse_gpio_num = w.pinA;
  c.ctrl_gpio_num = w.pinB;
  c.channel = PCNT_CHANNEL_0;
  c.lctrl_mode = PCNT_MODE_KEEP;
  c.hctrl_mode = PCNT_MODE_REVERSE;
  pcnt_unit_config(&c);
  c.pulse_gpio_num = w.pinB;
  c.ctrl_gpio_num = w.pinA;
  c.channel = PCNT_CHANNEL_1;
  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_unit_config(&c);

  gpio_pullup_en((gpio_num_t)w.pinA);  // 兼容推挽和开漏两种编码器
  gpio_pullup_en((gpio_num_t)w.pinB);
  pcnt_set_filter_value(w.unit, PCNT_FILTER);
  pcnt_filter_enable(w.unit);
  pcnt_counter_pause(w.unit);
  pcnt_counter_clear(w.unit);
  pcnt_counter_resume(w.unit);
}

static int32_t encRead(Wheel &w) {
  int16_t raw;
  portENTER_CRITICAL(&g_mux);
  pcnt_get_counter_value(w.unit, &raw);
  // 读和清之间到达的边沿会被抹掉，但这个窗口只有几百纳秒、且满速下约 1.4 秒才清一次，
  // 折算下来百万分之一量级，远小于 ppr 标定本身的误差
  if (raw > PCNT_RESET_AT || raw < -PCNT_RESET_AT) {
    pcnt_counter_clear(w.unit);
    w.accum += raw;
    raw = 0;
  }
  int32_t t = w.accum + raw;
  portEXIT_CRITICAL(&g_mux);
  return w.encSign * t;
}

static void encZero() {
  for (Wheel *w : {&g_wl, &g_wr}) {
    portENTER_CRITICAL(&g_mux);
    pcnt_counter_clear(w->unit);
    w->accum = 0;
    portEXIT_CRITICAL(&g_mux);
    w->lastCount = 0;
  }
}

// ---- 驱动 ----
// 低压闭锁时一律不给使能，堵住所有让电机动起来的路径
static void setEnable(bool on) { digitalWrite(PIN_STBY, on && !g_lowBat); }

// pwm 取值 -255~255，正为前进
static void setWheel(Wheel &w, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  int p = w.invert ? -pwm : pwm;
  digitalWrite(w.pinIn1, p >= 0);
  digitalWrite(w.pinIn2, p < 0);
  ledcWrite(w.ch, abs(p));
  w.pwm = pwm;
}

static void stopAll() {
  g_closed = false;
  for (Wheel *w : {&g_wl, &g_wr}) {
    w->target = w->tRamp = w->integ = w->stallT = 0;
    w->pwm = 0;
    ledcWrite(w->ch, 0);
    digitalWrite(w->pinIn1, LOW);
    digitalWrite(w->pinIn2, LOW);
  }
  setEnable(false);  // STBY 回到 GPIO 了，急停是真的切断驱动，不只是 PWM 归零
  if (g_ledState != LED_LINKLOST && g_ledState != LED_STALL) g_ledState = LED_OK;
}

static void drive(int left, int right) {
  g_closed = false;  // 手动给 PWM 就退出闭环
  g_wl.target = g_wr.target = 0;
  setEnable(true);
  setWheel(g_wl, left);
  setWheel(g_wr, right);
  g_lastCmd = millis();
  g_ledState = LED_OK;
}

// 闭环目标轮速 m/s。只在"从停机进闭环"这一刻复位状态：
// cmd_vel 会以几十 Hz 连续刷新，每帧都清积分等于永远没有积分
static void setTargets(float vl, float vr) {
  if (!g_encOk) {
    io.println(F("闭环已锁: 接线变了(CFG_REV)，编码器符号未重标。轮子架空跑 j，再敲 w"));
    return;
  }
  if (!g_closed) {
    g_wl.tRamp = g_wl.vMeas;  // 斜坡从当前实速起步，避免跳变
    g_wr.tRamp = g_wr.vMeas;
    g_wl.integ = g_wr.integ = 0;
    g_wl.stallT = g_wr.stallT = 0;
    g_closed = true;
    setEnable(true);
  }
  g_wl.target = vl;
  g_wr.target = vr;
  g_lastCmd = millis();
  g_ledState = LED_OK;
}

// 差速逆运动学：v 前进 m/s，w 逆时针 rad/s
static void setTwist(float v, float w) {
  setTargets(v - w * WHEEL_BASE * 0.5f, v + w * WHEEL_BASE * 0.5f);
}

// fromUart 时保持安静：上位机以几十 Hz 刷 cmd_vel，每条都回一大句会把串口塞满
static void reportTarget(bool quiet) {
  if (quiet) return;
  g_telem = true;  // 人在终端上敲的，顺手把遥测打开
  if (!g_wl.target && !g_wr.target)
    io.println(F("闭环减速停住(仍在顶住零速，s 才彻底松开)"));
  else
    io.printf("闭环目标: 左=%.3f 右=%.3f m/s\n", g_wl.target, g_wr.target);
}

// 中点积分：用这一步的半程角度算位移方向，转弯时比欧拉法误差小一个量级
static void odomUpdate() {
  float ds = 0.5f * (g_wl.dS + g_wr.dS);
  float dth = (g_wr.dS - g_wl.dS) / WHEEL_BASE;
  float mid = g_oth + 0.5f * dth;
  g_ox += ds * cosf(mid);
  g_oy += ds * sinf(mid);
  g_oth += dth;
  if (g_oth > PI) g_oth -= 2 * PI;
  else if (g_oth < -PI) g_oth += 2 * PI;
}

static float vbat() { return analogReadMilliVolts(PIN_VBAT) / 1000.0f * g_vbatK; }

// PWM 占空比控制的是"电源电压的百分之多少加到电机上"，所以同样的 PWM 在低电压下转得慢。
// 按标定时的电压反比缩放前馈，PI 就只用管小残差，不必硬扛整个掉压误差
static float vComp() {
  if (g_vNom < 1 || g_vFilt < 1) return 1.0f;
  return constrain(g_vNom / g_vFilt, 0.7f, 1.5f);  // 夹住，防止 ADC 断线时把 PWM 顶飞
}

// ---- 参数存取 ----
static void loadParams() {
  g_prefs.begin("car", true);
  g_ncmAuto = g_prefs.getBool("ncmauto", true);   // 默认自启，不然拔了网卡就没路进来
  g_kp = g_prefs.getFloat("kp", g_kp);
  g_ki = g_prefs.getFloat("ki", g_ki);
  g_accel = g_prefs.getFloat("acc", g_accel);
  g_decel = g_prefs.getFloat("dec", g_decel);
  g_brk = g_prefs.getInt("brk", g_brk);
  g_ledLvl = g_prefs.getUChar("led", g_ledLvl);
  g_vbatK = g_prefs.getFloat("vk", g_vbatK);
  g_vNom = g_prefs.getFloat("vnom", g_vNom);
  g_vLow = g_prefs.getFloat("vlow95", g_vLow);   // 换键名：旧键里存着 9.6，改默认值盖不过它
  g_encOk = g_prefs.getInt("cfg", 0) == CFG_REV;
  if (g_encOk) {
    for (Wheel *w : {&g_wl, &g_wr}) {
      String k = (w == &g_wl) ? "l" : "r";
      w->ppr = g_prefs.getFloat((k + "ppr4").c_str(), w->ppr);
      w->kff = g_prefs.getFloat((k + "kff").c_str(), w->kff);
      w->deadband = g_prefs.getInt((k + "db").c_str(), w->deadband);
      w->encSign = g_prefs.getChar((k + "sgn").c_str(), w->encSign);
    }
  }
  g_prefs.end();
}

static void saveParams() {
  g_prefs.begin("car", false);
  g_prefs.putFloat("kp", g_kp);
  g_prefs.putFloat("ki", g_ki);
  g_prefs.putFloat("acc", g_accel);
  g_prefs.putFloat("dec", g_decel);
  g_prefs.putInt("brk", g_brk);
  g_prefs.putUChar("led", g_ledLvl);
  g_prefs.putFloat("vk", g_vbatK);
  g_prefs.putFloat("vnom", g_vNom);
  g_prefs.putFloat("vlow95", g_vLow);
  for (Wheel *w : {&g_wl, &g_wr}) {
    String k = (w == &g_wl) ? "l" : "r";
    g_prefs.putFloat((k + "ppr4").c_str(), w->ppr);
    g_prefs.putFloat((k + "kff").c_str(), w->kff);
    g_prefs.putInt((k + "db").c_str(), w->deadband);
    g_prefs.putChar((k + "sgn").c_str(), w->encSign);
  }
  g_prefs.putInt("cfg", CFG_REV);
  g_prefs.end();
  io.println(F("已保存到 flash，下次上电自动生效"));
  ledFlash(0, g_ledLvl, g_ledLvl, 120);  // 青，不用白(白留给故障判据)
}

static void pinMap() {
  io.println(F(
      "接线表 (ESP32-S3-DevKitC-1，左排从 RST 下面第一个孔往下数):\n"
      "  左排 1  GPIO4  -> E2B   左轮编码器B\n"
      "  左排 2  GPIO5  -> E2A   左轮编码器A\n"
      "  左排 3  GPIO6  -> E1B   右轮编码器B\n"
      "  左排 4  GPIO7  -> E1A   右轮编码器A\n"
      "  左排 5  GPIO15 -> PWMB\n"
      "  左排 6  GPIO16 -> BIN2\n"
      "  左排 7  GPIO17 -> BIN1\n"
      "  左排 8  GPIO18 -> 空闲\n"
      "  左排 9  GPIO8  -> 空闲(ADC1_CH7，留给模拟量)\n"
      "  左排 10 GPIO3  -> 跳过(strapping，留空)\n"
      "  左排 11 GPIO46 -> STBY  (唯一一根交叉的线)\n"
      "  左排 12 GPIO9  -> 空闲(ADC1_CH8，留给模拟量)\n"
      "  左排 13 GPIO10 -> ADC   电池电压\n"
      "  左排 14 GPIO11 -> AIN1  ┐\n"
      "  左排 15 GPIO12 -> AIN2  ├ A 通道三根连着，和驱动板排针同序不交叉\n"
      "  左排 16 GPIO13 -> PWMA  ┘\n"
      "  左排倒数2 5V   -> 驱动板 5V      左排最后 GND -> 驱动板 GND\n"
      "  右排 GPIO40 = RX1 <- X5 的 TX    右排 GPIO41 = TX1 -> X5 的 RX\n"
      "       (已焊死。串口必须交叉，GPIO41 是我们发、GPIO40 是我们收)\n"
      "禁用: 26-32(flash) 33-37(八线PSRAM,丝印骗人) 45(VDD_SPI) 0/3(strapping)"));
}

static void help() {
  io.printf("\n当前参数: kp=%.1f ki=%.1f 加速度=%.2f 轮周长=%.4fm 轮距=%.3fm\n", g_kp, g_ki,
            g_accel, WHEEL_CIRC, WHEEL_BASE);
  for (Wheel *w : {&g_wl, &g_wr})
    io.printf("  %s轮: 脉冲/圈=%.1f kff=%.1f 截距=%d 编码器符号=%+d\n", w->name, w->ppr,
              w->kff, w->deadband, w->encSign);
  io.printf("  灯亮度=%d  电池=%.2fV(倍率 %.2f)  掉压补偿: %s\n", g_ledLvl, vbat(), g_vbatK,
            g_vNom < 1 ? "关(扫一次 o 自动开启)"
                       : (String("基准") + g_vNom + "V 当前系数" + vComp()).c_str());
  io.printf("  低压保护=%.2fV %s\n", g_vLow, g_lowBat ? "<< 已触发，闭锁中" : "");
  const char *lname[] = {"绿(正常)","黄(标定)","红慢闪(失联)","红快闪(堵转)","紫闪(低压)",
                         "橙慢闪(上位机静默)","青慢闪(网页断开)"};
  io.printf("  灯=%s  UART1 静默 %lus  网页=%s(%lus前报)\n", lname[g_ledState],
            (unsigned long)((millis() - g_lastUart) / 1000), g_pcOk ? "连着" : "断开",
            (unsigned long)((millis() - g_pcStamp) / 1000));
  io.printf("  链路事件: 上位机静默 %lu 次(最近 %lus前)  网页断开 %lu 次(最近 %lus前)\n",
            (unsigned long)g_silentCnt,
            (unsigned long)(g_silentCnt ? (millis() - g_silentAt) / 1000 : 0),
            (unsigned long)g_pcBadCnt,
            (unsigned long)(g_pcBadCnt ? (millis() - g_pcBadAt) / 1000 : 0));
  if (!g_encOk) io.println(F("  >> 闭环已锁: 接线版本变了，先架空跑 j 重标编码器方向，再敲 w"));
  io.printf("  启动次数=%lu 上次复位=%s  运行=%lu秒  上次塌前最低=%.2fV\n",
            (unsigned long)g_bootCnt, resetName(g_rst), (unsigned long)(millis() / 1000),
            g_vMinLast > 90 ? 0.0f : g_vMinLast);
  io.println(F(
      "命令(回车结束):\n"
      "-- 开环 --\n"
      "  f / b / l / r       前进 / 后退 / 左转 / 右转\n"
      "  s                   急停: 退出闭环 + STBY 断驱动\n"
      "  v <0-255>           设置默认速度\n"
      "  m <左> <右>         直接给两轮 PWM，范围 -255~255\n"
      "  d                   自动 demo 序列(前后左右各 1.5s)\n"
      "  g                   开关失联保护(默认开: 2s 无命令自动停)\n"
      "-- 标定 --\n"
      "  e                   打印两轮编码器状态\n"
      "  z                   计数清零\n"
      "  j                   编码器方向自动标定(架空跑一下，自动定符号)\n"
      "  n <左> [右]         每圈脉冲数(正交四倍频)\n"
      "  o [起始 结束 步长]  同时扫两轮 PWM->速度曲线(缺省 0 240 20)\n"
      "  x <左kff> <右kff>   前馈斜率 PWM/(m/s)\n"
      "  i <左截距> <右截距> 前馈零速截距\n"
      "-- 闭环 --\n"
      "  k <kp> <ki>         PID 参数，两轮共用\n"
      "  a <m/s^2>           目标斜坡加速度上限\n"
      "  A <m/s^2>           目标斜坡减速度上限（刹车，默认 3.0）\n"
      "  F [pwm]             静摩擦击穿 PWM，0=关闭；不带参数只查询\n"
      "  c <v> [vR]          闭环目标轮速，可为负，0 = 减速停住并顶住\n"
      "  u <v> [w]           底盘速度: 前进 m/s + 转向 rad/s(逆时针正)\n"
      "-- 里程计 --\n"
      "  p                   打印位姿 x y theta\n"
      "  q                   位姿清零\n"
      "  y                   开关遥测\n"
      "  w                   把全部参数存进 flash\n"
      "-- 排障 --\n"
      "  P                   打印完整接线表\n"
      "  t                   引脚自检: 逐个拉高输入脚，用万用表量\n"
      "  L <0-255>           状态灯亮度\n"
      "  B <倍率> [阈值V]    电池分压定标 + 低压保护阈值(0=关)\n"
      "  V <电压>            掉压补偿基准电压，0=关。扫 o 时自动记录\n"
      "  X                   监听 UART1(接 Looper) 三秒\n"
      "  X <文本>            发给 Looper，再监听两秒\n"
      "  K <0|1>             上位机报告看车的网页是否连着(网页后端判断，我们只显示)\n"
      "  U <字节数> <md5>    从串口 OTA 升级固件(Looper 上跑 flash.py，别手敲)\n"
      "  ?                   本帮助\n"
      "状态灯: 绿常亮=正常  黄=标定/自检/升级中  蓝一闪=刚启动\n"
      "        红慢闪=失联(2s无运动指令)  红快闪=堵转  紫闪=电池低压\n"
      "        橙慢闪=上位机静默(UART1 5s无字节: Looper挂了/串口断了/app没跑)\n"
      "        青慢闪=正在看车的网页异常断开了(断网/锁屏)，重新打开网页后熄灭\n"
      "        白 = 固件不会输出的颜色，看到就是灯的数据线被干扰了(见下面启动次数)"));
}

static void encStatus() {
  for (Wheel *w : {&g_wl, &g_wr}) {
    int32_t c = encRead(*w);
    // 目标和 PWM 也报出来：闭环稳态的 PWM 就是 kff 该拟合的目标值，而开环的 `o` 扫描
    // 会因为两轮 4.3% 的速度差把车带偏（实测 7.7 度/米），落地标定不能用它
    io.printf("%s轮: 脉冲/圈=%.1f 计数=%ld 圈数=%.3f 里程=%.3fm 实测=%.3fm/s 目标=%.3f PWM=%d\n",
              w->name, w->ppr, (long)c, c / w->ppr, c / w->ppr * WHEEL_CIRC, w->vMeas,
              w->target, w->pwm);
  }
  // 锁= 让网页说得出「车为什么不动」：低压和堵转都只断驱动，上位机原来看不到
  io.printf("电池: 当前=%.2fV 最低=%.2fV 锁=%s 阈值=%.2fV\n", vbat(), g_vMin > 90 ? 0.0f : g_vMin,
            g_lowBat ? "lowbat" : g_ledState == LED_STALL ? "stall" : "none", g_vLow);
  g_vMin = 99;  // 报一次清一次，这样每条日志都是"上个采样区间内的最低"
}

// 两轮同时扫，一次拿到两条曲线，还能直接看出左右差异
constexpr uint32_t SWEEP_SETTLE_MS = 1000;
constexpr uint32_t SWEEP_MEAS_MS = 600;

static void sweep(int from, int to, int step) {
  stopAll();
  g_ledState = LED_CAL;
  ledUpdate();
  setEnable(true);
  g_vNom = g_vFilt > 1 ? g_vFilt : vbat();  // 这条曲线是在这个电压下测的，补偿以它为基准
  io.printf("扫描 %d..%d 步长 %d  (标定电压 %.2fV)\n", from, to, step, g_vNom);
  // 每档正反各测一次：落地时车原地来回摆而不是一路跑掉，同时正反平均能抵消掉
  // 电机正反不对称。单向扫到 240 要跑十几米，往返只占约 2.5 米
  io.println(F("每档正反各测一次，车来回摆；用最后两列(正反平均)拟合"));
  io.println(F("PWM,左正,左反,左均,右正,右反,右均"));
  for (int pwm = from; pwm <= to; pwm += step) {
    float vl[2], vr[2];
    for (int d = 0; d < 2; d++) {
      int p = d ? -pwm : pwm;
      setWheel(g_wl, p);
      setWheel(g_wr, p);
      delay(SWEEP_SETTLE_MS);
      int32_t c0l = encRead(g_wl), c0r = encRead(g_wr);
      delay(SWEEP_MEAS_MS);
      float k = 1000.0f / SWEEP_MEAS_MS * WHEEL_CIRC;
      vl[d] = fabsf((encRead(g_wl) - c0l) / g_wl.ppr * k);
      vr[d] = fabsf((encRead(g_wr) - c0r) / g_wr.ppr * k);
    }
    io.printf("%3d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", pwm, vl[0], vl[1],
              0.5f * (vl[0] + vl[1]), vr[0], vr[1], 0.5f * (vr[0] + vr[1]));
  }
  stopAll();
  io.println(F("扫描结束"));
  g_ledState = LED_OK; ledUpdate();
}

// 镜像安装的两个电机，编码器符号必然一正一负，具体哪边负取决于接线，跑一下测出来
static void calibEncDir() {
  stopAll();
  g_ledState = LED_CAL;
  ledUpdate();
  io.println(F("编码器方向自动标定: 两轮各正转 1 秒(确保轮子架空)"));
  for (Wheel *w : {&g_wl, &g_wr}) {
    w->encSign = 1;
    encZero();
    setEnable(true);
    setWheel(*w, 120);
    delay(1000);
    int32_t c = encRead(*w);
    setWheel(*w, 0);
    setEnable(false);
    if (abs(c) < 200) {
      io.printf("  %s轮: 计数只有 %ld，几乎没转 —— 电机没通电还是编码器没接？\n", w->name,
                (long)c);
      continue;
    }
    w->encSign = c > 0 ? 1 : -1;
    io.printf("  %s轮: 正转计数 %+ld → 符号定为 %+d\n", w->name, (long)c, w->encSign);
    delay(500);
  }
  encZero();
  stopAll();
  g_encOk = true;
  io.println(F("标定结束，闭环已解锁，敲 w 存起来"));
  g_ledState = LED_OK; ledUpdate();
}

// 不接电机也能跑：万用表量 TB6612 输入排针，确认每根杜邦线通
static void pinTest() {
  const struct { const char *name; int pin; } pins[] = {
      {"STBY", PIN_STBY}, {"AIN1", PIN_AIN1}, {"AIN2", PIN_AIN2},
      {"BIN1", PIN_BIN1}, {"BIN2", PIN_BIN2},
  };
  stopAll();
  g_ledState = LED_CAL;
  ledUpdate();
  io.println(F("引脚自检: 每根拉高 3s，量对应引脚应为 3.3V"));
  for (auto &p : pins) {
    io.printf("  %s (GPIO%d) = HIGH ...\n", p.name, p.pin);
    digitalWrite(p.pin, HIGH);
    delay(3000);
    digitalWrite(p.pin, LOW);
  }
  io.println(F("  PWMA/PWMB 输出 50% 占空比 3s，量到约 1.65V"));
  ledcWrite(CH_A, 128);
  ledcWrite(CH_B, 128);
  delay(3000);
  stopAll();
  io.println(F("自检结束"));
  g_ledState = LED_OK; ledUpdate();
}

// UART1 在 setup() 里 begin 一次，之后再也不碰：不调 pinMode、不改波特率、不重新 begin。
// 这三样实测都会把 TX 静默弄哑——发不出去、且没有任何报错，排查代价极高。
// 波特率固定 115200，必须和 Looper 的 /dev/ttyS3 一致。
// 监听 UART1，收到什么原样打出来（不可打印字符显示成 .）
static void uartListen(uint32_t ms) {
  io.printf("监听 UART1 @%lu，%.1f 秒...\n", (unsigned long)UART_BAUD, ms / 1000.0f);
  uint32_t t0 = millis();
  int n = 0;
  String line;
  while (millis() - t0 < ms) {
    while (Serial1.available()) {
      int ch = Serial1.read();
      n++;
      line += (ch >= 0x20 && ch < 0x7f) ? (char)ch : '.';
      if (line.length() >= 72 || ch == '\n') { io.printf("  | %s\n", line.c_str()); line = ""; }
    }
  }
  if (line.length()) io.printf("  | %s\n", line.c_str());
  io.printf("共收到 %d 字节%s\n", n, n ? "" : " —— 对面没在发");
}

// 从 UART1 升级固件。ESP32-S3 的 ROM 引导程序只在 UART0(GPIO43/44) 和原生 USB 上听
// 下载命令，焊死到 Looper 的是 UART1(GPIO40/41)，esptool 永远够不着——所以只能由
// 运行中的固件自己收下新固件写进备用分区(app1)，重启时引导程序切过去。
// 全程 MD5 校验，校验不过就整个作废，老固件原封不动，不存在烧一半变砖
constexpr uint32_t OTA_CHUNK = 2048;  // 和 looper/flash.py 里的 CHUNK 必须一致

// 波特率参数是可选的：日常链路留在 115200 不动(遥控和导航都靠它，不能拿来冒险)，
// 只在传固件那几秒临时提速。传完或失败都无条件重启 —— setup() 会重新 begin 回 115200，
// 所以就算高波特率在这条线上跑不通，也只是这次升级失败，不会失联
static void otaRecv(String rest) {
  int sp1 = rest.indexOf(' ');
  uint32_t size = sp1 > 0 ? (uint32_t)rest.substring(0, sp1).toInt() : 0;
  String tail = sp1 > 0 ? rest.substring(sp1 + 1) : "";
  tail.trim();
  int sp2 = tail.indexOf(' ');
  String md5 = sp2 > 0 ? tail.substring(0, sp2) : tail;
  uint32_t baud = sp2 > 0 ? (uint32_t)tail.substring(sp2 + 1).toInt() : UART_BAUD;
  md5.trim();
  if (size < 65536 || md5.length() != 32) {
    io.println(F("用法: U <字节数> <32位md5> [波特率]"));
    return;
  }
  if (baud < UART_BAUD || baud > 2000000) baud = UART_BAUD;
  stopAll();
  g_ledState = LED_CAL;
  ledUpdate();
  if (!Update.begin(size)) {
    io.printf("OTA 起不来: %s\n", Update.errorString());
    return;
  }
  Update.setMD5(md5.c_str());
  io.printf("OTA-READY %lu\n", (unsigned long)baud);  // 上位机见到这行才开始灌裸二进制

  bool fast = baud != UART_BAUD;
  if (fast) {
    Serial1.flush();  // OTA-READY 必须以旧波特率发完，否则尾巴会以新波特率发出去变乱码
    delay(50);        // 给对面留出切换的时间
    Serial1.updateBaudRate(baud);
  }

  // 必须应答式流控：写 flash 会阻塞 CPU 并关 cache，那几十毫秒里 UART 的中断跑不了，
  // 硬件 FIFO 只有 128 字节(约 11ms)必溢出，软件缓冲再大也没用——因为搬运的中断本身没在跑。
  // 所以收满一块 → 停下来写 → 回一个 ACK，让对面在我们写 flash 期间保持安静
  static uint8_t buf[OTA_CHUNK];
  uint32_t got = 0;
  bool timeout = false;
  while (got < size && !timeout) {
    uint32_t want = min((uint32_t)OTA_CHUNK, size - got), n = 0, last = millis();
    while (n < want) {
      int r = Serial1.readBytes(buf + n, want - n);
      if (r > 0) { n += r; last = millis(); }
      else if (millis() - last > 5000) { timeout = true; break; }
    }
    if (timeout) break;
    Update.write(buf, n);
    got += n;
    Serial1.write(0x06);  // ACK：写完了，可以发下一块
  }

  if (timeout) {
    Update.abort();
    io.printf("OTA-FAIL 超时，只收到 %lu/%lu\n", (unsigned long)got, (unsigned long)size);
  } else if (Update.end(true)) {
    io.println(F("OTA-OK 校验通过，重启"));
  } else {
    io.printf("OTA-FAIL %s\n", Update.errorString());
  }
  // 成功要重启才能切到新分区；失败也重启 —— 这是把波特率拉回 115200 的唯一保证
  Serial1.flush();
  delay(200);
  ESP.restart();
}

static void demoSequence() {
  const struct { const char *name; int l, r; } steps[] = {
      {"前进", g_speed, g_speed},   {"停", 0, 0},
      {"后退", -g_speed, -g_speed}, {"停", 0, 0},
      {"左转", -g_speed, g_speed},  {"停", 0, 0},
      {"右转", g_speed, -g_speed},  {"停", 0, 0},
  };
  for (auto &s : steps) {
    io.printf("  %s\n", s.name);
    drive(s.l, s.r);
    delay(s.l == 0 && s.r == 0 ? 600 : 1500);
  }
  stopAll();
  io.println(F("demo 结束"));
}

// 按空格拆出最多 n 个数，返回实际拿到几个
static int parseNums(const String &s, float *out, int n) {
  int cnt = 0, start = 0;
  while (cnt < n && start < (int)s.length()) {
    int sp = s.indexOf(' ', start);
    if (sp < 0) sp = s.length();
    out[cnt++] = s.substring(start, sp).toFloat();
    start = sp + 1;
  }
  return cnt;
}

static uint32_t g_uartRejects = 0;   // 从 ttyS3 收到的、不在上位机白名单里的行
static char g_uartRejectLast = 0;

static void handleLine(String line, bool fromUart = false) {
  g_holdCmd = !fromUart;
  line.trim();
  if (line.isEmpty()) return;
  char c = line[0];
  String rest = line.substring(1);
  rest.trim();
  float a[3];
  int n;

  // 🔴 2026-09-14 车在桌上自己跑了起来：ttyS3 上的杂散字节被解析成 't'(引脚自检，里面
  // 把 PWM 拉到 50% 转 3 秒)。上位机只用得到下面这几个，其余一律不从串口执行。
  // 'N' 放进来是为了让 Looper 不插 COM 线也能查网桥状态；它的子命令在 case 'N' 里还有
  // 一道更严的过滤，只放行只读的那几个。
  // ? 只打印参数不动执行器，而板上的标定工具(diffcar_calib_vio.py)靠它现读轮周长/轮距/ppr。
  // 白名单是为了挡住充电器的杂散字节让车自己跑，只读命令没有这个风险。
  if (fromUart && !strchr("ucspeqzKUyN?", c)) { g_uartRejects++; g_uartRejectLast = c; return; }

  switch (c) {
    case 'f': drive(g_speed, g_speed); break;
    case 'b': drive(-g_speed, -g_speed); break;
    case 'l': drive(-g_speed, g_speed); break;
    case 'r': drive(g_speed, -g_speed); break;
    case 's': stopAll(); break;
    case 'v': g_speed = constrain(rest.toInt(), 0, PWM_MAX); break;
    case 'N':
      // ttyS3 那一路只放行只读查询。n/x 开关网桥、c 断 USB、i/k 是故障注入，而 s 的
      // scanNetworks 实测阻塞 6.4 秒，会把 50ms 的控制环一起卡死。d/dv 是掉线演练，只断几秒 WiFi。
      if (fromUart && !(rest == "" || rest == "l" || rest == "v" || rest == "t" ||
                        rest == "w" || rest == "z" || rest == "d" || rest == "dv" || rest == "dx" || rest == "dh" ||
                        rest == "b0" || rest == "b1" ||
                        rest == "ap" || rest == "apx" || rest.startsWith("w") ||
                        rest == "m" || rest == "m0" || rest == "m1" || rest == "mc")) {
        g_uartRejects++; g_uartRejectLast = 'N'; return;
      }
      if (rest == "r") { wifitestResetStats(); io.println(F("统计已清零")); }
      else if (rest == "s") wifitestScan(io);
      else if (rest.startsWith("d") && rest.length() <= 2) wifitestDrill(rest.length() == 2 ? rest[1] : 'd', io);
      else if (rest.startsWith("u")) wifitestUdpStart(rest.substring(1), io);
      // Nap：开 5 分钟热点（到点回到用户选的模式）；Napx：立刻关热点（热点模式下会马上重开，要换模式用 Nm0）
      else if (rest == "ap") wifitestApStart(1, 300000, io);
      // Nm：看网络模式；Nm0 连路由器 / Nm1 车的热点（存闪存，1.5 s 后切换）；Nmc 确认从热点切出来的路由器模式
      else if (rest == "m" || rest == "m0" || rest == "m1" || rest == "mc") wifitestModeCmd(rest, io);
      else if (rest == "apx") wifitestApStop(io);
      // Nwl/Nwb/Nwa/Nwc/Nwd：看和改 WiFi 候选表（网页的「网络」区块走这条）。Nw 单独一个字母是看门狗自检，不能吞掉
      else if (rest.startsWith("w") && rest.length() >= 2) wifitestListCmd(rest, io);
      // Nb0/Nb1：临时关/恢复低压保护，只改内存，重启回到闪存里的值。给低电量测试用，精确匹配防杂散字节。
      else if (rest == "b0") { g_vLow = 0; io.printf("低压保护已临时关闭(当前 %.2fV)，重启或 Nb1 恢复\n", g_vFilt); }
      else if (rest == "b1") { g_vLow = g_prefs.getFloat("vlow95", 9.5f); io.printf("低压保护已恢复 %.2fV\n", g_vLow); }
      else if (rest.startsWith("b")) { stopAll(); wifitestBlast(rest.substring(1), io); }
      // 开桥接后 WiFi 收包被接管，ESP32 自己的 lwIP 就收不到了 —— 上面那些测速命令随之失效，
      // 这正是产品行为：它只做网桥，IP 在 X5 那边。
      else if (rest == "n") { ncmSetAuto(true); ncm_bridge_start(); io.println(F("NCM 网桥已开(本机网络功能随之失效)，且已设为开机自启")); }
      else if (rest == "x") { ncmSetAuto(false); ncm_bridge_stop(); io.println(F("NCM 网桥已关，开机也不再自启")); }
      else if (rest == "c") { ncm_usb_recycle(); io.println(F("USB 已断开重连，等主机重新枚举")); }
      else if (rest == "w") {
        // 看门狗自检：故意把主循环空转住，不喂心跳。装了不验 = 不知道它到底能不能用。
        io.printf("故意卡住主循环 40 秒，看门狗应在 %d 秒内重启并报卡点=115\n", LOOP_STUCK_S);
        uint32_t t0 = millis();
        while (millis() - t0 < 40000) { }
        io.println(F("🔴 40 秒过去了看门狗没动 —— 它是坏的"));
      }
      else if (rest == "z") {
        g_wdtPeriodMs = (g_wdtPeriodMs == 1000) ? 30000 : 1000;
        io.printf("看门狗唤醒周期 -> %lu ms%s\n", (unsigned long)g_wdtPeriodMs,
                  g_wdtPeriodMs == 1000 ? "" : " (对照实验：≈等价于没有这个任务)");
      }
      else if (rest == "t") {
        uint32_t kb = 0, fr = 0, bs = 0;
        ncm_usb_speedtest(3000, &kb, &fr, &bs);
        io.printf("USB 段纯速率: %lu 帧×1400B / 3s = %lu KB/s  (can_xmit 拒绝 %lu 次)\n",
                  (unsigned long)fr, (unsigned long)kb, (unsigned long)bs);
      }
      else if (rest == "i") {
        ncm_set_test_ip();
        io.println(F("已人为塞入测试 IP 192.0.2.222 —— 下一轮 autoTick 应清掉并计数"));
      }
      // ⚠️ 字母不能用 s：上面 wifitestScan 已经占了 rest=="s"，写在后面就是永远不可达的
      // 死代码。三轮"看门狗造不出死锁"全是栽在这上面，而它不报错、只是静默走了扫描分支。
      else if (rest == "k") {
        ncm_dbg_force_stuck();
        io.println(F("已人为制造 NTB 死锁 —— 约 30s 后看门狗应自愈重启，之后用 Nl 看快照"));
      }
      else if (rest == "f") {
        saveNcmFault("手工触发(仅验证快照机制)");
        io.println(F("已写入一条故障快照，用 Nl 读回"));
      }
      else if (rest == "l") {
        Preferences p;
        if (!p.begin("ncmfault", true) || p.getUInt("n", 0) == 0) {
          io.println(F("没有故障快照记录"));
        } else {
          io.printf("上次自愈: 累计%lu次/连续%lu次 原因=%s\n",
                    (unsigned long)p.getUInt("n",0), (unsigned long)p.getUInt("run",0),
                    p.getString("why","?").c_str());
          io.printf("  当时: 运行%lums 提交%lu/完成%lu 偏移%lu 帧数%lu alt%lu\n",
                    (unsigned long)p.getUInt("ms",0), (unsigned long)p.getUInt("xsub",0),
                    (unsigned long)p.getUInt("xdone",0), (unsigned long)p.getUInt("off",0),
                    (unsigned long)p.getUInt("dgc",0), (unsigned long)p.getUInt("alt",0));
          io.printf("  计数: NTB容量满%lu 卡死%lu 丢发送%lu 收帧%lu 发帧%lu\n",
                    (unsigned long)p.getUInt("nxsize",0), (unsigned long)p.getUInt("stalls",0),
                    (unsigned long)p.getUInt("droptx",0), (unsigned long)p.getUInt("rxf",0),
                    (unsigned long)p.getUInt("txf",0));
          io.printf("  WiFi: status=%lu rssi=%d dBm\n",
                    (unsigned long)p.getUInt("wst",0), p.getInt("rssi",0));
          io.printf("  寄存器(卡死时): %s\n", p.getString("reg1","-").c_str());
          io.printf("  寄存器(重启前): %s\n", p.getString("reg2","-").c_str());
          { char rn[220]; ncmEpRegs(rn, sizeof rn); io.printf("  寄存器(现在): %s\n", rn); }
        }
        p.end();
      }
      else if (rest == "v") {
        // 唯一能问出"对面供不供 VBUS"的办法：把 override 撤掉一瞬间读真值。
        // 撤掉期间会话会断，所以立刻恢复；链路是否需要重新枚举由主机决定。
        uint32_t keep = USB0.gotgctl;
        USB0.gotgctl &= ~(USB_VBVALIDOVEN | USB_AVALIDOVEN | USB_BVALIDOVEN);
        delay(20);
        uint32_t og = USB0.gotgctl;
        USB0.gotgctl = keep;
        io.printf("真实 VBUS 检测: B会话=%d A会话=%d (1=对面在供 5V, 0=不供)\n",
                  (int)!!(og & USB_BSESVLD), (int)!!(og & USB_ASESVLD));
        io.println(F("override 已恢复。若链路没自己回来，发 Nc 重新枚举"));
      }
      else {
        wifitestReport(io);
        io.printf("串口拒收命令: %lu 次 (最近一个 '%c')\n",
                  (unsigned long)g_uartRejects, g_uartRejectLast ? g_uartRejectLast : '-');
        uint32_t us = ncm_usb_state();
        io.printf("USB 设备侧: 检到VBUS=%d 已被枚举=%d 挂起=%d 发送卡死自愈=%lu 次\n",
                  (int)(us & 1), (int)((us >> 1) & 1), (int)((us >> 2) & 1),
                  (unsigned long)ncm_stalls);
        // ⚠️ override 开着时 BSESVLD/ASESVLD 也被驱动成 1，这里读到的不是真实 VBUS。
        // 要测对面到底供不供电用 Nv。
        uint32_t og = USB0.gotgctl;
        io.printf("GOTGCTL=0x%08lx VBUS override=%s B会话=%d A会话=%d 连接ID=%d\n",
                  (unsigned long)og, (og & USB_BVALIDOVEN) ? "开" : "关",
                  (int)!!(og & USB_BSESVLD), (int)!!(og & USB_ASESVLD),
                  (int)!!(og & USB_CONIDSTS));
        io.printf("桥接卫生: STA占着IP=%d(桥接时必须为0) 累计回收IP %lu 次\n",
                  (int)ncm_sta_has_ip(), (unsigned long)ncm_ip_reclaims);
        io.printf("心跳: 已发 %lu 帧 | 报给板子 link=%u gen=%u ap=%d rssi=%d reason=%u\n",
                  (unsigned long)ncm_hb_sent, (unsigned)ncm_hb_link, (unsigned)ncm_hb_gen,
                  (int)ncm_hb_ap, (int)ncm_hb_rssi, (unsigned)ncm_hb_reason);
        io.printf("NTB发送态: 发送中=%lu (提交%lu/完成%lu, 差%ld 提交失败%lu) 偏移=%lu/%lu 帧数=%lu alt=%lu\n",
                  (unsigned long)ncm_dbg_transferring(),
                  (unsigned long)ncm_xfer_submit, (unsigned long)ncm_xfer_done,
                  (long)(ncm_xfer_submit - ncm_xfer_done), (unsigned long)ncm_xfer_fail,
                  (unsigned long)ncm_dbg_offset(), (unsigned long)ncm_dbg_ntbsize(),
                  (unsigned long)ncm_dbg_dgcount(), (unsigned long)ncm_dbg_alt());
        io.printf("can_xmit 拒绝原因: 接口未就绪=%lu NTB帧数满=%lu NTB容量满=%lu\n",
                  (unsigned long)ncm_nx_alt, (unsigned long)ncm_nx_count,
                  (unsigned long)ncm_nx_size);
        // 非 0 = 撞上过「提交失败后没人再踢」那个死锁。它曾经让 IN 方向永久停而全程零日志
        io.printf("提交失败后重踢发送 %lu 次 (非0=撞上过 NTB 死锁)；最近一次失败时硬件 %s\n",
                  (unsigned long)ncm_tx_restart, epStateStr(ncm_fail_ep_state));
        io.printf("端点自愈: 踢端点 %lu 次(主机无感，不用重新枚举) | 一次NTB提交到完成最长 %lu ms\n",
                  (unsigned long)ncm_ep_unstick, (unsigned long)ncm_xmit_max_ms);
        io.printf("USB 驱动补丁(11-10): 保留位15 %lu 次(原版会重置总线) | 空中断时放不下 %lu 次 | 续传 成功%lu/失败%lu | 完成未空 %lu | IN超时 %lu\n",
                  (unsigned long)dcdx_unknown15, (unsigned long)dcdx_nospace, (unsigned long)dcdx_rearm_ok,
                  (unsigned long)dcdx_rearm_fail, (unsigned long)dcdx_cmpl_notempty, (unsigned long)dcdx_in_timeout);
        io.printf("电压塌陷 %lu 次(阈值%.1fV)。ESP32 不跟着 Looper 重启，所以这条跨 Looper 复位仍可查\n",
                  (unsigned long)g_sagCnt, SAG_TH);
        if (!g_sagCnt) {
          io.println(F("  (无 —— 从 ESP32 上次上电起没跌破过)"));
        } else {
          uint32_t _n = g_sagCnt < SAG_N ? g_sagCnt : SAG_N;
          for (uint32_t _k = 0; _k < _n; _k++) {
            uint32_t _i = (g_sagCnt - _n + _k) % SAG_N;
            io.printf("  %lus 前  V=%.2f  左PWM=%d 右PWM=%d%s\n",
                      (unsigned long)((millis() - g_sag[_i].ms) / 1000), g_sag[_i].v,
                      (int)g_sag[_i].pl, (int)g_sag[_i].pr,
                      (g_sag[_i].pl || g_sag[_i].pr) ? "  ← 电机在动" : "  ← 静止时就塌了");
          }
        }
        {
          // 上位机的 IP 是从它发的 ARP/IP 包里抓的（网桥本来不看三层）。换网段之后
          // "板子现在是什么地址"只能靠人在手机上试，这条让 COM 口直接答得上来。
          uint32_t hip = ncm_host_ip, hms = ncm_host_ip_ms;
          char ipbuf[40];
          if (hip) snprintf(ipbuf, sizeof(ipbuf), "%lu.%lu.%lu.%lu(%lus前)",
                            (unsigned long)(hip >> 24), (unsigned long)((hip >> 16) & 0xFF),
                            (unsigned long)((hip >> 8) & 0xFF), (unsigned long)(hip & 0xFF),
                            (unsigned long)((millis() - hms) / 1000));
          else snprintf(ipbuf, sizeof(ipbuf), "还没见过(板子没发过包)");
          io.printf("上位机: 灯=%s | IP=%s | 距上次 ttyS3 数据 %lus | 网页连着=%d"
                    " | 累计 静默%lu次/网页断开%lu次\n",
                    ledName(g_ledState), ipbuf,
                    (unsigned long)((millis() - g_lastUart) / 1000),
                    (int)g_pcOk, (unsigned long)g_silentCnt, (unsigned long)g_pcBadCnt);
        }
      }
      return;
    case 'd': demoSequence(); return;
    case 't': pinTest(); return;
    case 'j': calibEncDir(); return;
    case 'X':
      if (rest.length()) {
        if (fromUart) { io.println(F("X 不能从串口自己发给自己")); return; }
        Serial1.print(rest);
        Serial1.print('\n');
        Serial1.flush();
        io.printf("已从 GPIO%d(TX1) 发出: \"%s\"\n", PIN_X5_TX, rest.c_str());
        uartListen(2000);
      } else {
        uartListen(3000);
      }
      return;
    case 'U':
      if (!fromUart) { io.println(F("U 只能从 Looper 串口发，见 looper/flash.py")); return; }
      otaRecv(rest);
      return;
    case 'K':                     // 上位机报告看车的网页是否连着（网页后端判断），我们只管显示
      g_pcOk = rest.toInt() != 0;
      g_pcStamp = millis();
      if (!fromUart) io.printf("PC 可达=%d\n", (int)g_pcOk);
      return;
    case 'P': pinMap(); return;
    case 'e': encStatus(); return;
    case '?': help(); return;
    case 'w': saveParams(); return;
    case 'z': encZero(); io.println(F("计数已清零")); return;
    case 'y':
      g_telem = !g_telem;
      g_telemUart = g_telem && fromUart;
      io.printf("遥测: %s\n", g_telem ? "开" : "关");
      return;
    case 'g':
      g_guard = !g_guard;
      io.printf("失联保护: %s\n", g_guard ? "开" : "关");
      return;
    case 'L':
      g_ledLvl = constrain(rest.toInt(), 0, 255);
      ledRaw(0, 0, 0);  // 强制下一次刷新
      io.printf("状态灯亮度 %d\n", g_ledLvl);
      return;
    case 'V':
      g_vNom = max(0.0f, rest.toFloat());
      io.printf("掉压补偿基准电压 %.2fV %s\n", g_vNom, g_vNom < 1 ? "(=关闭)" : "");
      return;
    case 'B':
      n = parseNums(rest, a, 2);
      if (!n) { io.println(F("用法: B <倍率> [低压阈值V]")); return; }
      g_vbatK = max(0.01f, a[0]);
      if (n > 1) g_vLow = max(0.0f, a[1]);
      io.printf("分压倍率 %.3f → 当前 %.2fV；低压阈值 %.2fV %s\n", g_vbatK, vbat(), g_vLow,
                g_vLow < 1 ? "(=关闭)" : "");
      return;
    case 'F':
      if (rest.length()) g_brk = constrain(rest.toInt(), 0, STALL_PWM - 1);
      io.printf("静摩擦击穿 %d PWM %s（判据 |实测|<%.3f 持续 %.2fs）\n", g_brk,
                g_brk ? "" : "(=关闭)", BRK_V, BRK_HOLD);
      return;
    case 'a':
      // 不带参数只显示。裸敲会让 toFloat() 返 0 → 限幅被设成 0.05，刹车斜坡变 8 秒
      if (rest.length()) g_accel = max(0.05f, rest.toFloat());
      io.printf("加速度上限 %.2f m/s^2\n", g_accel);
      return;
    case 'A':
      if (rest.length()) g_decel = max(0.05f, rest.toFloat());
      io.printf("减速度上限 %.2f m/s^2（0.40->0 需 %.0f ms）\n", g_decel, 400.0f / g_decel);
      return;
    case 'o': {
      float d[3] = {0, 240, 20};
      parseNums(rest, d, 3);
      sweep((int)d[0], (int)d[1], max(1, (int)d[2]));
      return;
    }
    case 'n':
      n = parseNums(rest, a, 2);
      if (!n) { io.println(F("用法: n <左> [右]")); return; }
      g_wl.ppr = max(1.0f, a[0]);
      g_wr.ppr = max(1.0f, n > 1 ? a[1] : a[0]);
      io.printf("脉冲/圈: 左=%.1f 右=%.1f\n", g_wl.ppr, g_wr.ppr);
      return;
    case 'x':
      if (parseNums(rest, a, 2) < 2) { io.println(F("用法: x <左kff> <右kff>")); return; }
      g_wl.kff = a[0];
      g_wr.kff = a[1];
      io.printf("kff: 左=%.1f 右=%.1f\n", g_wl.kff, g_wr.kff);
      return;
    case 'i':
      if (parseNums(rest, a, 2) < 2) { io.println(F("用法: i <左截距> <右截距>")); return; }
      g_wl.deadband = constrain((int)a[0], 0, PWM_MAX);
      g_wr.deadband = constrain((int)a[1], 0, PWM_MAX);
      io.printf("截距: 左=%d 右=%d\n", g_wl.deadband, g_wr.deadband);
      return;
    case 'k':
      if (parseNums(rest, a, 2) < 2) { io.println(F("用法: k <kp> <ki>")); return; }
      g_kp = a[0];
      g_ki = a[1];
      g_wl.integ = g_wr.integ = 0;  // 改参数就重来，别让旧积分带着跑
      io.printf("kp=%.1f ki=%.1f\n", g_kp, g_ki);
      return;
    case 'p':
      // vl/vr 挂在位姿这一行上，不另开一条命令：`p` 已经被上位机 20 Hz 轮询，而
      // 编码器实测速度以前只有 5 Hz 的 `y` 遥测和人工的 `e` 能看到，上位机拿不到。
      io.printf("里程计: x=%.3f y=%.3f theta=%.1f度 (%.3f rad) vl=%.3f vr=%.3f\n", g_ox, g_oy,
                g_oth * 180.0f / PI, g_oth, g_wl.vMeas, g_wr.vMeas);
      return;
    case 'q':
      g_ox = g_oy = g_oth = 0;
      io.println(F("里程计已清零"));
      return;
    case 'c':
      n = parseNums(rest, a, 2);
      if (!n) { io.println(F("用法: c <v> [vR]")); return; }
      setTargets(a[0], n > 1 ? a[1] : a[0]);
      reportTarget(fromUart);
      return;
    case 'u':
      n = parseNums(rest, a, 2);
      if (!n) { io.println(F("用法: u <前进m/s> [转向rad/s]")); return; }
      setTwist(a[0], n > 1 ? a[1] : 0);
      reportTarget(fromUart);
      return;
    case 'm':
      if (parseNums(rest, a, 2) < 2) { io.println(F("用法: m <左> <右>")); return; }
      drive((int)a[0], (int)a[1]);
      break;
    default: io.println(F("未知命令，输入 ? 看帮助")); return;
  }
  io.printf("速度=%d  左=%d 右=%d\n", g_speed, g_wl.pwm, g_wr.pwm);
}

static void controlWheel(Wheel &w, float dt) {
  int32_t c = encRead(w);
  w.dS = (c - w.lastCount) / w.ppr * WHEEL_CIRC;
  w.lastCount = c;
  w.vMeas += 0.5f * (w.dS / dt - w.vMeas);  // 一阶低通

  // 给了大 PWM 却几乎不动 = 堵转或编码器断线，两种都得赶紧停
  if (abs(w.pwm) > STALL_PWM && fabsf(w.vMeas) < STALL_V) w.stallT += dt;
  else w.stallT = 0;

  if (!g_closed) return;
  // 判据是「目标是否更靠近零」，这样换向时先按减速走到零、过零后再按加速走
  const bool braking = fabsf(w.target) < fabsf(w.tRamp);
  float step = (braking ? g_decel : g_accel) * dt;
  w.tRamp = w.tRamp < w.target ? min(w.target, w.tRamp + step)
                               : max(w.target, w.tRamp - step);
  float err = w.tRamp - w.vMeas;
  // 爬坡期间冻结积分：测量有滤波延迟，此时误差恒为正，攒下的积分会在到顶后顶过头
  if (w.tRamp == w.target) w.integ += err * dt;
  float lim = PWM_MAX / max(g_ki, 1.0f);
  w.integ = constrain(w.integ, -lim, lim);  // 抗积分饱和
  // 停住时前馈必须整个归零：截距那几个 PWM 会让轮子一直蠕动
  float ff = w.tRamp == 0 ? 0.0f
                          : (w.tRamp > 0 ? 1 : -1) * (w.deadband + w.kff * fabsf(w.tRamp));
  int out = (int)(ff * vComp() + g_kp * err + g_ki * w.integ);
  // 目标非零而轮子确实没动：把 PWM 顶到能挣脱的量级，动起来就自动松手。夹在堵转门限
  // 以下，否则这一脚自己会把堵转保护踩响。方向跟目标而不是跟 out —— 反向起步时 integ
  // 还留着旧符号，跟 out 会把车往反方向踹。
  if (g_brk > 0 && w.tRamp != 0.0f && fabsf(w.vMeas) < BRK_V) {
    w.brkT += dt;
    if (w.brkT > BRK_HOLD) {
      int mag = min(g_brk, STALL_PWM - 1);
      if (abs(out) < mag) out = (w.tRamp > 0 ? mag : -mag);
    }
  } else {
    w.brkT = 0;
  }
  setWheel(w, out);
}

// 🔴 主循环卡死时唯一的出路。互斥锁死锁只堵住等锁的任务，独立任务照样被调度，所以它跑得了；
// 钉在 core 0 是因为主循环在 core 1，万一是自旋独占那个核，同核的任务一样起不来。
static void loopWdtTask(void *arg) {
  (void)arg;
  uint32_t last = 0, lastChange = millis();
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(g_wdtPeriodMs));
    uint32_t now = millis();
    g_subHist[g_subHistIdx & 7] = g_sub; g_subHistIdx++;   // 每秒一个样本，8 秒的历史
    if (g_beat != last) { last = g_beat; lastChange = now; continue; }
    if (now - lastChange < (uint32_t)LOOP_STUCK_S * 1000) continue;
    g_stuckAt = g_stage; g_stuckAux = g_stageAux; g_stuckBeat = g_beat;
    g_stuckWant = ncm_lock_want; g_stuckSub = g_sub;
    snprintf(g_stuckHolder, sizeof(g_stuckHolder), "%s", ncm_mutex_holder_name());
    extern TaskHandle_t loopTaskHandle;
    g_stuckLoopState = loopTaskHandle ? (uint32_t)eTaskGetState(loopTaskHandle) : 99;
    TaskHandle_t c1 = xTaskGetCurrentTaskHandleForCPU(1);
    snprintf(g_stuckCore1, sizeof(g_stuckCore1), "%s", c1 ? pcTaskGetName(c1) : "?");
    g_stuckHeap = esp_get_free_heap_size();     // 卡死那一刻的堆，看是不是内存耗尽
    g_stuckOwner = ncm_lock_owner;              // 谁把 NCM 锁占着不放
    g_stuckLockSeq = ncm_lock_seq;              // 与上一轮比：不动=锁真被占死
    g_stuckRecStep = ncm_recover_step;          // 卡在 recover 的第几句
    g_stuckMagic = STUCK_MAGIC;
    esp_restart();
  }
}

void setup() {
  // 先把 STBY 拉低：GPIO46 复位期间靠内部下拉挡着，这里接管后继续压住直到收到命令
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);

  g_rst = esp_reset_reason();
  // ⚠️ 加/删 RTC 变量必须同时换 magic：布局变了而 magic 没变，上一版的残值会被当成
  // 真实测量报出来(注释在下面，踩过一次，报了个像模像样的 10.52V)
  if (g_bootMagic != 0x5AA5C346) {  // 上电首启：RTC 域刚通电，里面是随机值
    g_bootMagic = 0x5AA5C346;
    g_stuckMagic = 0; g_stage = 0; g_stuckAt = 0; g_stageAux = 0; g_stuckAux = 0; g_stuckHeap = 0;
    g_stuckOwner = 0; g_stuckLockSeq = 0; g_stuckBeat = 0; g_stuckWant = 0; g_stuckSub = 0; g_stuckHolder[0] = 0; g_stuckLoopState = 0;
    g_stuckCore1[0] = 0; g_subHistIdx = 0;
    for (int i = 0; i < 8; i++) g_subHist[i] = 0;
    g_bootCnt = 0;
    g_vMinRtc = 99;
    g_sagCnt = 0;
    for (int i = 0; i < SAG_N; i++) { g_sag[i].ms = 0; g_sag[i].v = 0; g_sag[i].pl = g_sag[i].pr = 0; }
  }
  g_bootCnt++;
  // 范围校验不能省：改固件时 RTC 变量的布局会变，而 magic 仍然有效，
  // 于是上一版留下的残值会被当成真实测量报出来(踩过一次，报了个像模像样的 10.52V)
  g_vMinLast = (g_vMinRtc > 0 && g_vMinRtc < 30) ? g_vMinRtc : 99;
  g_vMinRtc = 99;  // 交接完就清，让下一段重新记

#if CONSOLE_HAS_USB
  Serial.begin(115200);   // 原生 USB 口
#endif
  Serial0.begin(115200);  // CH343 口
  // OTA 时对面连着灌 1MB，而写 flash 会阻塞几十毫秒；默认 256B 缓冲(约 22ms 的量)必溢出。
  // 必须在 begin 之前设，begin 之后再碰 UART1 是禁区
  Serial1.setRxBufferSize(8192);
  Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_X5_RX, PIN_X5_TX);  // 接 Looper，全程只 begin 这一次

  g_rmt[0] = rmtInit(PIN_RGB_A, RMT_TX_MODE, RMT_MEM_64);
  g_rmt[1] = rmtInit(PIN_RGB_B, RMT_TX_MODE, RMT_MEM_64);
  for (auto *h : g_rmt)
    if (h) rmtSetTick(h, 100);
  // 启动色用蓝，不用白：固件里任何状态都不再输出白色，于是"看到白灯"就唯一地
  // 指向 WS2812 收到坏数据锁存了全 1，和重启(蓝一闪)彻底分开，不用再靠猜
  ledRaw(0, 0, g_ledLvl);

  analogSetPinAttenuation(PIN_VBAT, ADC_11db);  // 量程拉到约 3.1V

  for (Wheel *w : {&g_wl, &g_wr}) {
    pinMode(w->pinIn1, OUTPUT);
    pinMode(w->pinIn2, OUTPUT);
    digitalWrite(w->pinIn1, LOW);
    digitalWrite(w->pinIn2, LOW);
    ledcSetup(w->ch, PWM_FREQ, PWM_BITS);
    ledcAttachPin(w->pinPwm, w->ch);
    ledcWrite(w->ch, 0);
  }

  encInit(g_wl);
  encInit(g_wr);

  loadParams();
  delay(1500);  // 等 USB CDC 枚举完，否则开头的输出会丢
  while (io.available()) io.read();
  io.printf("\n就绪 (第 %lu 次启动，复位原因: %s)\n", (unsigned long)g_bootCnt, resetName(g_rst));
  io.println(F("输入 ? 看帮助和当前参数，P 看接线表"));
  // 也主动告诉 Looper 一声。上位机据此就能发现"车偷偷重启了"——里程计得清、
  // 参数得重设，靠猜是猜不出来的。以后的 ROS 2 节点也用这一行
  Serial1.printf("BOOT %lu %s 塌前最低=%.2fV\n", (unsigned long)g_bootCnt, resetName(g_rst),
                 g_vMinLast > 90 ? 0.0f : g_vMinLast);
  ncm_usb_register();   // 必须在 USB.begin() 之前
  USB.begin();          // CDC 关掉后 core 不再自己起 USB，要自己来
  usbForceSessionValid();   // 必须在 USB.begin() 之后：begin 会重置 OTG 核
  wifitestBegin();
  xTaskCreatePinnedToCore(ctrlTask, "ctrl", 4096, NULL, 10, NULL, 1);
  if (g_stuckMagic == STUCK_MAGIC) {
    // setup 会把 magic 清掉，所以完整现场必须在这里就留给 N。
    // 🔴 只留"卡点+锁持有者"不够：2026-09-22 查冷启动慢，正是缺了圈数和 recover 步
    //    才判不出是真卡死还是没跑起来 —— 而 [WDT] 那行走 Serial1，开机 90 秒内没人读。
    snprintf(g_stuckNote, sizeof(g_stuckNote),
             "  ⚠️上次主循环卡死重启: 心跳%lu 卡点%lu.%lu 堆%lu 锁在[%s]手里 等锁位图0x%lx "
             "| 主循环任务状态=%s core1上在跑[%s] | g_sub最近8秒 %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
             (unsigned long)g_stuckBeat, (unsigned long)g_stuckAt,
             (unsigned long)g_stuckSub, (unsigned long)g_stuckHeap,
             g_stuckHolder[0] ? g_stuckHolder : "?", (unsigned long)g_stuckWant,
             g_stuckLoopState == 0 ? "在跑" : g_stuckLoopState == 1 ? "🔴就绪但没被调度(=被饿死)"
               : g_stuckLoopState == 2 ? "🔴阻塞(=真卡在某个等待上)"
               : g_stuckLoopState == 3 ? "被挂起" : g_stuckLoopState == 4 ? "已删除" : "未知",
             g_stuckCore1[0] ? g_stuckCore1 : "?",
             (unsigned long)g_subHist[(g_subHistIdx + 0) & 7], (unsigned long)g_subHist[(g_subHistIdx + 1) & 7],
             (unsigned long)g_subHist[(g_subHistIdx + 2) & 7], (unsigned long)g_subHist[(g_subHistIdx + 3) & 7],
             (unsigned long)g_subHist[(g_subHistIdx + 4) & 7], (unsigned long)g_subHist[(g_subHistIdx + 5) & 7],
             (unsigned long)g_subHist[(g_subHistIdx + 6) & 7], (unsigned long)g_subHist[(g_subHistIdx + 7) & 7]);
    g_stuckMagic = 0;
    const char *ow = g_stuckOwner == 1 ? "recover(主循环自己)" : g_stuckOwner == 2 ? "xfer_cb(传输完成)"
                   : g_stuckOwner == 3 ? "can_xmit" : g_stuckOwner == 4 ? "xmit(发包)"
                   : g_stuckOwner == 5 ? "alt切换" : "没人(锁是空的)";
    const char *rs = g_stuckRecStep == 61 ? "edpt_stall" : g_stuckRecStep == 62 ? "edpt_clear_stall"
                   : g_stuckRecStep == 92 ? "edpt_xfer(提交发送)" : g_stuckRecStep == 2 ? "tx_watch"
                   : g_stuckRecStep == 5 ? "heal_step" : g_stuckRecStep == 90 ? "start_tx入口"
                   : g_stuckRecStep == 0 ? "不在recover里" : "见编号";
    Serial1.printf("[WDT] 上次主循环卡死，卡点=%lu 圈数=%lu 堆=%lu 锁持有者=%lu %s 锁流水=%lu recover步=%lu %s\n",
                   (unsigned long)g_stuckAt, (unsigned long)g_stuckAux, (unsigned long)g_stuckHeap,
                   (unsigned long)g_stuckOwner, ow, (unsigned long)g_stuckLockSeq,
                   (unsigned long)g_stuckRecStep, rs);
    io.printf("[WDT] 上次主循环卡死，卡点=%lu 圈数=%lu 堆=%lu 锁持有者=%lu %s 锁流水=%lu recover步=%lu %s\n",
              (unsigned long)g_stuckAt, (unsigned long)g_stuckAux, (unsigned long)g_stuckHeap,
              (unsigned long)g_stuckOwner, ow, (unsigned long)g_stuckLockSeq,
              (unsigned long)g_stuckRecStep, rs);
  }
  xTaskCreatePinnedToCore(loopWdtTask, "loopwdt", 2560, NULL, 20, NULL, 0);
  g_ledState = LED_OK;
}

// Looper 通过 UART1 发来的命令。和终端用同一个解析器，只是标记来源不同：
// 串口来的不续期失联保护，上位机一挂车就停
static void pollUart() {
  static String ubuf;
  while (Serial1.available()) {
    char c = Serial1.read();
    g_lastUart = millis();
    if (c == '\n' || c == '\r') {
      if (ubuf.length()) {
        io.toUart = true;
        handleLine(ubuf, true);
        io.toUart = false;
        ubuf = "";
      }
    } else if (c >= 0x20 && c < 0x7f && ubuf.length() < 200) {
      // 截断后命令报"用法错误"，而看不出是被截了 —— OTA 的 md5 踩过一次。
      // 200：最长的是 Nwa，32 字节 SSID + 63 位密码的十六进制共 195 字符
      ubuf += c;
    }
  }
}

static void ncmSetAuto(bool on) {
  g_ncmAuto = on;
  g_prefs.begin("car", false);
  g_prefs.putBool("ncmauto", on);
  g_prefs.end();
}

// 网桥和远程命令口正好互补：网桥关着时 ESP32 有自己的 IP，UDP 9004 能进来；
// 网桥开着时 X5 有网，走 X5 的串口进来。两种状态下都留了一条路，不会再像
// 2026-09-11 那样两头都够不着。
// 自愈重启前把现场存进 NVS。重启会把 RAM 里的一切冲掉，不留快照等于白重启 ——
// 而这个死锁恰恰只能靠重启解，所以快照是唯一能留下的证据。断电也不丢。
// NCM IN 端点的硬件现场。卡死时只有寄存器分得开：传输使能了却没人填 FIFO（空中断屏蔽位丢了，
// 嫌疑是 dcd_esp32sx 跨核读改写 fifoemptymsk）还是 FIFO 有货主机不取。11-10 加，纯诊断。
static char g_epReg1[220];   // 第一次判卡死(重新枚举之前)的现场
static void ncmEpRegs(char *b, size_t n) {
  uint8_t e = ncm_dbg_ep_in() & 0x7f;
  if (!e || e >= USB_IN_EP_NUM) { snprintf(b, n, "ep?=%u", e); return; }
  volatile usb_in_endpoint_t *r = &USB0.in_ep_reg[e];
  uint32_t ctl = r->diepctl, tsz = r->dieptsiz;
  snprintf(b, n, "ep%u 使能%lu NAK%lu 激活%lu | 剩%lu包%lu字节 | int=%02lx | FIFO空闲%lu字 | 空中断屏蔽%lu daint屏蔽%lu | gintmsk=%08lx gintsts=%08lx",
           e, (unsigned long)((ctl >> 31) & 1), (unsigned long)((ctl >> 17) & 1), (unsigned long)((ctl >> 15) & 1),
           (unsigned long)((tsz >> 19) & 0x3ff), (unsigned long)(tsz & 0x7ffff), (unsigned long)(r->diepint & 0xff),
           (unsigned long)(r->dtxfsts & 0xffff), (unsigned long)((USB0.dtknqr4_fifoemptymsk >> e) & 1),
           (unsigned long)((USB0.daintmsk >> e) & 1), (unsigned long)USB0.gintmsk, (unsigned long)USB0.gintsts);
}

static void saveNcmFault(const char *why) {
  Preferences p;
  if (!p.begin("ncmfault", false)) return;
  p.putUInt("n", p.getUInt("n", 0) + 1);          // 累计自愈次数
  p.putUInt("run", p.getUInt("run", 0) + 1);      // 连续自愈次数(恢复后清零)
  p.putString("why", why);
  p.putUInt("ms", millis());
  p.putUInt("xsub", ncm_xfer_submit);
  p.putUInt("xdone", ncm_xfer_done);
  p.putUInt("off", ncm_dbg_offset());
  p.putUInt("dgc", ncm_dbg_dgcount());
  p.putUInt("alt", ncm_dbg_alt());
  { char r2[220]; ncmEpRegs(r2, sizeof r2); p.putString("reg1", g_epReg1); p.putString("reg2", r2); }
  p.putUInt("nxsize", ncm_nx_size);
  p.putUInt("stalls", ncm_stalls);
  p.putUInt("droptx", ncm_drop_tx);
  p.putUInt("rxf", ncm_rx_frames);
  p.putUInt("txf", ncm_tx_frames);
  p.putInt("rssi", (int)WiFi.RSSI());
  p.putUInt("wst", (uint32_t)WiFi.status());
  p.end();
}

// NTB 发送死锁的看门狗。指纹是 transferring 卡在 1、且 submit-done 恒为同一个差值 ——
// 那次 xfer 主机永远不会取走，之后 ncm_start_tx() 全部空转、偏移顶死、can_xmit 永远
// 说满。实测这个状态不自愈、USB 重新枚举也救不回来，只有硬复位有效。
static void ncmStuckWatchdog() {
  static NcmWdSt wd{};
  static uint32_t cleared = 0;
  if (!ncm_bridge_running()) { wd.armed = 0; return; }
  if (millis() < 60000) return;                   // 开机冷却，别和启动过程抢

  // 判据用「done 不再增长」而不是「submit-done 差值不变」：差值在健康时也恒定
  // (每次提交失败都会让它永久 +1)，那个判据等于只看 transferring，会误杀。
  // 15 s：09-28 插回 USB 后主机一直在发、一帧不取，要赶在板子 61 s 判死转 device 之前自己复位
  bool fire = ncm_wd_step(ncm_dbg_transferring(), ncm_xfer_done, ncm_rx_frames, millis(),
                          ncm_enum_gen, 15000, &wd);
  if (!wd.armed) {
    // 稳跑 5 分钟就认为上一次自愈成功，清掉连续计数，免得它一直攒着误判成重启循环
    if (!cleared && millis() > 300000) {
      Preferences p;
      if (p.begin("ncmfault", false)) { p.putUInt("run", 0); p.end(); }
      cleared = 1;
    }
    return;
  }
  if (!fire) return;

  Preferences p; uint32_t run = 0;
  if (p.begin("ncmfault", true)) { run = p.getUInt("run", 0); p.end(); }
  if (run >= 5) {                                  // 连着救 5 次还不行，别再空转重启
    static uint32_t nag = 0;
    if (millis() - nag > 60000) {
      nag = millis();
      Serial1.printf("NCM 死锁且已连续自愈 %lu 次，停止自愈\n", (unsigned long)run);
    }
    return;
  }
  saveNcmFault("NTB xfer 卡死(transferring 不归零)");
  Serial1.printf("NCM 死锁自愈重启 (第 %lu 次连续)\n", (unsigned long)(run + 1));
  delay(150);
  esp_restart();
}

static void ncmAutoTick() {
  static NcmSt st;
  static uint32_t lastLinkChk = 0;
  static bool linkCache = false;
  const uint32_t now = millis();

  // esp_wifi_sta_get_ap_info 要进 WiFi 驱动拿锁，而 loop 每秒跑 2~3 万圈 —— 这个文件
  // 反复踩过裸调的坑(实测把经网桥的 ping 打到 12~20% 丢包)。它判的是链路断没断，秒级事件。
  if (now - lastLinkChk > 500) {
    lastLinkChk = now;
    wifi_ap_record_t ap;
    MARK(301);
    linkCache = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
    MARK(302);
    ncm_hb_link = linkCache;
    if (linkCache) ncm_hb_rssi = ap.rssi;
    if (ncm_link_ready()) ncm_tasks_start();
  }

  // 连上时 IDF 的默认处理会把 STA 收包回调换回 lwIP 的，和 CONNECTED 事件里的重挂分属两个任务、
  // 谁后跑谁赢。输了就是桥开着、射频连着、ESP32→板子整段断死（09-23 断开重连后断了 60 s+，
  // 直到 USB 角色守护重新枚举）。周期重挂兜底：同一个指针覆盖，没有副作用。
  static uint32_t lastRearm = 0;
  // 热点模式同理：AP 启动时 IDF 也会给 AP 接口挂上 lwIP 的回调
  if ((linkCache || ncm_hb_mode) && now - lastRearm > 2000) { lastRearm = now; ncm_bridge_rearm(); }

  NcmIn in;
  in.now = now;
  in.linkUp = linkCache;
  in.apMode = ncm_hb_mode != 0;
  in.usbReady = ncm_link_ready() != 0;
  MARK(303);
  in.bridgeOn = ncm_bridge_running() != 0;
  in.autoOn = g_ncmAuto;
  in.w2uIn = ncm_w2u_in;
  in.usbRxFrames = ncm_rx_frames;
  // 同样要拿 netif 锁，所以先按策略的节流窗口过滤再查
  MARK(304);
  in.staHasIp = (in.bridgeOn && now - st.lastIpChk > NCM_IPCHK_MS) ? (ncm_sta_has_ip() != 0) : false;
  MARK(305);

  NcmAct act;
  ncmPolicy(&in, &st, &act);

  MARK(306);
  if (act.stopBridge)  ncm_bridge_stop();     // 内含 disconnect + dhcpc_start + connect
  MARK(307);
  if (act.startBridge) {
    MARK(308);
    ncm_bridge_start();
    // 冷启动计时：从上电到网桥真正可用是多久。射频关联 != 能用 —— 网桥晚起的那几十秒里，
    // 板子发出去的 DHCP 请求回不来，表现就是"连上了但网页打不开"。
    if (!g_msBridge1) {
      g_msBridge1 = millis();
      bootEv("网桥开");
      wtLog("[T] 网桥首次开启 t=%lums\n", (unsigned long)g_msBridge1);
    }
  }
  MARK(309);
  if (act.clearIp)   { ncm_clear_sta_ip(); ncm_ip_reclaims++; }
  MARK(310);
}


// 🔴 电机控制必须独占一个高优先级任务。跑在 Arduino 的 loop() 里时它是全系统
// 优先级最低的，WiFi/USB/网桥一忙就被挤掉 —— 2026-09-14 实测转发大流量时控制
// 周期被拉到 2002 ms(标称 50)、loop 单圈最长 589 ms。
// 放 CPU1：WiFi 和网桥都在 CPU0，分开才不抢。实时段内不许有串口输出(会阻塞)。
static volatile uint8_t g_ctrlNotify = 0;   // bit0 低压触发 bit1 低压解除 bit2 堵转
// 实时任务的占用率：累计"真在干活"的微秒数，除以墙钟就是它吃掉多少 CPU
// g_ctrlBusyUs 累加的是微秒，32 位在 25% 占空比下约 4.6 小时就回绕，占比会变成假数。
volatile uint64_t g_ctrlBusyUs = 0;
volatile uint32_t g_ctrlWakes = 0, g_ctrlSince = 0;

static void ctrlTask(void *arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  uint32_t lastCtrl = millis(), lastVmin = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));   // 1 kHz 节拍，电压采样靠它
    uint32_t busy0 = micros();
    g_ctrlWakes++;
    uint32_t now = millis();
  // 单独用 1kHz 抓电压最低点：欠压跌落可能只有一两毫秒，控制环的 20Hz 会整个漏掉。
  // 一次转换约 100us，占约 10% CPU，控制环是 20Hz 有的是余量
  if (now - lastVmin >= 1) {
    lastVmin = now;
    static float s[3] = {99, 99, 99};
    static uint8_t si = 0;
    s[si++ % 3] = vbat();  // 不做低通，低通会把要抓的尖峰抹平
    // 取三连样本里的最大值当候选：一个孤立的 ADC 毛刺就会被旁边两个正常样本挡掉。
    // 纯 min-hold 撑不住——1kHz 跑两分钟是 12 万个样本，一个坏点就永久污染最小值
    float cand = max(max(s[0], s[1]), s[2]);
    if (cand > 1 && cand < 90) {
      if (cand < g_vMin) g_vMin = cand;
      if (cand < g_vMinRtc) g_vMinRtc = cand;
      // 一次塌陷只记一条：要回到阈值以上才重新武装，否则一次跌落能把整个环刷满
      static bool sagArmed = true;
      if (cand >= SAG_TH) {
        sagArmed = true;
      } else if (sagArmed) {
        sagArmed = false;
        uint32_t i = g_sagCnt % SAG_N;
        g_sag[i].ms = now; g_sag[i].v = cand;
        g_sag[i].pl = (int16_t)g_wl.pwm; g_sag[i].pr = (int16_t)g_wr.pwm;
        g_sagCnt++;
      }
    }
  }
  if (now - lastCtrl >= CTRL_PERIOD_MS) {
    wifitestCtrlMark(now - lastCtrl);
    float dt = (now - lastCtrl) / 1000.0f;
    lastCtrl = now;
    float vb = vbat();
    g_vFilt = g_vFilt < 1 ? vb : g_vFilt + 0.1f * (vb - g_vFilt);  // 时间常数约 0.5s
    // 读数低于 5V 认为压根没接电池分压(只插 USB 调试)，此时不做保护
    static float lowT = 0;
    if (g_vLow > 1 && g_vFilt > 5.0f && g_vFilt < g_vLow) lowT += dt; else lowT = 0;
    if (lowT > 5.0f && !g_lowBat) {  // 连续 5 秒才锁：躲开电机启动和急转时的跌落（09-24 用户定）
      g_lowBat = true;
      stopAll();
      g_ledState = LED_LOWBAT;
      g_ctrlNotify |= 1;
    }
    if (g_lowBat && g_vFilt > g_vLow + 0.5f) {  // 0.5V 回差，防止在阈值上反复跳
      g_lowBat = false;
      g_ledState = LED_OK;
      g_ctrlNotify |= 2;
    }
    controlWheel(g_wl, dt);
    controlWheel(g_wr, dt);
    odomUpdate();  // 开环手动给 PWM 时也在积分，dS 与 target 无关
    if ((g_wl.target || g_wr.target) && g_holdCmd) g_lastCmd = now;

    if (g_wl.stallT > STALL_SEC || g_wr.stallT > STALL_SEC) {
      stopAll();
      g_ledState = LED_STALL;
      g_ctrlNotify |= 4;
    }
  }
    g_ctrlBusyUs += micros() - busy0;
  }
}

// 断流探针：WiFi 收包空窗 >300ms 就记一次，恢复时打一行。日常只比较两个整数。
// loop 圈数和控制任务唤醒数是关键——它们分辨"CPU 被卡死"还是"CPU 活着但射频哑了"。
static void gapProbe() {
  static uint32_t loops = 0, inGap = 0, t0 = 0, l0 = 0, w0 = 0, r0 = 0, x0 = 0, s0 = 0;
  loops++;
  if (!ncm_bridge_running()) return;
  uint32_t now = millis(), since = now - ncm_last_w2u_ms;
  if (!inGap) {
    if (since > 300) {
      inGap = 1; t0 = ncm_last_w2u_ms; l0 = loops; w0 = g_ctrlWakes;
      r0 = ncm_tx_retry; x0 = ncm_rx_frames; s0 = ncm_drop_wifi;
    }
  } else if (since < 100) {
    inGap = 0;
    uint8_t ch = 0; wifi_second_chan_t sc;
    MARK(101);
    esp_wifi_get_channel(&ch, &sc);
    MARK(102);
    Serial1.printf("[GAP] 空窗%lums 起t=%lu 期间 loop%lu圈 控制唤醒%lu次(应约%lu) "
                   "USB进%lu帧 发送重试%lu 发送失败%lu 末错0x%lx 现ch=%d rssi=%d 状态=%d\n",
                   (unsigned long)(now - t0), (unsigned long)t0,
                   (unsigned long)(loops - l0), (unsigned long)(g_ctrlWakes - w0),
                   (unsigned long)((now - t0) / 50),
                   (unsigned long)(ncm_rx_frames - x0), (unsigned long)(ncm_tx_retry - r0),
                   (unsigned long)(ncm_drop_wifi - s0), (unsigned long)ncm_tx_last_err,
                   (int)ch, (int)WiFi.RSSI(), (int)WiFi.status());
    MARK(103);
  }
}

void loop() {
  g_beat++;                 // 看门狗任务据此判活，停跳 25 秒就重启
  STAGE(1);
  MARK(11); gapProbe();
  STAGE(2);
  MARK(21); wifitestTick();
  STAGE(3);
  MARK(31); ncmAutoTick();
  STAGE(4);
  {
    // ⚠️ 必须节流：loop 每秒跑 2~3 万圈，而这个函数每次都要拿 NCM 的互斥锁，
    // 裸调等于每秒上万次和 usbd 任务抢锁。门限是 500ms，10ms 的检查粒度绰绰有余。
    static uint32_t lastRec = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastRec >= 10) {
      lastRec = nowMs;
      // 主机还没配置好接口时没什么可恢复，跳过。（这条不是冷启动卡死的原因：实测加了之后
      // 断开时刻依旧是 46.07s，因为枚举在 t=20.3 就完成了、mounted 早就为真。）
      // 🔴 g_sub 逐句标记：卡点只到"stage 4"这一级，而这块里有四个语句，分不出是哪句。
      g_sub = 41;
      uint32_t us = ncm_usb_state();
      bool mounted = (us & 2) != 0;
      // 每次变化都记：09-23 冷启动主机 1.03 s 就在，ESP32 却到 20 s 才被枚举，中间经历了什么要看得见
      static uint32_t usPrev = 0;
      if ((us ^ usPrev) & 1) bootEv((us & 1) ? "USB连上" : "USB断开");
      if ((us ^ usPrev) & 2) bootEv(mounted ? "USB枚举" : "USB失去枚举");
      usPrev = us;
      if (mounted && !g_msUsb1) g_msUsb1 = nowMs;
      g_sub = 42;
      int heal = mounted ? ncm_tx_recover_if_lost(nowMs) : 0;
      g_sub = 43;
      if (heal == NCM_HEAL_REARM) {   // 第一级：续传没发出去的部分，先留寄存器现场
        char rg[220]; ncmEpRegs(rg, sizeof rg);
        bool ok = ncm_in_rearm();
        Serial1.printf("[NCMFIX] t=%lu 续传%s | 续传前 %s\n", (unsigned long)nowMs, ok ? "成功" : "失败", rg);
        Serial.printf("[NCMFIX] t=%lu 续传%s | 续传前 %s\n", (unsigned long)nowMs, ok ? "成功" : "失败", rg);
        bootEv(ok ? "网桥卡死续传" : "网桥卡死续传失败");
      }
      bool running = (heal == NCM_HEAL_RECYCLE) && ncm_bridge_running();
      g_sub = 44;
      if (running) {
        ncmEpRegs(g_epReg1, sizeof g_epReg1);
        Serial1.printf("[NCMREG] t=%lu %s\n", (unsigned long)nowMs, g_epReg1);
        Serial.printf("[NCMREG] t=%lu %s\n", (unsigned long)nowMs, g_epReg1);
        Serial1.printf("[NCMFIX] t=%lu 踢端点没救回来，只能让主机重新枚举 | 现场: %s\n",
                       (unsigned long)nowMs, epStateStr(ncm_fail_ep_state));
        g_sub = 45;
        bootEv("网桥卡死重新枚举");
        ncm_usb_recycle();
      }
      g_sub = 46;
    }
  }
  STAGE(5);
  {
    // 每 30 秒打一行关键计数。命令查询进不来(串口有来源白名单)，而这条日志会被
    // board-app 记进 nodes/*_diffcar_control.txt —— 目前唯一拿得到 ESP32 一手数据的路。
    static uint32_t lastSt = 0;
    if (millis() - lastSt >= 30000) {
      lastSt = millis();
      MARK(51);
      Serial1.printf("[NCMST] t=%lu 帧 收%lu/发%lu | NTB 提交%lu 完成%lu 失败%lu | 补齐%lu 补不下%lu | 踢端点%lu 重踢%lu | 丢发%lu 卡%lu | 最长%lums | 桥=%d STA占IP=%d\n",
                     (unsigned long)millis(),
                     (unsigned long)ncm_rx_frames, (unsigned long)ncm_tx_frames,
                     (unsigned long)ncm_xfer_submit, (unsigned long)ncm_xfer_done,
                     (unsigned long)ncm_xfer_fail,
                     (unsigned long)ncm_ntb_pad, (unsigned long)ncm_ntb_nopad,
                     (unsigned long)ncm_ep_unstick, (unsigned long)ncm_tx_restart,
                     (unsigned long)ncm_drop_tx, (unsigned long)ncm_stalls,
                     (unsigned long)ncm_xmit_max_ms,
                     (int)ncm_bridge_running(), (int)ncm_sta_has_ip());
      MARK(52);
      uint32_t _us = ncm_usb_state();
      // ⚠️ WiFi 侧这几个才是 USB->WiFi 方向真正的丢弃计数。上面那个"丢发"是 USB 侧的，
      //    两者完全不是一回事 —— 2026-09-17 我盯着 USB 侧的 0 以为一切正常，白查一轮。
      Serial1.printf("[NCMST] t=%lu USB 连接=%d 已枚举=%d 挂起=%d 掉线自愈%lu | WiFi 发失败%lu 队列满%lu 重试%lu 末错0x%lx 距上次收帧%lums rssi=%d ssid=%s\n",
                     (unsigned long)millis(), (int)!!(_us & 1), (int)!!(_us & 2),
                     (int)!!(_us & 4), (unsigned long)ncm_usb_reconn,
                     (unsigned long)ncm_drop_wifi, (unsigned long)ncm_drop_u2wq,
                     (unsigned long)ncm_tx_retry, (unsigned long)ncm_tx_last_err,
                     (unsigned long)(millis() - ncm_last_w2u_ms), (int)WiFi.RSSI(),
                     WiFi.SSID().c_str());   // 报出来才能知道候选列表到底选中了谁
      Serial1.printf("[NCMST] t=%lu 驱动补丁 保留位15=%lu 放不下=%lu 续传%lu/%lu 完成未空=%lu IN超时=%lu\n",
                     (unsigned long)millis(), (unsigned long)dcdx_unknown15, (unsigned long)dcdx_nospace,
                     (unsigned long)dcdx_rearm_ok, (unsigned long)dcdx_rearm_fail,
                     (unsigned long)dcdx_cmpl_notempty, (unsigned long)dcdx_in_timeout);
      // 🔴 内存是头号嫌疑：断死瞬间 esp_wifi_internal_tx 的错误码是 0x101 = ESP_ERR_NO_MEM。
      //    WiFi 发送要的是 internal(可 DMA)那块，和总堆量不是一回事，必须分开看。
      MARK(53);
      Serial1.printf("[NCMST] t=%lu 堆 空闲%lu 历史最低%lu | internal 空闲%lu 最大块%lu\n",
                     (unsigned long)millis(),
                     (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
                     (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    }
  }
  STAGE(6);
  {
    // 🔴 USB 设备整个从总线上掉了的自愈。2026-09-17 实测：大流量下 ESP32 的 USB device
    // 会消失，Looper 侧网卡直接变成 "No such device"，而 ESP32 自己毫无察觉 —— 灯是绿的、
    // NCM 各项计数全正常。板子重启都救不回来，只有这里重新连接一次。
    // 判据只能用 tud_mounted()：NCM 层那些计数在设备掉线后依然"正常"，看不出任何异常。
    // 线真拔了时重连没用：09-28 拔线 90 分钟重连了 165 次。间隔从 30 s 起翻倍、封顶 5 分钟，枚举上了就归零
    static uint32_t unmountAt = 0, lastReconn = 0, reconnGap = 30000, episodeN = 0;
    uint32_t nowMs = millis();
    MARK(61);
    if ((ncm_usb_state() & 2) || nowMs < NCM_BOOT_GUARD_MS) { unmountAt = 0; reconnGap = 30000; episodeN = 0; }
    else if (!unmountAt) unmountAt = nowMs;
    else if (nowMs - unmountAt > 5000 && (!lastReconn || nowMs - lastReconn > reconnGap)) {
      if (episodeN++) reconnGap = reconnGap >= 150000 ? 300000 : reconnGap * 2;
      lastReconn = nowMs;
      ncm_usb_reconn++;
      bootEv("USB掉线自愈重连");
      MARK(62);
      Serial1.printf("[NCMFIX] t=%lu USB 已掉线 %lums，重新连接(第%lu次)\n",
                     (unsigned long)nowMs, (unsigned long)(nowMs - unmountAt),
                     (unsigned long)ncm_usb_reconn);
      MARK(63);
      ncm_usb_recycle();
      MARK(64);
      unmountAt = 0;
    }
  }
  STAGE(7);
  {
    // 🔴 最后兜底：网桥彻底断死时自己重启。2026-09-18 实测断死后 Looper 侧网卡会消失，
    // 连板子重启都救不回（网卡不再出现），只有 ESP32 重新上电才行 —— 而 esp_restart()
    // 等效于重新上电，1 秒就能回来，比人工断电快两个数量级。
    // 判据要【同时】满足两条，缺一不可：
    //   ① 桥接开着且 USB 已枚举（否则开机/升级期间会误触发）
    //   ② 完成计数和收帧时刻【双双】长时间不动 —— 只看一个会在安静网络里误判
    static uint32_t deadAt = 0, lastDone = 0;
    uint32_t nowMs = millis();
    MARK(71);
    if (nowMs > NCM_BOOT_GUARD_MS && ncm_bridge_running() && (ncm_usb_state() & 2)) {
      bool moving = (ncm_xfer_done != lastDone) || (nowMs - ncm_last_w2u_ms < NCM_DEAD_MS);
      lastDone = ncm_xfer_done;
      if (moving) deadAt = 0;
      else if (!deadAt) deadAt = nowMs;
      else if (nowMs - deadAt > NCM_DEAD_MS) {
        MARK(72);
        Serial1.printf("[NCMFIX] t=%lu 网桥死了 %lums(完成计数和收帧都不动)，重启 ESP32\n",
                       (unsigned long)nowMs, (unsigned long)(nowMs - deadAt));
        Serial1.flush();
        delay(50);
        esp_restart();
      }
    } else deadAt = 0;
  }
  STAGE(8);
  MARK(81); ncmStuckWatchdog();
  STAGE(9);
  MARK(91);
  {
    // 死锁自愈是罕见事件，一发生就必须留痕：今天那次三道检测全看不见、零日志，
    // 只能靠读代码推断根因。限流是防止真坏掉时把 ttyS3 刷爆、连带卡住控制环。
    static uint32_t lastUnstick = 0;
    if (ncm_ep_unstick != lastUnstick) {
      lastUnstick = ncm_ep_unstick;
      MARK(92);
      Serial1.printf("[NCMFIX] t=%lu 踢端点第%lu次(主机无感) | 现场: %s\n",
                     (unsigned long)millis(), (unsigned long)ncm_ep_unstick,
                     epStateStr(ncm_fail_ep_state));
    }
    static uint32_t lastSeen = 0, lastLog = 0;
    if (ncm_tx_restart != lastSeen && millis() - lastLog > 5000) {
      lastLog = millis();
      Serial1.printf("[NCMFIX] t=%lu 重踢%lu次 xfer失败%lu 帧数满%lu 踢端点%lu次 | 现场: %s\n",
                     (unsigned long)millis(), (unsigned long)ncm_tx_restart,
                     (unsigned long)ncm_xfer_fail, (unsigned long)ncm_nx_count,
                     (unsigned long)ncm_ep_unstick, epStateStr(ncm_fail_ep_state));
      lastSeen = ncm_tx_restart;
    }
  }
  STAGE(10);
  {
    String c;
    if (wifitestTakeCmd(c)) {
      String reply;
      io.cap = &reply;
      handleLine(c, true);
      io.cap = nullptr;
      if (reply.isEmpty()) reply = "ok\n";
      wifitestReplyCmd(reply.c_str());
    }
  }
  static String buf;
  STAGE(111);
  pollUart();
  STAGE(112);
  g_stageAux = 0;
  while (io.available()) {
    g_stageAux++;            // 转了多少圈还没出去 —— 判断是不是在这里活锁
    STAGE(113);
    char c = io.read();
    if (c == '\n' || c == '\r') {
      STAGE(114);
      io.println();
      if (!buf.isEmpty()) { STAGE(115); handleLine(buf); buf = ""; }
    } else if ((c == 0x08 || c == 0x7f) && buf.length()) {
      buf.remove(buf.length() - 1);
      STAGE(116);
      io.print(F("\b \b"));
    } else if (c >= 0x20 && c < 0x7f && buf.length() < 32) {
      buf += c;
      STAGE(117);
      io.write(c);  // 回显，pio monitor 默认不回显本地输入
    }
    STAGE(118);
  }
  STAGE(119);

  static uint32_t lastTelem = 0, lastLed = 0;
  STAGE(12);
  uint32_t now = millis();

  if (g_ctrlNotify) {
    uint8_t n = g_ctrlNotify;
    g_ctrlNotify = 0;
    if (n & 1) io.printf("低压保护触发 %.2fV < %.2fV，已断驱动，请充电\n", g_vFilt, g_vLow);
    if (n & 2) io.printf("电压已回升到 %.2fV，低压闭锁解除\n", g_vFilt);
    if (n & 4) io.println(F("堵转保护触发，已断驱动 —— 轮子卡住了还是编码器断线了？"));
  }


  if (g_telem && (g_closed || g_wl.pwm || g_wr.pwm) && now - lastTelem >= 200) {
    lastTelem = now;
    io.toUart = g_telemUart;
    io.printf("左 目标=%.3f 实测=%.3f PWM=%d | 右 目标=%.3f 实测=%.3f PWM=%d\n",
              g_wl.target, g_wl.vMeas, g_wl.pwm, g_wr.target, g_wr.vMeas, g_wr.pwm);
    io.toUart = false;
  }

  if (g_guard && (g_closed || g_wl.pwm || g_wr.pwm) && now - g_lastCmd > CMD_TIMEOUT_MS) {
    // 红慢闪只在"正在动的时候上位机不说话了"才有意义。上位机主动叫停(c 0 / u 0 0 会让
    // g_closed 留着顶住零速)之后失联，车本来就停着，报紧急停车是误报 —— 那种情况交给
    // 5s 静默检测点橙灯。必须在 stopAll 之前取，它会把 target 和 pwm 都清掉。
    const bool was_moving = g_wl.target || g_wr.target || g_wl.pwm || g_wr.pwm;
    stopAll();
    if (was_moving) {
      g_ledState = LED_LINKLOST;
      io.println(F("失联保护触发，已停车"));
    }
  }

  // 链路指示。压过 LED_LINKLOST（"上位机没了"比"没收到运动指令"信息更具体），
  // 但让堵转和低压压过它们两个 —— 那两个是会烧东西的
  // ⚠️ LED_CAL 会【屏蔽掉下面全部链路指示】，所以标定/自检结束时必须把它放回 LED_OK。
  //    漏一处就永久卡黄灯，绿/橙/青从此再不出现 —— 2026-09-17 就这么误判过一轮。
  if (g_ledState != LED_CAL && g_ledState != LED_STALL && g_ledState != LED_LOWBAT) {
    if (now - g_lastUart > UART_SILENT_MS) {
      if (g_ledState != LED_UPSTREAM_SILENT) { g_silentCnt++; g_silentAt = now; }
      g_ledState = LED_UPSTREAM_SILENT;
    } else if (!g_pcOk && now - g_pcStamp < PC_STALE_MS) {
      if (g_ledState != LED_PC_UNREACHABLE) { g_pcBadCnt++; g_pcBadAt = now; }
      g_ledState = LED_PC_UNREACHABLE;
    }
    else if (g_ledState == LED_UPSTREAM_SILENT || g_ledState == LED_PC_UNREACHABLE)
      g_ledState = LED_OK;                 // 恢复了，别粘住
  }

  if (now - lastLed >= 50) {
    lastLed = now;
    ledUpdate();
  }
}
