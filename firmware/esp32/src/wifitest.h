// WiFi 吞吐 + 实时性影响测试（2026-09-11，评估去掉 USB 网卡改用 ESP32 自带 WiFi）
// 命令 'N' 看状态，PC 端 nc <ip> 9000 拉数据测下行，nc <ip> 9001 灌数据测上行。
// 关掉整块功能：把 WIFITEST_ON 改成 0。
#pragma once
#define WIFITEST_ON 1

#if WIFITEST_ON
#include <WiFi.h>
#include "wifi_scan_policy.h"
#include "wifi_list.h"
#include "wifi_mode_policy.h"
#include <esp_wifi.h>
#include <WiFiUdp.h>
#include "usbncm.h"
extern volatile uint64_t g_ctrlBusyUs;
extern volatile uint32_t ncm_usb_reconn;
uint32_t wt_boot_cnt(void);
const char *wt_reset_name(void);
float wt_vmin_last(void);
const char *wt_stuck_note(void);
extern volatile uint32_t g_ctrlWakes, g_ctrlSince;

#ifndef WIFITEST_SSID
#define WIFITEST_SSID ""
#define WIFITEST_PASS ""
#endif

// 候选 AP 列表。真实密码在 wifi_creds.h（gitignore），模板见 wifi_creds.example.h。
// 没有那个文件也能编译：候选表为空，开机直接开车的热点，用户在网页里加网络（wm_effective_mode）。
#if defined(__has_include)
#  if __has_include("wifi_creds.h")
#    include "wifi_creds.h"
#  endif
#endif
#ifndef WIFI_AP_LIST
#define WIFI_AP_LIST { {WIFITEST_SSID, WIFITEST_PASS} }
#endif
static const struct { const char *ssid, *pass; } g_wtBuiltin[] = WIFI_AP_LIST;
// 运行时的候选表：闪存里有就用闪存的（网页可改），没有就用编译进来的 g_wtBuiltin。
static WlList g_wtList;
static bool g_wtListFromNvs = false;
static WlList g_wtStage;                 // Nwb..Nwc 之间暂存，提交时整表替换
static bool g_wtScanning = false;
static uint8_t g_wtEmptyScans = 0;
static volatile uint8_t g_wtLastReason = 0;   // WiFi 事件任务写、loop 读 ⇒ volatile
static int8_t g_wtLastPick = -1;        // 上次连上的是列表里第几个
static uint32_t g_wtHoldUntil = 0;       // Ndh 演练：这之前状态机不重连，模拟长时间断网
static uint8_t g_wtApWhy = 0;            // 热点为什么开着：0 没开 / 1 Nap 限时测试 / 2 用户选了热点模式 / 3 兜底（连不上任何已存网络）
static uint32_t g_wtApUntil = 0;         // 限时热点的截止时刻，0 = 不限时
static uint8_t g_wtPrefMode = 0;         // 用户选的模式（闪存 netmode/m）：0 连路由器 1 车的热点
static uint32_t g_wtModeAt = 0;          // 切模式推迟到这个时刻：先让网页收到回复，射频一换它就收不到了
static uint8_t g_wtModeWant = 0;
static WlEntry g_wtAp = {};              // 网页改过的热点名和密码（闪存 netmode/as、ap），空 = 出厂默认
static uint32_t g_wtApRebuildAt = 0;     // 热点开着时改了名字/密码：推迟重建，理由同 g_wtModeAt
// 从车的热点切到路由器是「待确认」的：WT_CONFIRM_MS 内没人从新网络打开网页（后端发 Nmc）就自己回热点，
// 防的是用户找不到车的新地址。存闪存 netmode/pe，断电重启后重新计时。
#define WT_CONFIRM_MS 300000
static uint32_t g_wtConfirmBy = 0;       // 0 = 不在待确认
// 切路由器前试连（Nwt）：0 没试 1 扫描 2 连接 3 出结果
static uint8_t g_trSt = 0;
static int8_t g_trIdx = -1, g_trOk = -1, g_trRssi = 0;
static uint8_t g_trRes[WL_MAX];
static bool g_trSeen[WL_MAX];
static uint32_t g_trStart = 0, g_trBy = 0;
static bool g_trRetried = false;
static volatile uint32_t g_trIp = 0, g_trGw = 0;   // 事件任务写
static uint32_t g_wtFbScanAt = 0, g_wtFbScanStart = 0;   // 兜底热点里扫已存网络的节拍
static bool g_wtFbScanning = false;
static bool g_wtPcForced = false;        // 这次是被「开机 10 秒内断电连续 3 次」强制切成热点的
#define WT_FALLBACK_MS 90000             // 连不上多久开兜底热点：排在 60 s 射频复位之后，驱动卡死要先救驱动
#define WT_FB_SCAN_MS 30000
#ifndef WT_DEFAULT_MODE
#define WT_DEFAULT_MODE 0                // 闪存里没选过时的模式。开发车保持连路由器；产品固件编 -DWT_DEFAULT_MODE=1
#endif
static void wifitestApStart(uint8_t why, uint32_t ms, Print &o);
static void wtNetModeTick();
static void wtModePcClear();
static void wtModeLoad();
static const char *const kWtApWhy[] = {"没开", "限时测试", "热点模式", "兜底"};
#define WT_AP_PASS "looper1234"          // 热点密码，至少 8 位（WPA2 下限）
#define WT_AP_CHANNEL 6
static void wifitestApStop(Print &o);
static bool g_wtRetriedLast = false;    // 本次掉线是否已经直连重试过它
static uint32_t g_wtScanStart = 0, g_wtNextTry = 0, g_wtDownSince = 0, g_wtLastBlind = 0;

static WiFiServer g_wtDown(9000), g_wtUp(9001);
static WiFiUDP g_wtCmd;                  // 9004：网桥关着时的远程命令口
static bool g_wtCmdUp = false;
static String g_wtCmdLine;               // 收到的整行，等 loop 取走
static IPAddress g_wtCmdFrom;
static uint16_t g_wtCmdPort;
static WiFiClient g_wtDownCli, g_wtUpCli;
static uint8_t g_wtBuf[1460];               // 一个以太网 MSS，避免 lwIP 再分片
static uint32_t g_wtBytes = 0, g_wtStart = 0;
static uint32_t g_wtLoopMaxUs = 0, g_wtLoopOver1ms = 0, g_wtLoopOver10ms = 0;
static uint32_t g_wtLoopCount = 0, g_wtLoopSince = 0;   // loop 转速：判断裸调 API 代价的分母
static uint32_t g_wtCtrlMaxMs = 0, g_wtHeapMin = 0xFFFFFFFF, g_wtConn = 0;
static WiFiUDP g_wtUdp;
static IPAddress g_wtUdpDst;
static uint32_t g_wtUdpEnd = 0, g_wtUdpPkts = 0;

#include "esp_core_dump.h"
#include <Preferences.h>
#include "esp_ota_ops.h"

static bool g_wtInited = false, g_wtSrvUp = false;

// 只置位。在 setup 里直接 WiFi.begin() 实测拿到 status=255(驱动没起来)，
// 而且监听口要等拿到 IP 之后 begin 才有效 —— 两件事都挪进 tick 分阶段做。
// 闪存里没有或读坏了就退回编译进来的表。两边都空时表为空，只能开热点（wm_effective_mode）。
static void wtListLoad() {
  Preferences p;
  WlList l = {};
  if (p.begin("wifilist", true)) {
    uint8_t n = p.getUChar("n", 0);
    for (uint8_t i = 0; i < n && i < WL_MAX; i++) {
      char k[4] = {'s', (char)('0' + i), 0};
      String ss = p.getString(k, ""); k[0] = 'p';
      String pw = p.getString(k, "");
      if (!ss.length() || ss.length() > WL_SSID_MAX || pw.length() > WL_PASS_MAX) { l.n = 0; break; }
      strcpy(l.e[l.n].ssid, ss.c_str()); strcpy(l.e[l.n].pass, pw.c_str()); l.n++;
    }
    p.end();
  }
  g_wtListFromNvs = l.n > 0;
  if (!g_wtListFromNvs)
    for (size_t i = 0; i < sizeof(g_wtBuiltin) / sizeof(g_wtBuiltin[0]) && l.n < WL_MAX; i++) {
      if (!g_wtBuiltin[i].ssid[0]) continue;   // 没有 wifi_creds.h 时的空占位
      strncpy(l.e[l.n].ssid, g_wtBuiltin[i].ssid, WL_SSID_MAX);
      strncpy(l.e[l.n].pass, g_wtBuiltin[i].pass, WL_PASS_MAX);
      l.n++;
    }
  g_wtList = l;
}

static void wifitestBegin() {
  for (uint16_t i = 0; i < sizeof(g_wtBuf); i++) g_wtBuf[i] = (uint8_t)i;
  wtListLoad();
  wtModeLoad();
}

// 放 loop 最前面，量的是「上一整圈用了多久」—— 控制环 20Hz、电压采样 1kHz，
// 所以超过 1ms 就已经开始漏采样了，这是比平均值有用的判据。
// 断流探针：射频停 3.4s 的那一刻 ESP32 收到了什么事件。只在事件发生时输出，日常零开销。
// 冷启动里程碑和稳定性计数。三个测试要的就是这些数，做成变量而不是日志行 ——
// 日志会被刷掉、而且 [WIFIEV] 只往 Serial1 走(给板子)，插 COM 线时根本看不到。
static uint32_t g_msAssoc1 = 0;      // 第一次关联成功的时刻
static uint32_t g_msBridge1 = 0;     // 网桥第一次开起来（射频关联 != 能用）
static uint32_t g_cntAssoc = 0, g_cntDrop = 0;   // 关联/掉线累计次数
static uint32_t g_downMsTotal = 0;   // 累计掉线时长
static uint32_t g_dropAt = 0;        // 本次掉线开始时刻
static uint32_t g_apMsTotal = 0, g_apAt = 0;   // 热点模式累计时长 / 本次开热点时刻；不算掉线
static uint32_t g_msUsb1 = 0;        // 第一次被主机枚举（和板子 dmesg 里 cdc_ncm 注册是同一件事，拿来对齐两边时钟）
static uint8_t  g_bssid1[6];         // 第一次关联的 AP，定向重连要用
static volatile int16_t  g_wtScanDoneN = -1;   // 最近一次 SCAN_DONE 报的个数（事件任务写）
static volatile uint32_t g_wtScanDoneAt = 0;
static uint8_t g_wtLostScans = 0;              // 连续「结果丢了」的次数，封顶后按空扫描处理
static int8_t   g_ch1 = 0;
static uint8_t  g_wtCurBssid[6];                 // 当前/上次关联的 AP，掉线后定向直连它
static volatile int8_t g_wtCurCh = 0;
// 连接尝试失败的事件（8 = 我们自己 disconnect，不算）：让状态机不用等满超时。
static volatile bool g_wtAttemptFailed = false;
// 开机前 48 件事，满了就不再记（不滚动）：要的是开头那几十秒花在哪。
// WiFi 事件任务和主循环都会写，追加必须进临界区。
#define BOOTEV_N 48
static struct { uint32_t ms; const char *what; int32_t arg; } g_bootEv[BOOTEV_N];
static uint8_t g_bootEvN = 0;
static portMUX_TYPE g_bootEvMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_bootEvListAt = 0;   // 候选表最近一次被换掉时的事件数：之前的序号指的是旧表，不能按新表翻成名字
#define BOOTEV_NOARG INT32_MIN
static void bootEv(const char *what, int32_t arg = BOOTEV_NOARG) {
  uint32_t t = millis();
  taskENTER_CRITICAL(&g_bootEvMux);
  if (g_bootEvN < BOOTEV_N) g_bootEv[g_bootEvN++] = {t, what, arg};
  taskEXIT_CRITICAL(&g_bootEvMux);
}
// 两路都打：Serial1 进板子的日志，Serial 是原生 USB 口，插 COM 线时才看得到。
static void wtLog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void wtLog(const char *fmt, ...) {
  char b[256];                              // 中文一字 3 字节，最长那行约 190
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  Serial1.print(b); Serial.print(b);
}

static void wtOnWifiEvent(arduino_event_id_t ev, arduino_event_info_t info) {
  uint32_t t = millis();
  switch (ev) {
    case ARDUINO_EVENT_WIFI_AP_START: {
      // 驱动每次（重）起热点都会把 ESP32 自带的 DHCP 服务打开；地址归板子的 dnsmasq 发，两个会抢答
      esp_netif_t *apif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
      if (apif) esp_netif_dhcps_stop(apif);
      break;
    }
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
      g_wtScanDoneN = (int16_t)info.wifi_scan_done.number;
      g_wtScanDoneAt = t ? t : 1;
      bootEv("扫描完成", (int32_t)info.wifi_scan_done.number);
      wtLog("[WIFIEV] t=%lu 扫描完成 找到%d个\n", (unsigned long)t, (int)info.wifi_scan_done.number); break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      g_wtLastReason = info.wifi_sta_disconnected.reason;
      if (g_wtLastReason != 8) g_wtAttemptFailed = true;
      ncm_hb_reason = g_wtLastReason;
      if (!g_dropAt && !g_wtApWhy) { g_dropAt = t; g_cntDrop++; }   // 开热点时主动断开的不算
      bootEv("断开", (int32_t)g_wtLastReason);
      wtLog("[WIFIEV] t=%lu 断开 reason=%d\n", (unsigned long)t, (int)g_wtLastReason); break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      // 重新关联之后必须重挂网桥的 WiFi 收包回调，IDF 在断开时会把它丢掉。不重挂的话
      // 网桥显示 on、射频显示已关联，却一个包都收不进来。
      ncm_bridge_rearm();
      g_cntAssoc++;
      ncm_hb_gen++;
      if (!g_msAssoc1) {
        g_msAssoc1 = t;
        memcpy(g_bssid1, info.wifi_sta_connected.bssid, 6);
        g_ch1 = (int8_t)info.wifi_sta_connected.channel;
      }
      memcpy(g_wtCurBssid, info.wifi_sta_connected.bssid, 6);
      g_wtCurCh = (int8_t)info.wifi_sta_connected.channel;
      bootEv("已关联", (int32_t)info.wifi_sta_connected.channel);
      if (g_dropAt) { g_downMsTotal += t - g_dropAt; g_dropAt = 0; }
      wtLog("[WIFIEV] t=%lu 已连上 ch=%d\n", (unsigned long)t, (int)info.wifi_sta_connected.channel); break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      if (g_trSt == 2) { g_trGw = info.got_ip.ip_info.gw.addr; g_trIp = info.got_ip.ip_info.ip.addr; }
      bootEv("拿到IP");
      Serial1.printf("[WIFIEV] t=%lu 拿到IP\n", (unsigned long)t); break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP: {
      // 修复的基石就在这一刻：IP 没了，但射频层是否仍报"还关联着"。两者必须分叉。
      wifi_ap_record_t ap;
      bool link = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
      Serial1.printf("[WIFIEV] t=%lu 失去IP 射频层link=%d status=%d\n",
                     (unsigned long)t, (int)link, (int)WiFi.status());
      break;
    }
    // 不知道是什么事件才最该打——我们要找的就是没料到的那个
    default: Serial1.printf("[WIFIEV] t=%lu 事件号=%d\n", (unsigned long)t, (int)ev); break;
  }
}

// 🔴 判「掉线」只能看射频层有没有关联，【不能】看 WiFi.status()：网桥是靠刻意清掉
// ESP32 的 STA IP 来工作的（见 ncm_glue.c），于是 status 长期不是 WL_CONNECTED，而射频
// 其实好好连着。用 status 当判据会让重连逻辑不停把正常连接拆掉重扫 —— 实测每 10~14 秒
// 一次 reason=8（reason 8 = STA 自己离开，也就是我们自己拆的）。
// 这个文件开头 LOST_IP 事件处理里那句「IP 没了但射频层是否仍报还关联着，两者必须分叉」
// 说的就是这件事。
static bool wtLinkUp() {
  // ⚠️ 必须缓存：这个 API 要进 WiFi 驱动拿锁，而 loop 每秒跑上万圈。main.cpp 的
  // ncmAutoTick 早就为同一个调用加过 500ms 缓存，注释写明「裸调实测把经网桥的 ping
  // 打到 12~20% 丢包」。判链路断没断是秒级事件，500ms 粒度绰绰有余。
  static uint32_t lastChk = 0;
  static bool cache = false;
  uint32_t now = millis();
  if (now - lastChk > 500 || !lastChk) {
    lastChk = now ? now : 1;
    wifi_ap_record_t ap;
    MARK(227);
    cache = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
    MARK(228);
  }
  return cache;
}

// 按列表优先级挑一个【扫得到】的 AP 连上。只在射频没关联时调用。
//
// 🔴 必须异步扫描：同步 scanNetworks 会阻塞 loop 约 2 秒，而这个 loop 还在跑 20Hz 电机
//    闭环和 1kHz 电压采样 —— 卡 2 秒等于让电机失控。
// 🔴 连上之后绝不扫描：扫描把射频轮流切到十几个信道，网桥会整段断流。
// 🔑 三层兜底，防止改坏了就再也够不着板子（板上唯一的网络出口就是它）：
//    扫不到列表里任何一个 -> 盲连最后一项；扫描本身失败 -> 同样盲连；
//    掉线超过 3 分钟 -> 无条件盲连最后一项。所以最后一项必须是最可靠的那个 AP。
// 🔴 盲连必须【轮转整张表】。原来两条盲连路径都写死最后一项，而扫描一旦连续失败
// （库 6s 超时会被误记成"扫描空"），g_wtEmptyScans 在掉线期间又永不清零，于是三条
// 能发起连接的路径里两条固定最后一项、第三条一轮只用一次 —— 列表第一项(手机热点)
// 就再也试不到了。在室外(兜底 AP 根本不存在)等于永久失联。
// 发起一次连接，等 waitMs；期间来了失败事件就提前结束等待。给了 bssid 就是定向连接：
// 只给 SSID 时驱动按 ALL_CHANNEL_SCAN 全信道再扫一遍才连（实测 6.2 s，定向 0.45 s）。
// 扫描方式在 begin 时拷进配置，用完立刻改回。
// ⚠️ 不看 begin() 的返回值：它最后返回的是上一次失败留下的 status（实测 201 之后恒为
// WL_NO_SSID_AVAIL），据此缩短等待会让扫描打断正在进行的连接。失败靠断开事件来报。
static void wtConnect(int i, uint32_t waitMs, int32_t ch = 0, const uint8_t *bssid = nullptr) {
  g_wtAttemptFailed = false;
  if (bssid) WiFi.setScanMethod(WIFI_FAST_SCAN);
  if (bssid) WiFi.begin(g_wtList.e[i].ssid, g_wtList.e[i].pass, ch, bssid);
  else       WiFi.begin(g_wtList.e[i].ssid, g_wtList.e[i].pass);
  if (bssid) WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  // begin() 会重开 STA 的 DHCP；桥开着时 ESP32 不能占 IP（它和板子共用一个 MAC），关联前就停掉。
  if (ncm_bridge_running()) ncm_clear_sta_ip();
  g_wtNextTry = millis() + waitMs;
}

// Arduino 的 scanNetworks() 没把 wifi_scan_config_t 清零：被动扫描时长(12/13 信道按被动扫)和
// home_chan_dwell_time 是栈上残值，实测每轮恒 5.76 s、偶尔 1.28 s。照抄它，只多一步清零；
// 继承只为够着库的 protected 静态成员，让 scanComplete()/SSID(i) 照常能用。
struct WtScan : WiFiGenericClass, WiFiScanClass {
  static int16_t start(uint32_t maxMs) {
    if (getStatusBits() & WIFI_SCANNING_BIT) return WIFI_SCAN_RUNNING;
    _scanTimeout = maxMs * 20; _scanAsync = true;
    WiFi.enableSTA(true);
    WiFi.scanDelete();
    wifi_scan_config_t c = {};
    c.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    c.scan_time.active.min = 100; c.scan_time.active.max = maxMs; c.scan_time.passive = maxMs;
    if (esp_wifi_scan_start(&c, false) != ESP_OK) return WIFI_SCAN_FAILED;
    _scanStarted = millis() ? millis() : 1;
    clearStatusBits(WIFI_SCAN_DONE_BIT);
    setStatusBits(WIFI_SCANNING_BIT);
    return WIFI_SCAN_RUNNING;
  }
};

static uint8_t g_wtBlindIdx = 0;
static int wtBlindNext() { return (int)(g_wtBlindIdx++ % g_wtList.n); }

static uint32_t g_wtRadioResetAt = 0;
static uint32_t g_wtRadioResetCnt = 0;

// 🔴 驱动整个卡死时的正解：重新初始化射频，而不是靠看门狗重启整片。
// 观测到的卡死形态（2026-09-22 复现多次）：同步 WiFi.scanNetworks() 返回 -1、
// status 恒为 1(WL_NO_SSID_AVAIL)、WiFi.begin() 完全不产生关联，且持续几分钟不自愈。
// 我们自己的扫描超时兜底(esp_wifi_scan_stop + scanDelete)对这种状态无效，因为坏的
// 不是状态机而是驱动。关掉再打开射频能复位驱动，代价只有一次 200ms 的 loop 阻塞
// （电机跑在独立的 ctrlTask 上，不受影响），比整片重启轻得多。
static void wtRadioReset() {
  g_wtRadioResetAt = millis();
  g_wtRadioResetCnt++;
  bootEv("射频复位");
  Serial1.printf("[WIFI] 射频卡死，复位驱动（第%lu次）\n", (unsigned long)g_wtRadioResetCnt);
  esp_wifi_scan_stop();
  WiFi.scanDelete();
  WiFi.disconnect(true, true);
  ncm_bridge_hold(1);
  WiFi.mode(WIFI_OFF);
  delay(200);
  WiFi.persistent(false);                 // 和 setup 里一致，否则驱动会抢先自动连上次那个
  WiFi.mode(WIFI_STA);
  ncm_bridge_hold(0);
  WiFi.setSleep(false);
  g_wtScanning = false;
  g_wtRetriedLast = false;
  g_wtEmptyScans = 0;
  g_wtNextTry = millis() + 1000;
}

static void wtPickAp() {
  uint32_t now = millis();
  // ⚠️ loop 每秒跑约 8000 圈，而下面的 scanComplete/begin 都要进 WiFi 栈抢锁 ——
  // 裸调就是每秒上万次锁竞争。这个函数只管重连，100ms 一次绰绰有余。
  static uint32_t last = 0;
  if (now - last < 100) return;
  last = now;
  if (!g_wtDownSince) g_wtDownSince = now;
  if ((int32_t)(g_wtHoldUntil - now) > 0) return;
  // 路由器模式下 90 s 连不上任何已存网络就开兜底热点：密码填错、搬了家，用户也总有一个入口
  if (wm_fallback_due(g_wtPrefMode, (int32_t)(now - g_wtDownSince), WT_FALLBACK_MS)) { wifitestApStart(3, 0, Serial1); return; }

  if (g_wtScanning) {
    MARK(221);
    int n = WiFi.scanComplete();
    int doneN = (g_wtScanDoneAt && (int32_t)(g_wtScanDoneAt - g_wtScanStart) >= 0) ? g_wtScanDoneN : -1;
    WScanVerdict v = wscan_verdict(n, now - g_wtScanStart, 9000, doneN);
    if (v == WSCAN_RUNNING) return;
    if (v == WSCAN_ABORTED) {
      // 驱动自己在重连时会抢掉扫描槽。这不说明周围没 AP，所以【不】计进三次空扫描额度，
      // 只是隔一会儿再来 —— 混在一起会让掉线后三轮就永久退到盲连。
      bootEv("扫描中止", n);
      wtLog("[WIFI] t=%lu 扫描被中止(rc=%d)，1.5s 后重来\n", (unsigned long)now, n);
      WiFi.scanDelete(); g_wtScanning = false; g_wtNextTry = now + 1500; return;
    }
    if (v == WSCAN_LOST && g_wtLostScans < 2) {
      g_wtLostScans++;
      bootEv("扫描结果丢失", doneN);
      Serial1.printf("[WIFI] 事件报扫到 %d 个、读出来 0 个，0.5s 后重扫(第%u次)\n", doneN, (unsigned)g_wtLostScans);
      WiFi.scanDelete(); g_wtScanning = false; g_wtNextTry = now + 500; return;
    }
    if (v == WSCAN_TIMEOUT) {
      bootEv("扫描超时", n);
      // 9000 必须大于库自己的超时(max_ms_per_chan*20 = 3000ms)，否则我们抢在驱动前面放弃、
      // 而库的 SCANNING 位没清。等的代价有上限，放弃的代价没有，所以往宽了给。
      MARK(222);
      esp_wifi_scan_stop();                          // scanDelete 不会停驱动层扫描，必须自己停
      // 停完再查一次：结果常常就在这一刻到位，直接丢掉等于白等了 9 秒。
      n = WiFi.scanComplete();
      if (n > 0) {
        Serial1.printf("[WIFI] 扫描超时但停止时拿到了 %d 个，照用\n", n);
      } else {
        WiFi.scanDelete();
        g_wtScanning = false; g_wtNextTry = now + 5000; return;
      }
    }
    g_wtScanning = false;
    int pick = -1, pickJ = -1;
    for (int i = 0; i < g_wtList.n && pick < 0; i++)
      for (int j = 0; j < n; j++)
        if (WiFi.SSID(j) == g_wtList.e[i].ssid && (pickJ < 0 || WiFi.RSSI(j) > WiFi.RSSI(pickJ))) { pick = i; pickJ = j; }
    // 🔴 扫描结果为空/失败时【不能】马上退兜底：驱动没就绪时扫描会返回 0 或 -2，
    // 而一旦连上兜底 AP 就不再扫描了，等于永久钉死在它上面——换热点不生效就是这么来的。
    if (n <= 0 && g_wtEmptyScans < 3) {
      g_wtEmptyScans++;                              // 钳位而不是 ++ 后比较：uint8 回绕会凭空多给重试机会
      bootEv("扫描空", n);
      Serial1.printf("[WIFI] 扫描没结果(rc=%d %s，第%u次)，重试\n", n,
                     n == 0 ? "空" : n == -1 ? "还在跑" : "失败", (unsigned)g_wtEmptyScans);
      WiFi.scanDelete(); g_wtNextTry = now + 3000; return;
    }
    if (n > 0) { g_wtEmptyScans = 0; g_wtLostScans = 0; }
    // 「真扫到」和「没扫到只好盲连」必须分开打，否则选中最后一项时看不出是哪种
    bool found = (pick >= 0);
    if (pick < 0) pick = wtBlindNext();              // 一个都没扫到 -> 盲连，【轮转】不是固定兜底项
    bootEv(found ? "选中" : "盲连", pick);
    wtLog("[WIFI] t=%lu 扫到%d个AP，选 %s%s\n", (unsigned long)now, n > 0 ? n : 0, g_wtList.e[pick].ssid,
          found ? "" : "(没扫到列表里任何一个，盲连兜底)");
    // 🔴 结果会被事件任务清掉（stop 补发的"0 个"晚到），BSSID() 此时返回空指针，必须判空。
    // getNetworkInfo 一次取整条记录，信道和 BSSID 才保证出自同一次扫描。
    uint8_t bssid[6]; int32_t ch = 0; bool directed = false;
    if (found) {
      String ss; uint8_t enc; int32_t rs; uint8_t *b = nullptr;
      if (WiFi.getNetworkInfo(pickJ, ss, enc, rs, b, ch) && b && ss == g_wtList.e[pick].ssid) {
        memcpy(bssid, b, 6); directed = true;
        bootEv("信号dBm", rs);
      }
    }
    WiFi.scanDelete();                               // BSSID 指向扫描结果，必须先拷出来再删
    MARK(223);
    if (found && !directed) bootEv("定向失败");
    wtConnect(pick, 20000, ch, directed ? bssid : nullptr);
    MARK(224);
    return;
  }

  if (g_wtAttemptFailed) { g_wtAttemptFailed = false; g_wtNextTry = now; }
  if ((int32_t)(now - g_wtNextTry) < 0) return;   // 有符号差值：直接比绝对值在 millis 回绕那 30 秒会让退避全失效
  // 🔑 掉线先定向直连上次那个 AP：几百毫秒的抖动（手机热点常用 reason=8 踢人）0.45 s 就回来。
  // 只等 2 s：AP 真没了的话驱动会退到全信道扫描，6 s 后才报 201，不如我们自己扫。
  // disconnect 先打断核心库的重连：开机后第一次掉线它无视 setAutoReconnect 也会重连一次。
  if (g_wtLastPick >= 0 && !g_wtRetriedLast) {
    g_wtRetriedLast = true;
    wtLog("[WIFI] t=%lu 先定向直连上次的 %s\n", (unsigned long)now, g_wtList.e[g_wtLastPick].ssid);
    bootEv("直连上次", g_wtLastPick);
    MARK(225);
    WiFi.disconnect(false, false);
    wtConnect(g_wtLastPick, 2000, g_wtCurCh, g_wtCurBssid);
    MARK(226);
    return;
  }
  // 掉线 60 秒还连不上，先复位一次射频。放在盲连【之前】：盲连解决的是"扫不到"，
  // 而驱动卡死时连 begin 都不起作用，盲连再多次也没用。两次复位之间隔 60 秒。
  if (now - g_wtDownSince > 60000 && (!g_wtRadioResetAt || now - g_wtRadioResetAt > 60000)) {
    wtRadioReset();
    return;
  }
  // 掉线超 3 分钟，每 3 分钟插一次「盲连兜底项」：防的是扫描本身坏掉的情况。
  // ⚠️ 必须和扫描【交替】而不是从此只盲连 —— 否则在室外(兜底 AP 根本不存在)就永远
  //    连不回手机热点了。这是第一版写出来的真 bug。
  if (now - g_wtDownSince > 180000 && now - g_wtLastBlind > 180000) {
    g_wtLastBlind = now;
    Serial1.printf("[WIFI] 掉线%lus，插一次盲连\n",
                   (unsigned long)((now - g_wtDownSince) / 1000));
    int bi = wtBlindNext();
    bootEv("插盲连", bi);
    wtConnect(bi, 30000);
    return;
  }
  WiFi.disconnect(false, false);
  // 🔴 别无条件 esp_wifi_scan_stop()：它补发的「找到0个」SCAN_DONE 会落在新扫描起来【之后】，
  // 把库的 _scanStarted 清掉，scanComplete() 于是返回 0 而不是 RUNNING —— 见 wifi_scan_policy.h。
  // 只在真有扫描挂着时才停，停完把结果删干净再起新的。
  if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) {
    MARK(229);
    esp_wifi_scan_stop();
    WiFi.scanDelete();
  }
  // 显式给 max_ms_per_chan=150 => 库超时 3000ms，是个我们算得出来的值（默认 300 => 6000ms，
  // 而全信道扫描在现场 21 个 AP 挤 3 信道的环境下可能要 1.5~4s，很容易撞上并被误记成"扫描空"）。
  MARK(230);
  int16_t src = WtScan::start(150);
  // 起不来就别自称在扫：否则下一轮 scanComplete() 的 0 会被当成"扫到 0 个"，
  // 三次之后就永远走盲连，再也不按优先级挑 AP。
  if (src != WIFI_SCAN_RUNNING) {
    bootEv("起扫描失败", src);
    wtLog("[WIFI] t=%lu 起扫描失败 rc=%d，2 秒后重来\n", (unsigned long)now, (int)src);
    g_wtNextTry = now + 2000;
    return;
  }
  g_wtScanning = true; g_wtScanStart = now;
  bootEv("扫描起");
  wtLog("[WIFI] t=%lu 开始扫描\n", (unsigned long)now);
}

static void wifitestTick() {
  static uint32_t prev = 0;
  uint32_t now = micros();
  g_wtLoopCount++;
  if (!g_wtLoopSince) g_wtLoopSince = millis();
  if (prev) {
    uint32_t d = now - prev;
    if (d > g_wtLoopMaxUs) g_wtLoopMaxUs = d;
    if (d > 1000) g_wtLoopOver1ms++;
    if (d > 10000) g_wtLoopOver10ms++;
  }
  prev = now;

  // 堆查询要拿堆锁，loop 每秒 8000 圈裸调是纯浪费 —— 这只是个最低水位统计，半秒一次够
  static uint32_t lastHeapChk = 0;
  if (millis() - lastHeapChk > 500) {
    lastHeapChk = millis();
    MARK(201);
    uint32_t h = ESP.getFreeHeap();
    if (h < g_wtHeapMin) g_wtHeapMin = h;
  }

  if (!g_wtInited && millis() > 5000) {   // 5s 不是 3s：驱动刚起来就扫描会直接返回失败
    g_wtInited = true;
    bootEv("WiFi启动");
    MARK(202);
    WiFi.onEvent(wtOnWifiEvent);
    // 🔴 默认 persistent=true：驱动把上次连的 AP 存进 NVS，开机会抢在我们挑选之前自动连上它。
    // 那样候选列表永远不生效 —— 表现就是「换了热点还是连回原来那个」。必须先关存储再清掉旧配置。
    MARK(203);
    WiFi.persistent(false);
    MARK(204);
    WiFi.mode(WIFI_STA);
    // 重连只归状态机管。核心库对 201/203 等也会自己 begin()，它的连接扫描会冲掉我们的
    // 扫描结果，两次关热点实测各丢一整轮（~6 s）。
    WiFi.setAutoReconnect(false);
    MARK(205);
    WiFi.setSleep(false);                   // 省电模式会把吞吐砍掉一个量级，测的就不是上限
    MARK(206);
    WiFi.disconnect(false, true);           // eraseap=true：清掉 NVS 里存的上一个 AP
    // 🔴 默认是 WIFI_FAST_SCAN：扫到第一个同名 AP 就连，完全不看信号。现场实验室网络有
    // 好几个 AP，实测它长期钉在 ch=1 的 -56 上，而 ch=6 那个是 -44(差 12 dB)，还被同信道
    // 一个 -44 的邻居压着。改全信道扫描，配合默认的 BY_SIGNAL 排序才会挑最强的。
    MARK(207);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
    MARK(208);
    g_wtDownSince = millis();
  }
  MARK(209);
  // 🔴 必须先挡住：WiFi 还没初始化，状态机不能跑。
  if (!g_wtInited) return;
  wtModePcClear();
  wtNetModeTick();
  if (g_wtApWhy) return;                   // 热点开着时连路由器的状态机停摆
  static bool wasUp = false;
  if (!wtLinkUp()) { wasUp = false; g_wtSrvUp = false; MARK(210); wtPickAp(); MARK(211); return; }
  g_wtDownSince = 0; g_wtRetriedLast = false; g_wtEmptyScans = 0;
  g_wtRadioResetAt = 0;   // 连上了就把"复位冷却"清掉，下次真卡死能立刻再复位
  g_wtNextTry = 0;        // 不复位的话上一轮的退避会压住下一轮的首次重连
  // 断开原因不清零：每次断开事件都会重写它，清零反而在缓存滞后时抹掉刚到的那个（09-23 白等 6 s）。
  // 🔑 只在【真连上之后】记住是哪一个，而不是在发起连接时记。否则一个连不上的 AP
  // 会被当成"上次连上的"，每次掉线都先白白直连它 12 秒。每次连上只查一次：SSID() 要拿驱动锁。
  MARK(212);
  if (!wasUp) {
    String ss = WiFi.SSID();
    if (ss.length()) {                      // 空串 = 其实刚断开，缓存还没跟上；别把上次的抹掉
      wasUp = true;
      g_wtLastPick = -1;
      for (int i = 0; i < g_wtList.n; i++)
        if (ss == g_wtList.e[i].ssid) { g_wtLastPick = (int8_t)i; break; }
      ncm_hb_ap = g_wtLastPick;
    }
  }
  // ⚠️ 测试服务器(9000/9001/9004)确实要等 IP 才能 begin，但「没 IP」绝不等于「要拆连接」。
  MARK(213);
  if (WiFi.status() != WL_CONNECTED) { g_wtSrvUp = false; return; }
  MARK(214);
  if (!g_wtCmdUp) { g_wtCmdUp = true; g_wtCmd.begin(9004); }
  // ⚠️ parsePacket 和下面两个 available() 每次都进 LwIP 拿锁，而 loop 每秒跑 7924 圈 ——
  // 裸调等于每秒上万次和 WiFi 任务抢锁。它们只负责收命令和接受新连接，20ms 一次足够。
  // 不能在这里 return：后面的 write 是测速数据通路，节流它会把吞吐打下来。
  static uint32_t lastPoll = 0;
  bool doPoll = (millis() - lastPoll >= 20);
  if (doPoll) lastPoll = millis();
  int cn = doPoll ? g_wtCmd.parsePacket() : 0;
  if (cn > 0 && g_wtCmdLine.isEmpty()) {
    char b[96];
    int n = g_wtCmd.read(b, sizeof(b) - 1);
    if (n > 0) {
      b[n] = 0;
      g_wtCmdFrom = g_wtCmd.remoteIP();
      g_wtCmdPort = g_wtCmd.remotePort();
      g_wtCmdLine = String(b);
      g_wtCmdLine.trim();
    }
  }
  if (!g_wtSrvUp) {
    g_wtSrvUp = true;
    WiFi.setSleep(false);                   // 要在拿到 IP 之后再关一次，begin 前设的会被连接流程覆盖
    // 发射功率拉满。同信道有 3 个 -47dBm 的邻居 AP，这种拥塞环境下每 dB 都要。
    // 单位 0.25dBm，80 = 20dBm(S3 上限)；连接流程会覆盖，所以也放在拿到 IP 之后。
    esp_wifi_set_max_tx_power(80);
    g_wtDown.begin();
    g_wtUp.begin();
    g_wtDown.setNoDelay(true);
  }

  if (doPoll && (!g_wtDownCli || !g_wtDownCli.connected())) {
    WiFiClient c = g_wtDown.available();
    if (c) { g_wtDownCli = c; g_wtDownCli.setNoDelay(true); g_wtBytes = 0; g_wtStart = millis(); g_wtConn++; }
  }
  // 每圈只写一个缓冲。availableForWrite() 在 WiFiClient 上恒返回 0，用它当门会一个字节都发不出去
  if (g_wtDownCli && g_wtDownCli.connected()) {
    int w = g_wtDownCli.write(g_wtBuf, sizeof(g_wtBuf));
    if (w > 0) g_wtBytes += w;
  }

  if (doPoll && (!g_wtUpCli || !g_wtUpCli.connected())) {
    WiFiClient c = g_wtUp.available();
    if (c) { g_wtUpCli = c; g_wtBytes = 0; g_wtStart = millis(); g_wtConn++; }
  }
  // UDP 没有窗口和重传，用它把「射频真的慢」和「TCP 窗口 / RTT 太小」分开
  if (g_wtUdpEnd && (int32_t)(millis() - g_wtUdpEnd) < 0) {
    for (int i = 0; i < 8; i++) {           // 一圈 8 包，别让 loop 频率成为测出来的上限
      g_wtUdp.beginPacket(g_wtUdpDst, 9002);
      g_wtUdp.write(g_wtBuf, 1400);
      if (g_wtUdp.endPacket()) { g_wtBytes += 1400; g_wtUdpPkts++; }
    }
  } else if (g_wtUdpEnd) {
    g_wtUdpEnd = 0;
  }

  if (g_wtUpCli && g_wtUpCli.connected()) {
    int n = g_wtUpCli.available();
    while (n > 0) { int r = g_wtUpCli.read(g_wtBuf, min(n, (int)sizeof(g_wtBuf))); if (r <= 0) break; g_wtBytes += r; n -= r; }
  }
}

// loop 里取走命令行。回复原路发回去，所以 PC 端 nc -u 就能看到结果。
static bool wifitestTakeCmd(String &out) {
  if (g_wtCmdLine.isEmpty()) return false;
  out = g_wtCmdLine;
  g_wtCmdLine = "";
  return true;
}

static void wifitestReplyCmd(const char *txt) {
  if (!g_wtCmdPort) return;
  g_wtCmd.beginPacket(g_wtCmdFrom, g_wtCmdPort);
  g_wtCmd.write((const uint8_t *)txt, strlen(txt));
  g_wtCmd.endPacket();
}

static void wifitestCtrlMark(uint32_t period_ms) {
  if (period_ms > g_wtCtrlMaxMs) g_wtCtrlMaxMs = period_ms;
}

// 命令路径用的现查（不走 wtLinkUp 的缓存：刚改完表要的是此刻连着谁）。热循环里别调。
static bool wtApNow(wifi_ap_record_t *ap) { return esp_wifi_sta_get_ap_info(ap) == ESP_OK; }

static void wifitestReport(Print &o) {
  // ⚠️ 射频关联和 IP 层必须分开报：网桥【刻意】清掉 STA IP，所以 status 恒为 0。
  // 旧版据此打「未连接」并跳过 ip/rssi/ch，而射频其实好好连着 —— 这条误导过排查。
  wifi_ap_record_t _ap;
  bool _link = wtApNow(&_ap);
  o.printf("WiFi 射频: %s", _link ? "已关联" : "未关联");
  if (_link) o.printf(" ssid=%s rssi=%d dBm ch=%d", (const char *)_ap.ssid, (int)_ap.rssi, (int)_ap.primary);
  o.printf("  | IP 层: status=%d%s ip=%s (网桥开着时没有 IP 是正常的)\n",
           (int)WiFi.status(), WiFi.status() == WL_CONNECTED ? "(有IP)" : "(无IP)",
           WiFi.localIP().toString().c_str());
  // 板子的 boot_timeline.py 按「<ms> <事件>[=<参数>]」逐项解析，改格式要两边一起改。
  o.printf("冷启动事件(ms):");
  for (uint8_t i = 0; i < g_bootEvN; i++) {
    o.printf("%s %lu %s", i ? " |" : "", (unsigned long)g_bootEv[i].ms, g_bootEv[i].what);
    const char *w = g_bootEv[i].what;
    int32_t a = g_bootEv[i].arg;
    bool isAp = !strcmp(w, "选中") || !strcmp(w, "盲连") || !strcmp(w, "直连上次") || !strcmp(w, "插盲连");
    if (isAp && a >= 0 && a < g_wtList.n && i >= g_bootEvListAt) o.printf("=%s", g_wtList.e[a].ssid);   // 序号看不出是哪个热点
    else if (isAp && a >= 0) o.printf("=旧表第%ld个", (long)a + 1);
    else if (a != BOOTEV_NOARG) o.printf("=%ld", (long)a);
  }
  if (ncm_host_ip_first_ms) o.printf(" | %lu 上位机首个带地址的包", (unsigned long)ncm_host_ip_first_ms);
  o.printf("%s | 现在 %lu\n", g_bootEvN >= BOOTEV_N ? " | (表满，后面的没记)" : "", (unsigned long)millis());
  if (g_msAssoc1)
    o.printf("首次关联的 AP: bssid=%02x:%02x:%02x:%02x:%02x:%02x ch=%d\n",
             g_bssid1[0], g_bssid1[1], g_bssid1[2], g_bssid1[3], g_bssid1[4], g_bssid1[5], (int)g_ch1);
  // 崩溃栈只打到没接线的调试口，闪存里这份是远程唯一拿得到的。固件号对上当前的才是这版崩的。
  {
    esp_core_dump_summary_t cd;
    char cur[17] = "";
    esp_ota_get_app_elf_sha256(cur, sizeof(cur));
    if (esp_core_dump_image_check() == ESP_OK && esp_core_dump_get_summary(&cd) == ESP_OK) {
      o.printf("上次崩溃: 任务=%s pc=0x%08lx cause=%lu vaddr=0x%08lx 崩溃固件=%.16s 当前固件=%s 回溯:",
               cd.exc_task, (unsigned long)cd.exc_pc, (unsigned long)cd.ex_info.exc_cause,
               (unsigned long)cd.ex_info.exc_vaddr, (const char *)cd.app_elf_sha256, cur);
      for (uint32_t i = 0; i < cd.exc_bt_info.depth && i < 16; i++) o.printf(" 0x%08lx", (unsigned long)cd.exc_bt_info.bt[i]);
      o.printf("%s\n", cd.exc_bt_info.corrupted ? " (回溯已损坏)" : "");
    } else {
      o.printf("上次崩溃: 闪存里没有崩溃现场 (当前固件=%s)\n", cur);
    }
  }
  // === 这次为什么起来的 ===
  // 🔴 必须打在 N 里。复位原因原先只在 `?` 的输出里，而 `?` 被串口来源白名单挡着；
  // BOOT 那行只走 Serial1，而开机头 90 秒 board-app 还没起、没人读 ttyS3 —— 2026-09-22
  // 查"冷启动为什么要 93 秒"时，ESP32 在 t≈26s 自己重启了一次，原因就是这么丢掉的。
  o.printf("本次启动: 第%lu次 复位原因=%s | USB掉线后重连 %lu 次 | 上次塌前最低 %.2fV%s\n",
           (unsigned long)wt_boot_cnt(), wt_reset_name(), (unsigned long)ncm_usb_reconn,
           wt_vmin_last(), wt_stuck_note());
  // === 链路稳定性 ===（测试 3 要的：掉了几次、总共断了多久、最后一次为什么断）
  {
    uint32_t now = millis();
    uint32_t ap = g_apMsTotal + (g_apAt ? (now - g_apAt) : 0);
    uint32_t up = now - ap;   // 分母只算该连路由器的时间
    uint32_t down = g_downMsTotal + (g_dropAt ? (now - g_dropAt) : 0);
    o.printf("稳定性: 关联%lu次 掉线%lu次 | 累计断开 %.1fs / 连路由器 %.1fs = %.2f%% (另有热点 %.1fs) | 最后断开原因 %d%s\n",
             (unsigned long)g_cntAssoc, (unsigned long)g_cntDrop,
             down / 1000.0, up / 1000.0, up ? 100.0 * down / up : 0.0, ap / 1000.0,
             (int)g_wtLastReason,
             g_wtLastReason == 8 ? "(8=对方主动踢，手机热点常用这个)"
             : g_wtLastReason == 201 ? "(201=找不到AP)"
             : g_wtLastReason == 4 ? "(4=对方认为我们掉线了)" : "");
  }
  if (g_wtRadioResetCnt) o.printf("射频驱动复位 %lu 次\n", (unsigned long)g_wtRadioResetCnt);
  o.printf("网络模式: 选的=%s 当前热点=%s%s\n", g_wtPrefMode ? "车的热点" : "连路由器", kWtApWhy[g_wtApWhy],
           g_wtPcForced ? " (本次是连续 3 次快速断电强制切的)" : "");
  o.printf("候选表: ");
  for (int i = 0; i < g_wtList.n; i++) o.printf("%d.%s%s ", i, g_wtList.e[i].ssid, i == g_wtLastPick ? "<=当前" : "");
  o.println();
  o.printf("省电=%d  UDP 已发 %lu 包\n", (int)WiFi.getSleep(), (unsigned long)g_wtUdpPkts);
  o.printf("连接数 %lu, 下行客户端 %s\n", (unsigned long)g_wtConn,
           (g_wtDownCli && g_wtDownCli.connected()) ? "在线" : "无");
  float s = g_wtStart ? (millis() - g_wtStart) / 1000.0f : 0;
  o.printf("吞吐: %lu KB / %.1f s = %.2f Mbit/s\n", (unsigned long)(g_wtBytes / 1024), s,
           s > 0.5f ? g_wtBytes * 8.0f / 1e6f / s : 0.0f);
  int8_t txp = 0;
  esp_wifi_get_max_tx_power(&txp);
  o.printf("发射功率 %.2f dBm\n", txp / 4.0f);
  o.printf("loop 间隔最大 %lu us, >1ms %lu 次, >10ms %lu 次\n",
           (unsigned long)g_wtLoopMaxUs, (unsigned long)g_wtLoopOver1ms,
           (unsigned long)g_wtLoopOver10ms);
  o.printf("控制环实际周期最大 %lu ms (标称 %d)\n", (unsigned long)g_wtCtrlMaxMs, CTRL_PERIOD_MS);
  {
    uint32_t ls = millis() - g_wtLoopSince;
    // 🔴 必须 64 位再乘。unsigned long 在 ESP32 上是 32 位，圈数超过 429 万(约 728 秒)
    // 时 `圈数*1000` 回绕，实测把 5925 圈/秒打成 76 —— 我据此误报过一次"主循环塌了"。
    o.printf("loop 转速: %lu 圈/%lu ms = %lu 圈/秒\n", (unsigned long)g_wtLoopCount,
             (unsigned long)ls, (unsigned long)(ls ? (uint64_t)g_wtLoopCount * 1000ULL / ls : 0));
  }
  o.printf("NCM: 注册rc=%d 桥接=%s 主机已开=%lu 链路=%lu mac=%s\n", ncm_reg_rc,
           ncm_bridge_running() ? "on" : "off",
           (unsigned long)ncm_opened, (unsigned long)ncm_link_up, ncm_mac_string());
  o.printf("  [USB->WiFi 方向] 丢 %lu 帧(WiFi 发送失败) / %lu 帧(队列满)\n",
           (unsigned long)ncm_drop_wifi, (unsigned long)ncm_drop_u2wq);
  o.printf("NTB 64字节倍数补位 %lu 次 (占 %.1f%%，这就是被主机当填充吞掉的那些)\n",
           (unsigned long)ncm_ntb_pad,
           ncm_xfer_submit ? 100.0f * ncm_ntb_pad / ncm_xfer_submit : 0.0f);
  o.printf("下行对帐: 进NTB %lu 帧 -> 真发出 %lu 个datagram (差 %ld = 卡在NTB里没发走)\n",
           (unsigned long)ncm_tx_frames, (unsigned long)ncm_dg_sent,
           (long)(ncm_tx_frames - ncm_dg_sent));
  o.printf("xfer 提交->完成 最长 %lu ms (门限 500ms；常态应是个位数)\n", (unsigned long)ncm_xmit_max_ms);
  o.printf("发送回调丢失就地恢复 %lu 次 (这条涨=治本修复在起作用，且没惊动整机看门狗)\n",
           (unsigned long)ncm_tx_recover);
  o.printf("ICMP 对帐: WiFi收到 %lu -> 发往USB %lu (持锁重查拒绝 %lu)\n",
           (unsigned long)ncm_icmp_in, (unsigned long)ncm_icmp_out,
           (unsigned long)ncm_xmit_reject);
  o.printf("WiFi收包回调 %lu 帧 -> 转发出去 %lu 帧 (差 %ld = 被静默挡掉)\n",
           (unsigned long)ncm_w2u_in, (unsigned long)ncm_tx_frames,
           (long)(ncm_w2u_in - ncm_tx_frames));
  o.printf("NCM 帧: USB->WiFi %lu (%lu B) / WiFi->USB %lu (%lu B) / 丢 队列%lu 发送%lu\n",
           (unsigned long)ncm_rx_frames, (unsigned long)ncm_rx_bytes,
           (unsigned long)ncm_tx_frames, (unsigned long)ncm_tx_bytes,
           (unsigned long)ncm_drop_q, (unsigned long)ncm_drop_tx);
  {
    uint32_t el = millis() - g_ctrlSince;
    o.printf("控制任务: 占 %0.1f%% 一个核 (唤醒 %lu 次/%lums, 累计忙 %lu ms)\n",
             el ? g_ctrlBusyUs / 10.0f / el : 0.0f, (unsigned long)g_ctrlWakes,
             (unsigned long)el, (unsigned long)(g_ctrlBusyUs / 1000));
  }
  o.printf("空闲堆 现在 %lu B / 最低 %lu B, PSRAM %lu B\n",
           (unsigned long)ESP.getFreeHeap(), (unsigned long)g_wtHeapMin,
           (unsigned long)ESP.getPsramSize());
}

// 扫一圈：连不上时先分清是「射频起不来」还是「认证失败」——前者一个 AP 都扫不到。
static void wifitestScan(Print &o) {
  o.println(F("扫描中..."));
  int n = WiFi.scanNetworks();
  o.printf("看到 %d 个 AP\n", n);
  for (int i = 0; i < n && i < 12; i++)
    o.printf("  %-24s ch=%2d rssi=%d enc=%d\n", WiFi.SSID(i).c_str(), WiFi.channel(i),
             WiFi.RSSI(i), (int)WiFi.encryptionType(i));
  WiFi.scanDelete();
  // ⚠️ 上面的 scanNetworks 是同步的，会阻塞 loop 约 2 秒 —— 车在动的时候别敲这个命令。
  // 这里只把状态机踢回「重新挑」，不自己 begin，免得和它抢。
  // ⚠️ 必须把 g_wtRetriedLast 置真：否则状态机会先"直连重试上次那个"，又连回原来的 AP，
  // 这条命令的本意（强制按优先级重挑）就落空了。
  g_wtScanning = false; g_wtNextTry = 0; g_wtDownSince = millis();
  g_wtRetriedLast = true; g_wtEmptyScans = 0;
  WiFi.disconnect(false, false);
  o.println(F("已请求重新挑选 AP（按候选表的优先级）"));
}

// 热点模式：ESP32 自己当 AP，网桥改挂 AP 接口，手机连上后由板子（192.168.4.1）发地址、出网页。
// 单射频：开着热点时不能扫描或连路由器，否则切信道会把手机踢掉，所以两种模式二选一、期间状态机停摆。
static void wtApSsid(char *out, size_t n) {
  if (g_wtAp.ssid[0]) { snprintf(out, n, "%s", g_wtAp.ssid); return; }
  uint8_t m[6];
  esp_wifi_get_mac(WIFI_IF_STA, m);
  snprintf(out, n, "Looper-%02X%02X", m[4], m[5]);
}

static const char *wtApPass() { return g_wtAp.ssid[0] ? g_wtAp.pass : WT_AP_PASS; }

static void wtFbScanStop() {
  if (!g_wtFbScanning) return;
  esp_wifi_scan_stop();
  WiFi.scanDelete();
  g_wtFbScanning = false;
}

// why 见 g_wtApWhy；ms=0 不限时。已经开着就只改原因和时限，不重建热点（手机不用重连）。
static void wifitestApStart(uint8_t why, uint32_t ms, Print &o) {
  char ssid[WL_SSID_MAX + 1];
  wtApSsid(ssid, sizeof(ssid));
  bool was = g_wtApWhy != 0;
  wtFbScanStop();
  g_wtApWhy = why;
  g_wtApUntil = ms ? (millis() + ms) | 1 : 0;
  if (why == 3) g_wtFbScanAt = millis() + WT_FB_SCAN_MS;
  if (was) { o.printf("热点本来开着，改为%s\n", kWtApWhy[why]); return; }
  g_apAt = millis() | 1;
  if (g_dropAt) { g_downMsTotal += g_apAt - g_dropAt; g_dropAt = 0; }
  bootEv("热点开", why);
  wtLog("[WIFI] t=%lu 开热点(%s)\n", (unsigned long)millis(), kWtApWhy[why]);
  if (g_wtScanning) { esp_wifi_scan_stop(); WiFi.scanDelete(); g_wtScanning = false; }
  WiFi.disconnect(false, false);
  ncm_bridge_hold(1);                      // 换模式期间别往 WiFi 发，见 ncm_glue.c 的 s_tx_mu
  WiFi.mode(why == 3 ? WIFI_AP_STA : WIFI_AP);   // 兜底要能扫已存网络，得留着 STA
  bool ok = WiFi.softAP(ssid, wtApPass(), WT_AP_CHANNEL, 0, 4);
  esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (ap) esp_netif_dhcps_stop(ap);        // 地址由板子的 dnsmasq 发，ESP32 自己的 DHCP 服务会抢答
  ncm_bridge_set_ap(1);
  ncm_bridge_hold(0);
  if (!ncm_bridge_running()) ncm_bridge_start();   // USB 还没就绪时开不起来，由网桥策略在就绪后补开
  o.printf("热点 %s 已开(%s，%s)，密码 %s，信道 %d%s；板子应在 192.168.4.1\n", ssid, kWtApWhy[why],
           ok ? "ok" : "失败", wtApPass(), WT_AP_CHANNEL, ms ? "，限时" : "");
}

// 热点开着时换名字/密码：原地改配置，不能先关再开 —— 中间那一下心跳会报「切回路由器」，
// 板子就撤掉 192.168.4.1 去续租，28 s 连不上（09-28 实测）。手机会被踢，要用新密码重连。
static void wtApReconfig() {
  char ssid[WL_SSID_MAX + 1];
  wtApSsid(ssid, sizeof(ssid));
  ncm_bridge_hold(1);
  bool ok = WiFi.softAP(ssid, wtApPass(), WT_AP_CHANNEL, 0, 4);
  esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (ap) esp_netif_dhcps_stop(ap);
  ncm_bridge_hold(0);
  bootEv("热点重开", ok);
  wtLog("[WIFI] t=%lu 热点换成 %s (%s)\n", (unsigned long)millis(), ssid, ok ? "ok" : "失败");
}

static void wifitestApStop(Print &o) {
  if (!g_wtApWhy) { o.println(F("不在热点模式")); return; }
  if (g_trSt == 1 || g_trSt == 2) { if (g_trSt == 1) esp_wifi_scan_stop(); g_trSt = 3; bootEv("试连被打断"); }
  bootEv("热点关", g_wtApWhy);
  wtFbScanStop();
  ncm_bridge_hold(1);
  ncm_bridge_set_ap(0);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  ncm_bridge_hold(0);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  g_wtApWhy = 0;
  g_wtApUntil = 0;
  g_apMsTotal += millis() - g_apAt; g_apAt = 0;
  g_dropAt = millis() | 1;                 // 到连上路由器之前算断开，但不算一次掉线
  // 上次就是排第一的才直连，否则扫描按优先级挑：热点期间用户可能刚开了更靠前的网（09-28 直连回了实验室网络、没选排第一的手机热点）
  g_wtRetriedLast = g_wtLastPick != 0;
  g_wtDownSince = millis();
  g_wtNextTry = millis();
  o.println(F("热点已关，切回连路由器"));
}

// ===== 切到路由器前试连（Nwt 开始 / Nwts 查结果）=====
// 热点不关，用 STA 按候选表顺序试连附近的网络，第一个拿到地址的就停。ESP32 和板子共用一个 MAC，
// 切过去后路由器多半给板子同一个地址，网页据此告诉用户去哪儿找车；顺带当场查出密码错。
// 单射频：STA 连上后热点被迫换到路由器的信道，手机会断几秒再自己连回来，网页要能挺过去。
#define WT_TRY_EACH_MS 12000
#define WT_TRY_TOTAL_MS 50000

static void wtTryEnd(const char *why) {
  WiFi.disconnect(false, false);           // 不清配置也不退租：留着租约，板子过去时多半还是这个地址
  g_trSt = 3;
  bootEv("试连结束", g_trOk);
  wtLog("[WIFI] t=%lu 试连结束(%s) 结果=%d\n", (unsigned long)millis(), why, (int)g_trOk);
}

static void wtTryBegin(uint32_t now) {
  const WlEntry &e = g_wtList.e[g_trIdx];
  g_wtAttemptFailed = false; g_trIp = 0;
  WiFi.begin(e.ssid, e.pass[0] ? e.pass : NULL);
  g_trBy = now + WT_TRY_EACH_MS;
  g_trSt = 2;
  bootEv("试连", g_trIdx);
}

static void wtTryNext(uint32_t now) {
  WiFi.disconnect(false, false);
  g_trRetried = false;
  while (++g_trIdx < g_wtList.n) {
    if (!g_trSeen[g_trIdx]) { g_trRes[g_trIdx] = WT_TRY_ABSENT; continue; }
    wtTryBegin(now);
    return;
  }
  wtTryEnd("试完了");
}

static void wtTryTick() {
  uint32_t now = millis();
  if (g_trSt == 1) {
    int n = WiFi.scanComplete();
    int doneN = (g_wtScanDoneAt && (int32_t)(g_wtScanDoneAt - g_trStart) >= 0) ? g_wtScanDoneN : -1;
    if (wscan_verdict(n, now - g_trStart, 9000, doneN) == WSCAN_RUNNING) return;
    for (int j = 0; j < n; j++) {
      int k = wl_find(&g_wtList, WiFi.SSID(j).c_str());
      if (k >= 0) g_trSeen[k] = true;
    }
    WiFi.scanDelete();
    wtTryNext(now);
    return;
  }
  if (g_trSt != 2) return;
  wifi_ap_record_t ap;
  bool assoc = wtApNow(&ap);
  if (assoc && g_trIp) {
    g_trRes[g_trIdx] = WT_TRY_OK; g_trOk = g_trIdx; g_trRssi = ap.rssi;
    wtTryEnd("拿到地址");
    return;
  }
  bool over = wm_after(now, g_trStart + WT_TRY_TOTAL_MS);
  if (g_wtAttemptFailed || wm_after(now, g_trBy) || over) {
    g_trRes[g_trIdx] = wm_try_code(g_wtAttemptFailed ? g_wtLastReason : 0, assoc);
    // 空口很忙时握手包会丢，密码对也报 reason 15（09-28 热点上灌广播时实测）：判密码错之前重试一次
    if (!over && g_trRes[g_trIdx] == WT_TRY_PASS && !g_trRetried) {
      g_trRetried = true;
      WiFi.disconnect(false, false);
      wtTryBegin(now);
    } else if (over) wtTryEnd("总时长到");
    else wtTryNext(now);
  }
}

static void wtIp(Print &o, uint32_t a) { o.printf("%u.%u.%u.%u", a & 255, (a >> 8) & 255, (a >> 16) & 255, a >> 24); }

static void wifitestTryCmd(const String &rest, Print &o) {
  if (rest == "wt") {
    if (!g_wtApWhy) { o.println(F("WIFITRY notap")); return; }       // 连着路由器时用不着，也会打断现有连接
    if (g_trSt == 1 || g_trSt == 2) { o.println(F("WIFITRY busy")); return; }
    if (g_wtFbScanning) { esp_wifi_scan_stop(); WiFi.scanDelete(); g_wtFbScanning = false; }
    memset(g_trRes, 0, sizeof(g_trRes)); memset(g_trSeen, 0, sizeof(g_trSeen));
    g_trIdx = -1; g_trOk = -1; g_trIp = g_trGw = 0; g_trRssi = 0;
    WiFi.setAutoReconnect(false);           // 否则核心库会在认证失败后自己重连，打乱逐个试
    if (WiFi.getMode() != WIFI_AP_STA) { ncm_bridge_hold(1); WiFi.mode(WIFI_AP_STA); ncm_bridge_hold(0); }
    g_trStart = millis();
    if (WtScan::start(150) != WIFI_SCAN_RUNNING) { g_trSt = 3; o.println(F("WIFITRY scanfail")); return; }
    g_trSt = 1;
    bootEv("试连开始");
    o.println(F("WIFITRY started"));
    return;
  }
  // wts：st=idle|scan|conn|done ok=<序号|-1> ip= gw= rssi= res=<每个候选一位，见 WT_TRY_*>
  static const char *const kSt[] = {"idle", "scan", "conn", "done"};
  o.printf("WIFITRY st=%s ok=%d ip=", kSt[g_trSt], (int)g_trOk);
  wtIp(o, g_trOk >= 0 ? g_trIp : 0);
  o.print(" gw=");
  wtIp(o, g_trOk >= 0 ? g_trGw : 0);
  o.printf(" rssi=%d res=", (int)g_trRssi);
  for (int i = 0; i < g_wtList.n; i++) o.print((int)g_trRes[i]);
  o.println();
}

// 兜底热点里每 30 s 扫一次：扫到已存网络就回去连它。有手机连着时不扫 —— 扫描轮流切信道，会把它踢掉。
static void wtFallbackTick() {
  uint32_t now = millis();
  if (g_wtFbScanning) {
    int n = WiFi.scanComplete();
    int doneN = (g_wtScanDoneAt && (int32_t)(g_wtScanDoneAt - g_wtFbScanStart) >= 0) ? g_wtScanDoneN : -1;
    WScanVerdict v = wscan_verdict(n, now - g_wtFbScanStart, 9000, doneN);
    if (v == WSCAN_RUNNING) return;
    int hit = -1;
    for (int j = 0; j < n && hit < 0; j++) hit = wl_find(&g_wtList, WiFi.SSID(j).c_str());
    wtFbScanStop();
    g_wtFbScanAt = now + WT_FB_SCAN_MS;
    if (hit >= 0) {
      wtLog("[WIFI] t=%lu 兜底热点里扫到已存网络 %s，回去连它\n", (unsigned long)now, g_wtList.e[hit].ssid);
      wifitestApStop(Serial1);
    }
    return;
  }
  if ((int32_t)(now - g_wtFbScanAt) < 0) return;
  g_wtFbScanAt = now + WT_FB_SCAN_MS;
  if (WiFi.softAPgetStationNum() > 0) return;
  if (WtScan::start(150) == WIFI_SCAN_RUNNING) { g_wtFbScanning = true; g_wtFbScanStart = now; }
}

// 模式相关的每圈检查：推迟执行的切换、限时热点到点、热点模式下热点被关掉（Napx）要重开、兜底扫描。
static void wtNetModeTick() {
  uint32_t now = millis();
  bool due = g_wtModeAt && (int32_t)(now - g_wtModeAt) >= 0;
  if (due) g_wtModeAt = 0;
  bool expired = g_wtApUntil && (int32_t)(now - g_wtApUntil) >= 0;
  switch (wm_tick(g_wtPrefMode, g_wtApWhy, due, g_wtModeWant, g_wtModeAt != 0, expired)) {
    case WM_AP_PERSIST: wifitestApStart(2, 0, Serial1); break;
    case WM_AP_STOP: wifitestApStop(Serial1); break;
  }
  if (g_wtApRebuildAt && (int32_t)(now - g_wtApRebuildAt) >= 0) {
    g_wtApRebuildAt = 0;
    if (g_wtApWhy) wtApReconfig();
  }
  if (g_trSt == 1 || g_trSt == 2) wtTryTick();
  else if (g_wtApWhy == 3) wtFallbackTick();
  if (g_wtConfirmBy && wm_after(now, g_wtConfirmBy)) {
    g_wtConfirmBy = 0;
    Preferences p;
    if (p.begin("netmode", false)) { p.putUChar("m", 1); p.remove("pe"); p.end(); }
    g_wtPrefMode = 1; g_wtModeWant = 1; g_wtModeAt = now | 1;
    bootEv("没人确认,回热点");
    wtLog("[WIFI] t=%lu 切到路由器后 %d s 没人从新网络打开网页，回到车的热点\n", (unsigned long)now, WT_CONFIRM_MS / 1000);
  }
}

// 开机读模式。「开机 10 秒内断电」连续 3 次 = 强制切热点：路由器模式下进不来时的最后一招，不用拆车。
static void wtModeLoad() {
  Preferences p;
  uint8_t m = WT_DEFAULT_MODE;
  if (p.begin("netmode", false)) {
    m = p.getUChar("m", WT_DEFAULT_MODE);
    bool on = esp_reset_reason() == ESP_RST_POWERON;
    uint8_t pc = wm_pc_next(p.getUChar("pc", 0), on, &g_wtPcForced);
    if (g_wtPcForced) { m = 1; p.putUChar("m", 1); }
    // 空表强制热点并存下来：网页加了网络之后，要用户在网页上切路由器（带确认）才离开热点
    if (wm_effective_mode(m, g_wtList.n) != m) { m = 1; p.putUChar("m", 1); }
    if (on) p.putUChar("pc", pc);
    if (m == 0 && p.getUChar("pe", 0)) g_wtConfirmBy = (millis() + WT_CONFIRM_MS) | 1;   // 待确认期间断过电：重新计时
    if (!p.isKey("as") || !p.isKey("ap") || p.getString("as", g_wtAp.ssid, sizeof(g_wtAp.ssid)) == 0
        || p.getString("ap", g_wtAp.pass, sizeof(g_wtAp.pass)) < 8)
      g_wtAp = {};                         // 读不全就当没改过，别开出一个没密码的热点
    p.end();
  }
  g_wtPrefMode = wm_effective_mode(m, g_wtList.n);   // 闪存打不开时也成立
}

// 活过 10 秒就不算「快速断电」了，计数清零
static void wtModePcClear() {
  static bool done = false;
  if (done || millis() < 10000) return;
  done = true;
  Preferences p;
  if (p.begin("netmode", false)) { if (p.getUChar("pc", 0)) p.putUChar("pc", 0); p.end(); }
}

// Nm 查看 / Nm0 连路由器 / Nm1 车的热点。先存闪存、回复，1.5 s 后再切，否则网页收不到这句回复。
static void wifitestModeCmd(const String &rest, Print &o) {
  if (rest == "m0" && !g_wtList.n) { o.println(F("WIFIMODE empty")); return; }
  if (rest == "m0" || rest == "m1") {
    uint8_t m = rest[1] - '0';
    bool pend = m == 0 && (g_wtPrefMode == 1 || g_wtApWhy == 3);   // 从车的热点（含兜底热点）切出去才需要确认
    Preferences p;
    bool ok = p.begin("netmode", false) && p.putUChar("m", m) == 1;
    if (ok && pend) ok = p.putUChar("pe", 1) == 1;
    else if (ok) p.remove("pe");
    p.end();
    if (!ok) { o.println(F("WIFIMODE nvs")); return; }
    g_wtPrefMode = m; g_wtModeWant = m; g_wtModeAt = (millis() + 1500) | 1;
    g_wtConfirmBy = pend ? (millis() + WT_CONFIRM_MS) | 1 : 0;
  }
  if (rest == "mc" && g_wtConfirmBy) {
    Preferences p;
    if (p.begin("netmode", false)) { p.remove("pe"); p.end(); }
    g_wtConfirmBy = 0;
    bootEv("路由器模式已确认");
  }
  o.printf("WIFIMODE ok pref=%s now=%s confirm=%lu\n", g_wtPrefMode ? "ap" : "sta", kWtApWhy[g_wtApWhy],
           g_wtConfirmBy ? (unsigned long)((g_wtConfirmBy - millis()) / 1000) : 0UL);
}

// 换了候选表之后：当前连着的网还在表里就不动它（用户多半正通过它改表），不在了才断开重挑。
static void wtListApply() {
  wifi_ap_record_t ap;
  bool link = !g_wtApWhy && wtApNow(&ap);
  g_wtLastPick = link ? (int8_t)wl_find(&g_wtList, (const char *)ap.ssid) : -1;
  ncm_hb_ap = g_wtLastPick;
  g_wtBlindIdx = 0; g_wtEmptyScans = 0; g_wtNextTry = 0;
  if (link && g_wtLastPick < 0) {
    bootEv("候选表换掉了当前网");
    g_wtRetriedLast = true;                // 别先定向直连刚被删掉的那个
    g_wtDownSince = millis();
    WiFi.disconnect(false, false);
  }
}

// 一行列出候选表，给板子解析。只出 SSID 的十六进制和「有没有密码」，密码永远不出 ESP32。
static void wifitestListPrint(Print &o) {
  wifi_ap_record_t ap;
  bool link = !g_wtApWhy && wtApNow(&ap);
  char h[2 * WL_PASS_MAX + 1], ss[WL_SSID_MAX + 1];
  wtApSsid(ss, sizeof(ss));
  wl_hex(ss, h);
  o.printf("WIFILIST n=%d src=%s mode=%s pref=%s why=%d apssid=%s",
           (int)g_wtList.n, g_wtListFromNvs ? "nvs" : "builtin", g_wtApWhy ? "ap" : "sta",
           g_wtPrefMode ? "ap" : "sta", (int)g_wtApWhy, h);
  wl_hex(wtApPass(), h);
  o.printf(" confirm=%lu", g_wtConfirmBy ? (unsigned long)((g_wtConfirmBy - millis()) / 1000) : 0UL);
  o.printf(" appass=%s apdef=%d cur=%d rssi=%d", h, g_wtAp.ssid[0] ? 0 : 1,
           (int)(link ? wl_find(&g_wtList, (const char *)ap.ssid) : -1), link ? (int)ap.rssi : 0);
  for (int i = 0; i < g_wtList.n; i++) {
    wl_hex(g_wtList.e[i].ssid, h);
    o.printf(" %s:%d", h, g_wtList.e[i].pass[0] ? 1 : 0);
  }
  o.println();
}

// Nwb 开始 / Nwa <ssid十六进制> <密码十六进制|-|*> 追加 / Nwc <条数> 提交存闪存 / Nwd 恢复编译进来的表。
// 分行传是因为串口一行装不下 5 条；提交时核对条数，中途丢行就整表作废，不会存进半张表。
static void wifitestListCmd(const String &rest, Print &o) {
  if (rest == "wl") { wifitestListPrint(o); return; }
  if (rest == "wt" || rest == "wts") { wifitestTryCmd(rest, o); return; }
  if (rest == "wb") { g_wtStage = {}; o.println(F("WIFISET ok stage")); return; }
  if (rest.startsWith("wa ")) {
    int sp = rest.indexOf(' ', 3);
    WlErr e = sp < 0 ? WL_EHEX
            : wl_stage_add(&g_wtStage, &g_wtList, rest.substring(3, sp).c_str(), rest.substring(sp + 1).c_str());
    o.printf("WIFISET %s add n=%d\n", wl_err_name(e), (int)g_wtStage.n);
    return;
  }
  if (rest.startsWith("wc ")) {
    WlErr e = wl_commit_check(&g_wtStage, rest.substring(3).toInt());
    if (e != WL_OK) { o.printf("WIFISET %s commit\n", wl_err_name(e)); return; }
    Preferences p;
    bool saved = p.begin("wifilist", false);
    if (saved) {
      p.putUChar("n", 0);                  // 先作废旧表：中途断电读到的是「没有」而不是半新半旧
      for (int i = 0; i < WL_MAX; i++) {
        char k[4] = {'s', (char)('0' + i), 0};
        const WlEntry &e = g_wtStage.e[i];
        // 表外的槽删掉：被删的网络密码不留在闪存里。putString 写空串也返回 0，所以空密码走 remove
        if (i >= g_wtStage.n) { p.remove(k); k[0] = 'p'; p.remove(k); continue; }
        saved = saved && p.putString(k, e.ssid) == strlen(e.ssid); k[0] = 'p';
        if (!e.pass[0]) p.remove(k);
        else saved = saved && p.putString(k, e.pass) == strlen(e.pass);
      }
      saved = saved && p.putUChar("n", g_wtStage.n) == 1;
      p.end();
    }
    if (!saved) { o.println(F("WIFISET nvs commit")); return; }
    g_wtList = g_wtStage; g_wtListFromNvs = true;
    wtListApply();
    bootEv("候选表已更新", g_wtList.n);
    g_bootEvListAt = g_bootEvN;
    o.printf("WIFISET ok commit n=%d\n", (int)g_wtList.n);
    return;
  }
  // Nwh <名字hex> <密码hex> 改车的热点 / Nwh - 恢复出厂（Looper-MAC 后四位 + looper1234）
  if (rest.startsWith("wh ")) {
    WlEntry ap = {};
    int sp = rest.indexOf(' ', 3);
    WlErr e = rest == "wh -" ? WL_OK
            : sp < 0 ? WL_EHEX : wl_ap_parse(rest.substring(3, sp).c_str(), rest.substring(sp + 1).c_str(), &ap);
    if (e != WL_OK) { o.printf("WIFISET %s ap\n", wl_err_name(e)); return; }
    Preferences p;
    bool saved = p.begin("netmode", false);
    if (saved) {
      if (!ap.ssid[0]) { p.remove("as"); p.remove("ap"); }
      else saved = p.putString("ap", ap.pass) == strlen(ap.pass) && p.putString("as", ap.ssid) == strlen(ap.ssid);
      p.end();
    }
    if (!saved) { o.println(F("WIFISET nvs ap")); return; }
    g_wtAp = ap;
    bootEv("热点名已改");
    if (g_wtApWhy) g_wtApRebuildAt = (millis() + 1500) | 1;
    o.printf("WIFISET ok ap%s\n", g_wtApWhy ? " restart" : "");
    return;
  }
  if (rest == "wd") {
    Preferences p;
    if (p.begin("wifilist", false)) { p.clear(); p.end(); }
    wtListLoad();
    wtListApply();
    if (!g_wtList.n && !g_wtPrefMode) {     // 恢复成空表：路由器模式待不住，切热点
      if (p.begin("netmode", false)) { p.putUChar("m", 1); p.remove("pe"); p.end(); }
      g_wtPrefMode = 1; g_wtModeWant = 1; g_wtModeAt = (millis() + 1500) | 1; g_wtConfirmBy = 0;
    }
    bootEv("候选表恢复默认", g_wtList.n);
    g_bootEvListAt = g_bootEvN;
    o.printf("WIFISET ok default n=%d\n", (int)g_wtList.n);
    return;
  }
  o.println(F("WIFISET usage"));
}

// 'Nd' 掉线演练：断开一次，走定向直连上次 AP。'Ndv' 模拟 AP 消失：上次的 BSSID 换成不存在的，
// 再像核心库那样对它发起一次连接占住射频 —— 状态机要打断它、2 s 放弃、扫描、按优先级重挑。
// 'Ndx' 同 Ndv，但把「上次」换成列表第 0 项（手机热点，要先关掉它）：SSID 也不存在才是真实关热点。
// 'Ndh' 断开并 90 s 不重连：验证长时间断网时板子的 USB 角色守护和 netheal 都不乱动。
static void wifitestDrill(char kind, Print &o) {
  if (g_wtApWhy) { o.println(F("热点开着，不能做掉线演练")); return; }
  if (kind == 'h') {
    bootEv("演练:断网90s");
    g_wtHoldUntil = millis() + 90000;
    WiFi.disconnect(false, false);
    o.println(F("已断开，90 秒内不重连"));
    return;
  }
  bool vanished = (kind == 'v' || kind == 'x');
  bootEv(kind == 'x' ? "演练:热点消失" : vanished ? "演练:AP消失" : "演练:掉线");
  if (kind == 'x') g_wtLastPick = 0;
  WiFi.disconnect(false, false);
  if (vanished) {
    static const uint8_t fake[6] = {0x02, 0, 0, 0, 0, 0x01};   // 本地管理地址，不会撞上真 AP
    memcpy(g_wtCurBssid, fake, 6);
    WiFi.setScanMethod(WIFI_FAST_SCAN);
    WiFi.begin(g_wtList.e[g_wtLastPick >= 0 ? g_wtLastPick : 0].ssid,
               g_wtList.e[g_wtLastPick >= 0 ? g_wtLastPick : 0].pass, g_wtCurCh, fake);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  }
  o.println(vanished ? F("已模拟 AP 消失，约 10 秒内应重新连上") : F("已断开，约 1 秒内应重新连上"));
}

// 'Nu <ip>' 向该地址 9002 口狂发 8 秒 UDP
static void wifitestUdpStart(const String &ip, Print &o) {
  if (!g_wtUdpDst.fromString(ip)) { o.println(F("地址解析失败")); return; }
  g_wtUdp.begin(9003);
  g_wtBytes = 0; g_wtUdpPkts = 0; g_wtStart = millis(); g_wtUdpEnd = millis() + 8000;
  o.printf("向 %s:9002 发 8 秒 UDP\n", ip.c_str());
}

// 'Nb <ip>' 独占 CPU 发 5 秒 UDP：loop 里的 1kHz ADC 和 20Hz 控制环全不参与，
// 用来判定 0.19 Mbit/s 是被这些挤出来的，还是射频本身就这么慢。
static void wifitestBlast(const String &ip, Print &o) {
  IPAddress d;
  if (!d.fromString(ip)) { o.println(F("地址解析失败")); return; }
  g_wtUdp.begin(9003);
  uint32_t pkts = 0, fails = 0, t0 = millis();
  while (millis() - t0 < 5000) {
    // 一轮 8 包再让出：只发 1 包的话 delay(1) 自己就成了 11 Mbit/s 的天花板，
    // 量到的是节拍不是射频上限。
    for (int i = 0; i < 8; i++) {
      g_wtUdp.beginPacket(d, 9002);
      g_wtUdp.write(g_wtBuf, 1400);
      if (g_wtUdp.endPacket()) pkts++; else fails++;
    }
    // 发送是异步的，实际搬包的是 WiFi/lwIP 任务。不让出 CPU 就是自己把它们饿死，
    // 量出来的是调度失败率而不是射频能力 —— 第一版就栽在这，90% 的"失败"是假的。
    delay(1);
  }
  float s = (millis() - t0) / 1000.0f;
  o.printf("独占发送: %lu 包成功 / %lu 失败 / %.1f s = %.2f Mbit/s (rssi=%d)\n",
           (unsigned long)pkts, (unsigned long)fails, s, pkts * 1400.0f * 8 / 1e6f / s,
           WiFi.RSSI());
}

static void wifitestResetStats() {
  g_ctrlBusyUs = 0; g_ctrlWakes = 0; g_ctrlSince = millis();
  g_wtLoopMaxUs = g_wtLoopOver1ms = g_wtLoopOver10ms = g_wtCtrlMaxMs = 0;
  g_wtLoopCount = 0; g_wtLoopSince = millis();
  g_wtHeapMin = 0xFFFFFFFF;
  g_wtBytes = 0;
  g_wtStart = millis();
}
#else
static inline void wifitestBegin() {}
static inline void wifitestTick() {}
static inline void wifitestCtrlMark(uint32_t) {}
static inline void wifitestReport(Print &) {}
static inline void wifitestResetStats() {}
static inline void wifitestListCmd(const String &, Print &) {}
static inline void wifitestModeCmd(const String &, Print &) {}
static inline void wifitestTryCmd(const String &, Print &) {}
#endif
