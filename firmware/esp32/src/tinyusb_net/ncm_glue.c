// USB NCM 网卡 + WiFi 二层网桥：X5 把 ESP32 当一块普通网卡，以太网帧原样搬到 WiFi 上。
//
// 为什么必须是"网卡"而不是"串口"：X5 内核 CONFIG_USB_ACM 和 CONFIG_PPP 都没编，
// 三条路里只有 cdc_ncm.ko 这一条活着。
//
// 桥接照搬 ESP-IDF 官方 tusb_ncm 例子：不过 esp_netif，直接在以太网帧层对接。
// Arduino 的 TinyUSB 预编译库里没有 NCM 类(符号数 0)，所以 ncm_device.c 自带、
// 再用 weak 的 usbd_app_driver_get_cb() 把它挂进协议栈。
#include "sdkconfig.h"
#if CONFIG_TINYUSB_ENABLED

#include <string.h>
#include <stdio.h>
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "net_device.h"
#include "esp32-hal-tinyusb.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Arduino SDK 不暴露 esp_wifi_internal.h，但符号在 libnet80211.a / libpp.a 里
typedef esp_err_t (*ncm_wifi_rxcb_t)(void *buffer, uint16_t len, void *eb);
esp_err_t esp_wifi_internal_tx(wifi_interface_t ifx, void *buffer, uint16_t len);
esp_err_t esp_wifi_internal_reg_rxcb(wifi_interface_t ifx, ncm_wifi_rxcb_t fn);
void esp_wifi_internal_free_rx_buffer(void *buffer);

#define NCM_TAG "ncm"
#define NCM_QUEUE_LEN 32
#define NCM_U2W_WAIT_MS 20      // 队列满时最多等多久(背压)，等不到才丢
#define U2W_QUEUE_LEN 16        // USB 进 WiFi 出，速率接近，主要吸收抖动
#define NCM_FRAME_MAX 1600

typedef struct { uint16_t len; uint8_t data[NCM_FRAME_MAX]; } ncm_frame_t;

static QueueHandle_t s_w2u;              // WiFi 收到的帧，等着发去 USB
static QueueHandle_t s_u2w;              // USB 收到的帧，等着发去 WiFi
static bool s_bridge_on = false;
// 网桥挂在哪个 WiFi 接口上。热点模式下换成 AP：手机发来的帧原样转给板子，板子当 192.168.4.1。
static wifi_interface_t s_wifi_if = WIFI_IF_STA;

uint8_t tud_network_mac_address[6] = {0x02, 0x02, 0x84, 0x6a, 0x96, 0x00};
static char s_mac_str[13];

// 计数器：全部走 volatile，读的人在另一个任务里
volatile uint32_t ncm_rx_frames, ncm_rx_bytes;    // USB -> WiFi
volatile uint32_t ncm_tx_frames, ncm_tx_bytes;    // WiFi -> USB
volatile uint32_t ncm_drop_q, ncm_drop_tx, ncm_link_up, ncm_opened;
volatile uint32_t ncm_w2u_in;      // 进 WiFi 收包回调的总帧数(含被条件挡掉的)
volatile uint32_t ncm_icmp_in, ncm_icmp_out;   // 只数 ICMP，用来和两端对帐
// 上位机(Looper)的 IP。网桥是二层的、本来不看 IP，但"板子现在是什么地址"在换 AP、换网段
// 之后是最难回答的一件事：板子跑到别的网段后 PC 就够不着它，只能靠人在手机上试。
// 板子发出去的每个 ARP / IP 包里都带着它自己的地址，顺手记一下，COM 口就能直接问出来。
volatile uint32_t ncm_host_ip;          // 网络字节序
volatile uint32_t ncm_host_ip_ms;       // 最后一次看到的时刻
volatile uint32_t ncm_host_ip_first_ms; // 第一次看到的时刻（冷启动用时）
// ⚠️ 曾经和 WiFi->USB 方向共用 ncm_drop_tx，两个方向的丢包混在一个数里，根本分不清谁在丢
volatile uint32_t ncm_drop_wifi;   // USB->WiFi 方向：esp_wifi_internal_tx 失败
volatile uint32_t ncm_drop_u2wq;   // USB->WiFi 方向：队列满
volatile uint32_t ncm_stalls;        // 发送端卡死后自愈的次数
// 断流探针：射频停摆时这三个是唯一能分辨"驱动拒收"还是"驱动收下但没发出去"的证据
volatile uint32_t ncm_last_w2u_ms;   // 最后一次从 WiFi 收到帧的时刻
volatile uint32_t ncm_tx_retry;      // esp_wifi_internal_tx 返回非 OK 的累计次数
volatile uint32_t ncm_tx_last_err;   // 最近一次的错误码
void ncm_usb_recycle(void);          // 实现都在下面，转发任务里要先用
extern volatile uint32_t g_sub;      // 唯一位置编号(定义在 main.cpp)。这是 C 文件，
#define MARK(n) (g_sub = (n))        // include 不进 usbncm.h 那个 C++ 头，只能本地声明
uint32_t ncm_link_ready(void);

//====================== USB -> WiFi ======================
// 🔴 2026-09-14 改成流水线。原来在这里【同步】调 esp_wifi_internal_tx，发完才 renew ——
// 收一帧、发一帧、才收下一帧，每帧的 USB 接收和 WiFi 发送延迟串行累加。实测这条路只有
// 1.79 Mbit/s，而 ESP32 独占发 WiFi 有 8.60，差 4.8 倍。注意这个方向丢帧只有 1.3%，
// 丢包率根本量不到串行延迟 —— 我据此判过它"干净"，是错的。
bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
  if (size && s_bridge_on && size <= NCM_FRAME_MAX && s_u2w) {
    ncm_rx_frames++;
    ncm_rx_bytes += size;
    // ARP(0x0806) 的 sender protocol address 在 28，IPv4(0x0800) 的源地址在 26。
    // 跳过 0.0.0.0（DHCP discover 阶段）和 169.254（USB gadget 的链路本地，不是它的真地址）。
    if (size >= 34) {
      const uint8_t *ip = NULL;
      if (src[12] == 0x08 && src[13] == 0x06 && size >= 32) ip = src + 28;
      else if (src[12] == 0x08 && src[13] == 0x00) ip = src + 26;
      if (ip && !(ip[0] == 0 && ip[1] == 0) && !(ip[0] == 169 && ip[1] == 254)) {
        ncm_host_ip = ((uint32_t)ip[0] << 24) | ((uint32_t)ip[1] << 16)
                    | ((uint32_t)ip[2] << 8) | ip[3];
        // 🔴 这里原来用 ESP_LOGW，而 platformio.ini 没设 CORE_DEBUG_LEVEL（默认 0），
        // 整条日志被编译期掐掉、永远打不出来。改成变量，由 N 命令打印 —— 变量还有个
        // 好处：日志会被刷掉，变量不会，事后随时能问。
        if (!ncm_host_ip_first_ms)
          ncm_host_ip_first_ms = (uint32_t)(esp_timer_get_time() / 1000);
        ncm_host_ip_ms = (uint32_t)(esp_timer_get_time() / 1000);
      }
    }
    static ncm_frame_t tmp;              // usbd 任务独占，入队是值拷贝
    tmp.len = size;
    memcpy(tmp.data, src, size);
    // 🔴 队列满时【等一下】而不是立刻丢：立刻丢等于对上游说"继续给我"，而下游根本发不动，
    // WiFi 驱动的发送缓冲被一路灌爆，esp_wifi_internal_tx 返回 0x101(ESP_ERR_NO_MEM)，
    // 链路就此断死且只能断电 —— 2026-09-18 实测双向同时压 10 秒必现。等待即背压。
    // ⚠️ 必须带超时：无限等会把 usbd 任务焊死，那是比丢帧严重得多的死法。
    if (xQueueSend(s_u2w, &tmp, pdMS_TO_TICKS(NCM_U2W_WAIT_MS)) != pdTRUE) ncm_drop_u2wq++;
  }
  tud_network_recv_renew();              // 立刻放行，不等 WiFi
  return true;
}

// 🔴 换 WiFi 模式（开关热点、复位射频）时驱动在拆接口，这时调 esp_wifi_internal_tx 会在驱动里空指针崩
// （09-24 关热点时网页正在传画面，崩在 ieee80211_output_do）。换模式的一方持有它，发送一方每帧拿一次。
static SemaphoreHandle_t s_tx_mu;

void ncm_bridge_hold(int on) {
  if (!s_tx_mu) return;
  if (on) xSemaphoreTake(s_tx_mu, portMAX_DELAY);
  else xSemaphoreGive(s_tx_mu);
}

// USB 收到的帧在这里发去 WiFi。和 usbd 任务并行，这才是流水线的另一半。
static void ncm_u2w_task(void *arg) {
  (void)arg;
  static ncm_frame_t cur;
  for (;;) {
    if (xQueueReceive(s_u2w, &cur, portMAX_DELAY) != pdTRUE) continue;
    // WiFi 忙时重试而不是一失败就丢：原来那版没有缓冲也没有重试
    bool ok = false;
    xSemaphoreTake(s_tx_mu, portMAX_DELAY);
    for (int i = 0; i < 8 && !ok; i++) {
      esp_err_t e = esp_wifi_internal_tx(s_wifi_if, cur.data, cur.len);
      if (e == ESP_OK) ok = true;
      else { ncm_tx_retry++; ncm_tx_last_err = (uint32_t)e; vTaskDelay(1); }
    }
    xSemaphoreGive(s_tx_mu);
    if (!ok) ncm_drop_wifi++;
  }
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
  (void)arg;
  ncm_frame_t *f = (ncm_frame_t *)ref;
  if (!f) return 0;
  memcpy(dst, f->data, f->len);
  return f->len;
}

void tud_network_init_cb(void) { ncm_opened++; }
void tud_network_link_state_cb(bool state) { ncm_link_up = state ? 1 : 0; }

//====================== WiFi -> USB ======================
// WiFi 任务上下文，不能阻塞：拷一份进队列就放行，eb 必须还给驱动
static esp_err_t ncm_wifi_rx(void *buffer, uint16_t len, void *eb) {
  ncm_last_w2u_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  // 进回调的总帧数。没有它就分不清「ESP32 压根没收到」和「收到了后面丢的」——
  // 下面那个 if 不满足时是静默丢弃，drop_q 一样是 0。
  ncm_w2u_in++;
  // 只数 ICMP：背景广播流量比 ping 多一个数量级，用总帧数对不准帐。
  // 以太帧 [12..13]=ethertype(0x0800=IPv4)，IP 头 protocol 在 offset 23，1=ICMP。
  // 只数 echo request(ICMP type=8)：Looper 侧的 InEchos 也只数它，口径必须一致才能对帐。
  // 之前数的是全部 ICMP，结果 ESP32 报 69、Looper 报 45，根本不是同一种东西。
  if (len > 34) { const uint8_t *b = (const uint8_t *)buffer;
                  if (b[12] == 0x08 && b[13] == 0x00 && b[23] == 1 && b[34] == 8) ncm_icmp_in++; }
  if (s_bridge_on && len && len <= NCM_FRAME_MAX && s_w2u) {
    static ncm_frame_t tmp;              // 只在本函数内用，入队是值拷贝
    tmp.len = len;
    memcpy(tmp.data, buffer, len);
    if (xQueueSend(s_w2u, &tmp, 0) != pdTRUE) ncm_drop_q++;
  }
  if (eb) esp_wifi_internal_free_rx_buffer(eb);
  return ESP_OK;
}

// tud_network_xmit 是异步的：交出去之后 TinyUSB 才回头调 xmit_cb 取数据，
// 所以那一帧必须一直活到 can_xmit 再次为真，不能是栈上的临时变量。
// 心跳：板子判 USB 链路死活只看得到收包计数，而 WiFi 断开时网桥一帧不转，计数冻结就和链路死了
// 分不清（USB 角色守护会据此切口）。每秒插一帧，顺带把 WiFi 状态报给 netheal。由本任务发是因为它是唯一的发送者。
#define NCM_HB_MS 1000
volatile uint8_t ncm_hb_link, ncm_hb_reason;
volatile int8_t ncm_hb_ap = -1, ncm_hb_rssi;
volatile uint16_t ncm_hb_gen;
volatile uint32_t ncm_hb_sent;
volatile uint8_t ncm_hb_mode;   // 0=连路由器 1=热点

// 格式和 board_netheal.py 的 parse_hb() 一一对应，改一边必须改另一边。
static void ncm_hb_fill(ncm_frame_t *f) {
  uint8_t *d = f->data;
  memset(d, 0, 60);
  memset(d, 0xff, 6);                      // 广播：主机侧网卡 MAC 和 STA 相同，不必区分
  memcpy(d + 6, tud_network_mac_address, 6);
  d[12] = 0x88; d[13] = 0xB5;              // IEEE 本地实验以太类型，板子内核计数后直接丢
  memcpy(d + 14, "LPHB", 4);
  d[18] = 1;                               // 版本
  d[19] = ncm_hb_link;
  uint16_t g = ncm_hb_gen;
  d[20] = g >> 8; d[21] = g & 0xff;
  d[22] = (uint8_t)ncm_hb_ap; d[23] = (uint8_t)ncm_hb_rssi; d[24] = ncm_hb_reason;
  uint32_t up = (uint32_t)(esp_timer_get_time() / 1000);
  d[25] = up >> 24; d[26] = up >> 16; d[27] = up >> 8; d[28] = up;
  d[29] = ncm_hb_mode;
  f->len = 60;
}

static void ncm_bridge_task(void *arg) {
  (void)arg;
  static ncm_frame_t cur, hb;
  uint32_t consec_fail = 0;
  TickType_t hb_at = xTaskGetTickCount();
  for (;;) {
    TickType_t now = xTaskGetTickCount(), due = hb_at + pdMS_TO_TICKS(NCM_HB_MS);
    if ((int32_t)(now - due) >= 0) {
      hb_at = now;
      // xmit_cb 是在 tud_network_xmit 里同步拷走的，hb 用完即可复用。NTB 满就跳过这一拍。
      if (ncm_link_ready() && tud_network_can_xmit(60)) { ncm_hb_fill(&hb); tud_network_xmit(&hb, 0); ncm_hb_sent++; }
      continue;
    }
    if (xQueueReceive(s_w2u, &cur, due - now) != pdTRUE) continue;

    // 链路没起来(主机还没 SET_INTERFACE alt=1，或正处在断开重连的窗口里)：自旋毫无意义，
    // 实测空转 67 万次。更要命的是这些失败会一路累进 consec_fail，让重连本身触发下一次
    // 重连 —— 自激。这里直接丢帧休眠，并把计数清掉。
    if (!ncm_link_ready()) {
      ncm_drop_tx++;
      consec_fail = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    // 先忙等再睡：vTaskDelay(1) 是一个完整 tick，每帧都付 1ms 的话 TCP 的 ACK 会被
    // 拖慢，而 NCM 本来能把多个帧打进同一个 NTB —— 帧之间一睡就打不成包了。
    uint32_t spin = 0;
    while (!tud_network_can_xmit(cur.len)) {
      if (!ncm_link_ready()) break;   // 自旋期间链路掉了就别空转
      if (++spin <= 400) { taskYIELD(); continue; }
      if (spin > 400 + 20) break;        // 忙等不成再睡，20ms 仍发不出去就丢
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (spin > 400 + 20) {
      ncm_drop_tx++;
      // 真卡死(主机不再取 IN 端点)才断开重连。⚠️ 判据必须是【连续】失败次数，不能用
      // "距上次成功多久" —— 那个时间把空闲期也算进去，突发流量的第一帧稍堵一下就误判，
      // 13 次误重连把吞吐从 1.42 打到 0.22 Mbit/s。100 次 ≈ 连续 2 秒一帧都发不出去。
      // ⚠️ 这里曾经调 ncm_usb_recycle() 断开重连，实测 55 次之后链路彻底起不来：重连窗口
      // 里积压的帧又在下面空转 400 次/帧(计到 214 万)，越救越糟。链路真卡死由 Looper 侧的
      // 角色轮询兜底 —— ping 不通网关就转 device 再转回 host，等于重新枚举。这里只统计。
      if (++consec_fail >= 100) { ncm_stalls++; consec_fail = 0; }
      continue;
    }
    tud_network_xmit(&cur, 0);
    if (cur.len > 34 && cur.data[12] == 0x08 && cur.data[13] == 0x00 &&
        cur.data[23] == 1 && cur.data[34] == 8)
      ncm_icmp_out++;
    ncm_tx_frames++;
    ncm_tx_bytes += cur.len;
    consec_fail = 0;
  }
}

//====================== 挂进 TinyUSB ======================
static const usbd_class_driver_t s_ncm_driver = {
#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
    .name = "NCM",
#endif
    .init = netd_init,
    .reset = netd_reset,
    .open = netd_open,
    .control_xfer_cb = netd_control_xfer_cb,
    .xfer_cb = netd_xfer_cb,
    .sof = NULL,
};

// weak 符号，覆盖掉库里的空实现
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = 1;
  return &s_ncm_driver;
}

static uint16_t ncm_load_descriptor(uint8_t *dst, uint8_t *itf) {
  // MAC 在这里读而不是在注册时读：注册跑在全局构造阶段，那时候取 MAC 太早。
  // NCM 网卡的 MAC 直接用 WiFi STA 的 —— 二层桥接下 AP 只认一个 MAC，各用各的
  // 就成了 802.11 的四地址(WDS)场景，普通路由器根本不转发。
  esp_read_mac(tud_network_mac_address, ESP_MAC_WIFI_STA);
  for (int i = 0; i < 6; i++)
    sprintf(s_mac_str + i * 2, "%02X", tud_network_mac_address[i]);
  s_mac_str[12] = 0;
  uint8_t str_itf = tinyusb_add_string_descriptor("ESP32 NCM");
  uint8_t str_mac = tinyusb_add_string_descriptor(s_mac_str);
  uint8_t ep_notif = tinyusb_get_free_in_endpoint();
  uint8_t ep_data = tinyusb_get_free_duplex_endpoint();
  uint8_t d[TUD_CDC_NCM_DESC_LEN] = {TUD_CDC_NCM_DESCRIPTOR(
      *itf, str_itf, str_mac, (uint8_t)(0x80 | ep_notif), 64,
      ep_data, (uint8_t)(0x80 | ep_data), 64, CFG_TUD_NET_MTU)};
  *itf += 2;
  memcpy(dst, d, sizeof(d));
  return sizeof(d);
}

// 必须在 USB.begin() 之前调：tinyusb_enable_interface 一旦看到 tinyusb_is_initialized
// 就直接拒绝。CDC 关掉之后 core 不会自己起 USB，所以 setup 里调来得及。
volatile int ncm_reg_rc = -999;
void ncm_usb_register(void) {
  ncm_reg_rc = (int)tinyusb_enable_interface(USB_INTERFACE_CUSTOM,
                                             TUD_CDC_NCM_DESC_LEN, ncm_load_descriptor);
}

// WiFi 连上之后再调：接管收包并起桥接任务
volatile uint32_t ncm_ip_reclaims = 0;

// 桥接期间 ESP32 必须不占 IP，否则 AP 看到"一个 MAC 两个 IP"，防 IP 欺骗会把转发的包
// 整批丢掉(单向不通，而 ESP32 这侧毫无察觉)。抽成函数是因为它得被反复调用 —— 见下。
void ncm_clear_sta_ip(void) {
  esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (!sta) return;
  esp_netif_dhcpc_stop(sta);
  esp_netif_ip_info_t zero;
  memset(&zero, 0, sizeof(zero));
  esp_netif_set_ip_info(sta, &zero);
}

// 只为验证上面那条周期回收：人为塞一个 IP 进去，下一轮 ncmAutoTick 应当发现并清掉。
// 留着当诊断用 —— 这个 bug 会周期性复发，手上得有个能随时复现它的开关。
void ncm_set_test_ip(void) {
  esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (!sta) return;
  esp_netif_ip_info_t fake;
  memset(&fake, 0, sizeof(fake));
  esp_netif_str_to_ip4("192.0.2.222", &fake.ip);   // 文档专用地址段，不会和真网络撞
  esp_netif_set_ip_info(sta, &fake);
}

uint32_t ncm_sta_has_ip(void) {
  esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_netif_ip_info_t ip;
  if (sta && esp_netif_get_ip_info(sta, &ip) == ESP_OK) return ip.ip.addr != 0 ? 1 : 0;
  return 0;
}

// 转发任务在 USB 一枚举就要起来（心跳靠它发），不能等 WiFi 连上开桥：开机连不上 WiFi 时板子照样要看到心跳。
// ⚠️ 任务只能建一次。原来每次 start 都建一个新的 ncm_br，多个任务抢同一个队列，帧会乱序，还白吃栈。
bool ncm_tasks_start(void) {
  static bool s_tasks_up = false;
  if (s_tasks_up) return true;
  if (!s_w2u) s_w2u = xQueueCreate(NCM_QUEUE_LEN, sizeof(ncm_frame_t));
  if (!s_u2w) s_u2w = xQueueCreate(U2W_QUEUE_LEN, sizeof(ncm_frame_t));
  if (!s_tx_mu) s_tx_mu = xSemaphoreCreateMutex();
  if (!s_w2u || !s_u2w || !s_tx_mu) { ESP_LOGE(NCM_TAG, "queue alloc failed"); return false; }
  xTaskCreatePinnedToCore(ncm_bridge_task, "ncm_br", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(ncm_u2w_task, "ncm_u2w", 4096, NULL, 5, NULL, 0);
  s_tasks_up = true;
  return true;
}

void ncm_bridge_start(void) {
  if (s_bridge_on) return;
  if (!ncm_tasks_start()) return;
  // 🔴 桥接期间必须把自己的 IP 交出去。二层桥接下转发出去的帧源 MAC 是 STA 的，
  // 而源 IP 是主机的 —— 如果 ESP32 自己还占着另一个 IP，AP 就看到"一个 MAC 两个
  // IP"，带防 IP 欺骗的企业 AP 会把转发的包整批丢掉。
  // 2026-09-14 实测：主机发 3540 包只有 399 到达(丢 88.7%)，而 ESP32 自己发同样
  // 速率零丢包 —— 两者唯一的差别就是源 IP。
  ncm_clear_sta_ip();
  esp_wifi_internal_reg_rxcb(s_wifi_if, ncm_wifi_rx);
  s_bridge_on = true;
}

void ncm_bridge_stop(void) {
  if (!s_bridge_on) return;
  esp_wifi_internal_reg_rxcb(s_wifi_if, NULL);
  s_bridge_on = false;
  if (s_wifi_if == WIFI_IF_AP) return;   // 热点模式没有 STA 连接可重建，断开重连只会打断兜底扫描
  // 🔴 光把收包回调注销掉不够：lwIP 那条收包路径不会自己接回去，实测关桥之后
  // WiFi 一直停在 status=0(未连接)，等多久都不恢复。断开重连让整条链路重新绑定。
  // 2026-09-14 看门狗第一次真触发时撞上的。
  esp_wifi_disconnect();
  esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (sta) esp_netif_dhcpc_start(sta);   // 把自己的 IP 要回来，UDP 命令口才能用
  esp_wifi_connect();
}

int ncm_bridge_running(void) { return s_bridge_on ? 1 : 0; }

// 切网桥接口。旧接口的收包回调必须摘掉，否则 STA 恢复前的残帧会继续灌进 USB。
void ncm_bridge_set_ap(int ap) {
  wifi_interface_t want = ap ? WIFI_IF_AP : WIFI_IF_STA;
  if (want == s_wifi_if) return;
  if (s_bridge_on) esp_wifi_internal_reg_rxcb(s_wifi_if, NULL);
  s_wifi_if = want;
  ncm_hb_mode = ap ? 1 : 0;
  if (s_bridge_on) esp_wifi_internal_reg_rxcb(s_wifi_if, ncm_wifi_rx);
}

volatile uint32_t ncm_rearm_cnt;
void ncm_bridge_rearm(void) {
  // 🔴 ncm_bridge_start() 开头就 `if (s_bridge_on) return;`，所以 rxcb 一辈子只注册一次。
  // 而 IDF 在 STA 断开/重新关联时会把这个内部回调丢掉 —— 重连之后网桥显示 on、射频显示
  // 已关联，却一个包都收不进来（2026-09-22 实测 桥接=on 而 WiFi->USB 恒为 0，板子的 DHCP
  // 请求出得去回不来，表现就是"连上了但网页打不开"）。关联事件到来时必须重挂一次。
  if (!s_bridge_on) return;
  esp_wifi_internal_reg_rxcb(s_wifi_if, ncm_wifi_rx);
  ncm_rearm_cnt++;
}

// ⚠️ ncm_link_up 只在 tud_network_link_state_cb 里被设，而 USB 物理断开时那个回调不一定
// 来 —— 它会永远停在 1。所有"链路还在吗"的判断都必须走这里，配上 tud_mounted() 才准。
uint32_t ncm_link_ready(void) { return (ncm_link_up && tud_mounted()) ? 1 : 0; }

// USB 段纯速率：绕过 WiFi 直接往 USB 灌固定大小的帧，测这一段自己能跑多快。
// 端到端速率量的是整条链路，分不出瓶颈在哪一段 —— 这里把 USB 单独隔离出来。
// 发的是无效以太网帧，Looper 收到会直接丢弃，不影响它。
void ncm_usb_speedtest(uint32_t run_ms, uint32_t *out_kbps, uint32_t *out_frames, uint32_t *out_busy) {
  static ncm_frame_t f;
  f.len = 1400;
  memset(f.data, 0xA5, f.len);
  uint32_t t0 = millis(), sent = 0, busy = 0;
  while (millis() - t0 < run_ms) {
    if (!ncm_link_ready()) break;
    if (tud_network_can_xmit(f.len)) { tud_network_xmit(&f, 0); sent++; }
    else { busy++; taskYIELD(); }
  }
  uint32_t ms = millis() - t0; if (!ms) ms = 1;
  *out_kbps = (uint32_t)(((uint64_t)sent * f.len * 1000ULL) / ms / 1024ULL);
  *out_frames = sent;
  *out_busy = busy;
}

// 区分"ESP32 的 USB 压根没启动"和"启动了但主机不来枚举" —— 只看 链路/主机已开 这两个
// 计数分不开，它们在两种情况下都是 0。
uint32_t ncm_usb_state(void) {
  uint32_t v = 0;
  MARK(411);
  if (tud_connected()) v |= 1;
  MARK(412);
  if (tud_mounted())   v |= 2;
  MARK(413);
  if (tud_suspended()) v |= 4;
  MARK(414);
  return v;
}

void ncm_usb_recycle(void) {
  tud_disconnect();
  vTaskDelay(pdMS_TO_TICKS(200));
  tud_connect();
}
const char *ncm_mac_string(void) { return s_mac_str; }

#endif  // CONFIG_TINYUSB_ENABLED
