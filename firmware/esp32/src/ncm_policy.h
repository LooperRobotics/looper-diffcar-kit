// NCM 网桥的自动策略。决策与硬件解耦，好让 test_ncm_policy.cpp 在电脑上直接跑 ——
// 这段逻辑锁死过板子两次，不能再靠上板试。
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint32_t now;
  bool linkUp;           // 射频层：还关联着 AP 吗(esp_wifi_sta_get_ap_info==ESP_OK)
  bool usbReady;         // ncm_link_ready()
  bool bridgeOn;         // ncm_bridge_running()
  bool autoOn;           // g_ncmAuto
  bool staHasIp;         // ncm_sta_has_ip()
  uint32_t w2uIn;        // ncm_w2u_in：WiFi 侧收到的总帧数(含广播)
  uint32_t usbRxFrames;  // ncm_rx_frames：USB 侧进来的帧数
  bool apMode;           // ESP32 自己开着热点、网桥挂在 AP 接口上：没有「关联着上游」这回事
} NcmIn;

typedef struct {
  uint32_t onAt, lastRetry, coolUntil, rxAtOn, lastIpChk, w2uLast, w2uAt, usbLast, usbAt;
} NcmSt;

typedef struct { bool stopBridge, startBridge, clearIp; } NcmAct;

#define NCM_BOOT_GUARD_MS  15000
#define NCM_START_GUARD_MS  8000
#define NCM_RETRY_MS       10000
#define NCM_IPCHK_MS        2000
// 🔴 30000 在只连一台设备的手机热点上是灾难：那种网络 WiFi 侧本来就几乎收不到东西
// （实验室网有三十多台设备的广播，热点没有），而板子在不停发 DHCP 让 USB 侧一直活跃，
// 于是"必须 USB 侧还在发"这个保护条件不但没挡住，反而正好凑齐触发条件。
// 2026-09-22 实测：网桥开启后 26~44 秒必被判成"射频假活"，stopBridge 内含 disconnect，
// 表现成 reason=8 掉线，然后重连、再来一遍 —— 累计断开占比 61.9%。
// 放宽到 120 秒，并且要求 USB 侧确实发过足够多的帧（几次 DHCP 重试不算数）。
#define NCM_RXDEAD_MS     120000
#define NCM_RXDEAD_MIN_TX     30   // 这段时间里 USB 侧至少发过这么多帧才算"我们真在用它"
#define NCM_IDLE_MS       120000
#define NCM_COOL_MS        60000

static inline void ncmPolicy(const NcmIn *in, NcmSt *st, NcmAct *act) {
  act->stopBridge = act->startBridge = act->clearIp = false;
  const uint32_t now = in->now;

  // 射频假活：关联还在、却收不到任何东西。⚠️ 判据必须同时要求"主机确实还在发" ——
  // 只看 WiFi 侧静默会在安静网络里误判，把正常的桥每 30 秒断一次，比它要修的 bug 还频繁。
  if (in->w2uIn != st->w2uLast) { st->w2uLast = in->w2uIn; st->w2uAt = now; }
  if (in->usbRxFrames != st->usbLast) { st->usbLast = in->usbRxFrames; st->usbAt = now; }
  if (!st->w2uAt) st->w2uAt = now;
  if (!st->usbAt) st->usbAt = now;
  // "我们真在用它"要用**发了多少帧**来衡量，不能只看"最近发过"：板子每秒重试一次 DHCP
  // 也会让 usbAt 一直刷新，那不足以证明链路该活着。
  // 热点模式不判：没人连着时 WiFi 侧本来就一帧不收，按这条会每 10 s 停一次桥
  bool rxDead = !in->apMode && in->bridgeOn && in->linkUp && (now - st->w2uAt > NCM_RXDEAD_MS)
                             && (now - st->usbAt < NCM_RXDEAD_MS)
                             && (in->usbRxFrames - st->rxAtOn > NCM_RXDEAD_MIN_TX);

  // ① 只救射频假活。链路断了不归这里管：重连只归 WiFi 状态机(wtPickAp)，这里再断开重连
  //    会打断它的扫描 —— 09-23 两次关热点的「扫描结果丢失」都正好落在这里的 10 s 重试上。
  //    断开期间桥照开着，重新关联后收包回调由 ncmAutoTick 周期重挂(ncm_bridge_rearm)。
  // 🔴 linkUp 必须问射频层，不能用 WiFi.status()：桥接时 STA 没有 IP，120s 后 LOST_IP 会把
  //    status 永久打成 WL_IDLE —— 旧判据据此每 122.6s 自断一次，那是当时全部丢包的来源。
  if (rxDead && now > NCM_BOOT_GUARD_MS && now - st->lastRetry > NCM_RETRY_MS) {
    st->lastRetry = now;
    st->w2uAt = now; st->usbAt = now;   // 救过就重新计时，否则 rxDead 每 10s 连锁触发
    act->stopBridge = true; st->onAt = 0;
    return;
  }

  // ② 开桥。这里同样不能用 status：桥停掉后 status 若停在 WL_IDLE，桥就再也开不起来。
  // 热点模式同样要开：开机时热点先于 USB 就绪，桥被 ④ 撤掉之后只能靠这里重开（STA 永远不会关联）
  if (in->autoOn && !in->bridgeOn && in->usbReady && (in->linkUp || in->apMode) &&
      now > NCM_START_GUARD_MS && now > st->coolUntil) {
    act->startBridge = true;
    st->onAt = now; st->rxAtOn = in->usbRxFrames; st->w2uAt = now; st->usbAt = now;
    return;
  }

  // ③ 桥在跑时 ESP32 绝不能占着 IP（重连会让 DHCP 自己把 IP 装回来）。查询要拿 netif 锁，必须节流。
  // 热点模式下网桥挂在 AP 接口上，STA 这边的地址是切路由器前试连（Nwt）要的，不能清
  if (in->bridgeOn && !in->apMode && now - st->lastIpChk > NCM_IPCHK_MS) {
    st->lastIpChk = now;
    if (in->staHasIp) act->clearIp = true;
  }

  // ④ 主机把 USB 链路撤了(拔线/重启/切回 device)：退桥，把自己的网络要回来
  if (in->bridgeOn && !in->usbReady) { act->stopBridge = true; st->onAt = 0; return; }

  // ⑤ 链路在、但主机两分钟一帧没发过 = 它那头没配好。冷却是必须的，否则退桥→立刻重开死循环。
  if (st->onAt && in->bridgeOn && now - st->onAt > NCM_IDLE_MS && in->usbRxFrames == st->rxAtOn) {
    act->stopBridge = true; st->onAt = 0; st->coolUntil = now + NCM_COOL_MS;
  }
}
