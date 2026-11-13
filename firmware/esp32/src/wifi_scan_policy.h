// 怎么读 WiFi.scanComplete() 的返回值。抽成纯函数是为了能在电脑上测 ——
// 这条路径只能靠现场日志验证，上板一轮十几分钟。
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum { WSCAN_RUNNING, WSCAN_TIMEOUT, WSCAN_EMPTY, WSCAN_GOT, WSCAN_ABORTED, WSCAN_LOST } WScanVerdict;

// 🔴 n==0 有两种来源，混起来会让板子再也挑不回 AP：
//   ① 真的一个都没扫到（驱动没就绪时会这样）
//   ② esp_wifi_scan_stop() 补发的旧 SCAN_DONE 把库的 _scanStarted 清了，新扫描其实还在跑
// 2026-09-22 现场：读到 0 的 2ms 后事件报"找到28个"，而实验室的 AP 就在 −42dBm。
// 分辨靠时间：13 个信道 × 150ms 的扫描不可能这么快结束。
#define WSCAN_MIN_MS 1500
#define WSCAN_RC_RUNNING (-1)

// done_n：本次扫描开始后 SCAN_DONE 事件报的个数，没来过就是 -1。
static inline WScanVerdict wscan_verdict(int n, uint32_t since_start_ms, uint32_t timeout_ms, int done_n) {
  if (n > 0) return WSCAN_GOT;
  // 事件说扫到了、读出来却是 0：结果被别人取走了（09-23 关热点后，多半是库在 201 后自动重连），
  // 不是周围没 AP，不能扣空扫描额度。补发的「找到 0 个」done_n 就是 0，不会落到这里。
  if (n == 0 && done_n > 0) return WSCAN_LOST;
  // 🔴 -2 【不】说明周围没有 AP，不能和真空扫描共用三次额度（2026-09-22 掉线后三轮就退盲连）。
  // 🔴 驱动交回结果之前也不能起新扫描：新扫描清掉驱动的列表，而事件任务正在取它，WiFi 任务
  // 崩在 wifi_get_ap_list_process（2026-09-23，库超时 3 s + 重试 1.5 s 正好撞上 4.6 s 的扫描）。
  // 所以 -2 和 -1 一样等到总超时：刚初始化的驱动实测一次要 4.6~5.8 s，而库 3 s 就报 -2。
  if (n < 0 && n != WSCAN_RC_RUNNING)
    return since_start_ms < timeout_ms ? WSCAN_RUNNING : WSCAN_ABORTED;
  bool running = (n == WSCAN_RC_RUNNING) || (n == 0 && since_start_ms < WSCAN_MIN_MS);
  if (!running) return WSCAN_EMPTY;
  return since_start_ms >= timeout_ms ? WSCAN_TIMEOUT : WSCAN_RUNNING;
}
