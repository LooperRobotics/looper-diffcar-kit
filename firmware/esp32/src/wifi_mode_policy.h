// 网络模式（连路由器 / 车的热点）的决策。抽成纯函数好在电脑上测：test_wifi_mode.cpp。
#pragma once
#include <stdint.h>
#include <stdbool.h>

enum { WM_NONE, WM_AP_PERSIST, WM_AP_STOP };

// why：0 没开热点 / 1 限时测试 / 2 热点模式 / 3 兜底。每圈最多做一件事，优先级从上到下。
static inline int wm_tick(uint8_t pref, uint8_t why, bool mode_due, uint8_t want, bool mode_pending, bool expired) {
  if (mode_due) return want ? WM_AP_PERSIST : (why >= 2 ? WM_AP_STOP : WM_NONE);   // 限时测试到点自己会回来
  if (why && expired) return pref ? WM_AP_PERSIST : WM_AP_STOP;
  if (pref && !why && !mode_pending) return WM_AP_PERSIST;   // 热点模式下热点被关（Napx、开机）就重开
  return WM_NONE;
}

// 路由器模式下连不上多久开兜底热点。热点模式不需要兜底。
static inline bool wm_fallback_due(uint8_t pref, int32_t down_ms, int32_t limit_ms) {
  return !pref && down_ms > limit_ms;
}

// 候选表为空（没编进 wifi_creds.h、网页也没加过）只能用热点：路由器状态机连盲连对象都没有。
static inline uint8_t wm_effective_mode(uint8_t pref, uint8_t list_n) { return list_n ? pref : 1; }

// 「开机 10 秒内断电」计数。只数上电复位：看门狗、OTA、软件重启不算。返回新计数，满 3 次置 *force。
static inline uint8_t wm_pc_next(uint8_t pc, bool poweron, bool *force) {
  *force = false;
  if (!poweron) return pc;
  if (pc + 1 >= 3) { *force = true; return 0; }
  return pc + 1;
}

// 到点了没有（t 由 millis()+时长算出，跨 49 天回绕也对）
static inline bool wm_after(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

// 切到路由器前试连一个候选（Nwt）的结论，数字原样进 WIFITRY 那一行给网页
enum { WT_TRY_NONE, WT_TRY_ABSENT, WT_TRY_PASS, WT_TRY_NOIP, WT_TRY_TIMEOUT, WT_TRY_OK };
static inline uint8_t wm_try_code(uint8_t reason, bool assoc) {
  switch (reason) {
    case 201: return WT_TRY_ABSENT;                          // NO_AP_FOUND
    case 2: case 15: case 202: case 204: return WT_TRY_PASS; // 认证过期/四次握手超时/认证失败 —— 实际多半是密码错
  }
  return assoc ? WT_TRY_NOIP : WT_TRY_TIMEOUT;               // 连上了却拿不到地址 = 路由器 DHCP 不给
}
