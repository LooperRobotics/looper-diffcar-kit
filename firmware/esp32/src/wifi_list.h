// WiFi 候选列表的编辑逻辑（存闪存、串口改）。抽成纯函数为了能在电脑上测：test_wifi_list.cpp。
// 串口协议走十六进制：SSID 可以是中文、密码可以有空格，而串口那一路只收可打印 ASCII。
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define WL_MAX 5
#define WL_SSID_MAX 32   // 802.11 上限
#define WL_PASS_MAX 63   // WPA2 口令上限；64 位十六进制 PSK 不支持

typedef struct { char ssid[WL_SSID_MAX + 1]; char pass[WL_PASS_MAX + 1]; } WlEntry;
typedef struct { WlEntry e[WL_MAX]; uint8_t n; } WlList;

typedef enum { WL_OK, WL_EFULL, WL_EHEX, WL_ESSID, WL_EPASS, WL_EDUP, WL_EKEEP, WL_ECOUNT } WlErr;

static inline const char *wl_err_name(WlErr e) {
  switch (e) {
    case WL_OK: return "ok";
    case WL_EFULL: return "full";      // 超过 WL_MAX 条
    case WL_EHEX: return "hex";        // 编码坏了（多半是串口传输出错）
    case WL_ESSID: return "ssid";      // 空或超过 32 字节
    case WL_EPASS: return "pass";      // 不是 0 或 8~63 个可打印字符
    case WL_EDUP: return "dup";        // 同名 SSID 出现两次
    case WL_EKEEP: return "keep";      // 要求沿用旧密码，但旧列表里没有这个 SSID
    case WL_ECOUNT: return "count";    // 提交时条数对不上 = 中间有行丢了
  }
  return "?";
}

static inline int wl_nib(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// 解出的字节数；奇数长度、非法字符、超长、含 \0 都返回 -1
static inline int wl_unhex(const char *h, char *out, int max) {
  int len = (int)strlen(h);
  if (len % 2 || len / 2 > max) return -1;
  for (int i = 0; i < len / 2; i++) {
    int a = wl_nib(h[2 * i]), b = wl_nib(h[2 * i + 1]);
    if (a < 0 || b < 0 || (a | b) == 0) return -1;
    out[i] = (char)(a << 4 | b);
  }
  out[len / 2] = 0;
  return len / 2;
}

static inline void wl_hex(const char *s, char *out) {
  static const char d[] = "0123456789abcdef";
  for (; *s; s++) { *out++ = d[(uint8_t)*s >> 4]; *out++ = d[(uint8_t)*s & 15]; }
  *out = 0;
}

static inline int wl_find(const WlList *l, const char *ssid) {
  for (int i = 0; i < l->n; i++) if (!strcmp(l->e[i].ssid, ssid)) return i;
  return -1;
}

// WPA2 口令：8~63 个可打印 ASCII
static inline WlErr wl_pass_parse(const char *hex, char *out) {
  int pl = wl_unhex(hex, out, WL_PASS_MAX);
  if (pl < 0) return WL_EHEX;
  if (pl < 8) return WL_EPASS;
  for (int i = 0; i < pl; i++) if (out[i] < 0x20 || out[i] > 0x7e) return WL_EPASS;
  return WL_OK;
}

// 往暂存表追加一条。pass_tok：十六进制口令 / "-" 开放网络 / "*" 沿用 cur 里同名那条的密码
// （网页不回显密码，用户只改顺序时没法再填一遍）。
static inline WlErr wl_stage_add(WlList *st, const WlList *cur, const char *ssid_hex, const char *pass_tok) {
  if (st->n >= WL_MAX) return WL_EFULL;
  WlEntry e;
  int sl = wl_unhex(ssid_hex, e.ssid, WL_SSID_MAX);
  if (sl < 0) return WL_EHEX;
  if (sl == 0) return WL_ESSID;
  if (wl_find(st, e.ssid) >= 0) return WL_EDUP;
  if (!strcmp(pass_tok, "*")) {
    int k = wl_find(cur, e.ssid);
    if (k < 0) return WL_EKEEP;
    strcpy(e.pass, cur->e[k].pass);
  } else if (!strcmp(pass_tok, "-")) {
    e.pass[0] = 0;
  } else {
    WlErr pe = wl_pass_parse(pass_tok, e.pass);
    if (pe != WL_OK) return pe;
  }
  st->e[st->n++] = e;
  return WL_OK;
}

// 提交前的最后一道：必须非空（空表 = 车再也连不上任何网），且条数等于网页发的条数。
static inline WlErr wl_commit_check(const WlList *st, int expect) {
  if (st->n == 0 || st->n != expect) return WL_ECOUNT;
  return WL_OK;
}

// 车的热点名和密码（Nwh）。热点必须带密码：开放热点谁都能连上来开车。
static inline WlErr wl_ap_parse(const char *ssid_hex, const char *pass_hex, WlEntry *out) {
  int sl = wl_unhex(ssid_hex, out->ssid, WL_SSID_MAX);
  if (sl < 0) return WL_EHEX;
  if (sl == 0) return WL_ESSID;
  return wl_pass_parse(pass_hex, out->pass);
}
