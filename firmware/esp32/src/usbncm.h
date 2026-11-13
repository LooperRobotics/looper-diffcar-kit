// NCM 网卡桥接的 C++ 侧接口，实现在 tinyusb_net/ncm_glue.c
#pragma once
#include <stdint.h>
extern "C" {
void ncm_usb_register(void);
extern volatile int ncm_reg_rc;
void ncm_bridge_start(void);
void ncm_bridge_stop(void);
int ncm_bridge_running(void);
const char *ncm_mac_string(void);
uint32_t ncm_link_ready(void);   // 链路真的还在吗（ncm_link_up 会残留，别直接读）
uint32_t ncm_usb_state(void);
void ncm_usb_speedtest(uint32_t run_ms, uint32_t *kbps, uint32_t *frames, uint32_t *busy);   // bit0=connected bit1=mounted bit2=suspended
void ncm_usb_recycle(void);     // 断开 200ms 再连，逼主机重新枚举，不用整机复位
extern volatile uint32_t ncm_rx_frames, ncm_rx_bytes, ncm_tx_frames, ncm_tx_bytes;
extern volatile uint32_t ncm_w2u_in;
extern volatile uint32_t ncm_xmit_reject;
extern volatile uint32_t ncm_icmp_in, ncm_icmp_out;
extern volatile uint32_t ncm_host_ip, ncm_host_ip_ms, ncm_host_ip_first_ms;  // 上位机的 IP（从它发的 ARP/IP 包里抓）
extern volatile uint32_t ncm_tx_recover, ncm_dg_sent, ncm_ntb_pad, ncm_xmit_max_ms;
extern volatile uint32_t ncm_tx_restart;
extern volatile uint32_t ncm_fail_ep_state;
int  ncm_tx_recover_if_lost(uint32_t now_ms);   // 返回 NCM_HEAL_*，RECYCLE 由调用方执行
extern volatile uint32_t ncm_drop_q, ncm_drop_tx, ncm_link_up, ncm_opened;
extern volatile uint32_t ncm_stalls;
extern volatile uint32_t ncm_last_w2u_ms, ncm_tx_retry, ncm_tx_last_err;
extern volatile uint32_t ncm_nx_alt, ncm_nx_count, ncm_nx_size;
extern volatile uint32_t ncm_drop_wifi;
extern volatile uint32_t ncm_drop_u2wq;
void ncm_clear_sta_ip(void);     // 桥接期间把 STA 的 IP 交出去
uint32_t ncm_sta_has_ip(void);
void ncm_set_test_ip(void);     // 仅测试：人为塞 IP，验证周期回收   // STA 当前是否占着 IP（桥接时必须为 0）
extern volatile uint32_t ncm_ip_reclaims;
// NCM 发送路径的内部状态。transferring 卡在 1 = 上一个 NTB 主机永远没取走，
// 之后 ncm_start_tx() 全部空转，偏移顶到 ntb_in_size 就永远"满"。
extern volatile uint32_t ncm_xfer_submit, ncm_xfer_done, ncm_xfer_fail;
extern volatile uint32_t ncm_enum_gen;   // 总线复位（主机枚举）次数
extern volatile uint32_t ncm_ep_unstick;
extern volatile uint32_t ncm_ntb_nopad;
void ncm_dbg_force_stuck(void);  // 仅测试：人为制造死锁，验证看门狗
uint32_t ncm_dbg_transferring(void);
uint8_t ncm_dbg_ep_in(void);
bool ncm_in_rearm(void);   // 卡死第一级兜底：续传在途 IN 传输没发出去的部分
extern volatile uint32_t dcdx_unknown15, dcdx_nospace, dcdx_cmpl_notempty, dcdx_in_timeout, dcdx_rearm_ok, dcdx_rearm_fail;
uint32_t ncm_dbg_offset(void);
uint32_t ncm_dbg_dgcount(void);
uint32_t ncm_dbg_ntbsize(void);
uint32_t ncm_dbg_alt(void);
extern volatile uint32_t g_sub;   // 唯一位置编号，见 main.cpp 的说明
#define MARK(n) (g_sub = (n))

bool ncm_tasks_start(void);             // USB 枚举后就调：心跳不等 WiFi
// 心跳里报给板子的 WiFi 状态；gen 每次关联 +1，板子据此知道 ESP32 重新连过（可能换了网）
extern volatile uint8_t ncm_hb_link, ncm_hb_reason;
extern volatile int8_t ncm_hb_ap, ncm_hb_rssi;
extern volatile uint16_t ncm_hb_gen;
extern volatile uint32_t ncm_hb_sent;
extern volatile uint8_t ncm_hb_mode;
void ncm_bridge_hold(int on);           // 1 = 暂停网桥往 WiFi 发（换 WiFi 模式前），0 = 恢复
void ncm_bridge_set_ap(int ap);        // 1 = 网桥改挂热点接口（热点模式），0 = 挂回 STA
void ncm_bridge_rearm(void);            // STA 重新关联后必须重挂 WiFi 收包回调
extern volatile uint32_t ncm_rearm_cnt;
}
