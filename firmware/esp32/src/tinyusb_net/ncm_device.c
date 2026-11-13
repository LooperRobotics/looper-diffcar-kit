/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jacob Berg Potter
 * Copyright (c) 2020 Peter Lawrence
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if ( CFG_TUD_ENABLED && CFG_TUD_NCM )

#include "device/usbd.h"
#include "device/usbd_pvt.h"
#include "net_device.h"
#include "../ncm_tx_policy.h"

// Level where CFG_TUSB_DEBUG must be at least for this driver is logged
#ifndef CFG_TUD_NCM_LOG_LEVEL
  #define CFG_TUD_NCM_LOG_LEVEL   CFG_TUD_LOG_LEVEL
#endif

#define TU_LOG_DRV(...)   TU_LOG(CFG_TUD_NCM_LOG_LEVEL, __VA_ARGS__)

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

#define NTH16_SIGNATURE      0x484D434E
#define NDP16_SIGNATURE_NCM0 0x304D434E
#define NDP16_SIGNATURE_NCM1 0x314D434E

typedef struct TU_ATTR_PACKED
{
  uint16_t wLength;
  uint16_t bmNtbFormatsSupported;
  uint32_t dwNtbInMaxSize;
  uint16_t wNdbInDivisor;
  uint16_t wNdbInPayloadRemainder;
  uint16_t wNdbInAlignment;
  uint16_t wReserved;
  uint32_t dwNtbOutMaxSize;
  uint16_t wNdbOutDivisor;
  uint16_t wNdbOutPayloadRemainder;
  uint16_t wNdbOutAlignment;
  uint16_t wNtbOutMaxDatagrams;
} ntb_parameters_t;

typedef struct TU_ATTR_PACKED
{
  uint32_t dwSignature;
  uint16_t wHeaderLength;
  uint16_t wSequence;
  uint16_t wBlockLength;
  uint16_t wNdpIndex;
} nth16_t;

typedef struct TU_ATTR_PACKED
{
  uint16_t wDatagramIndex;
  uint16_t wDatagramLength;
} ndp16_datagram_t;

typedef struct TU_ATTR_PACKED
{
  uint32_t dwSignature;
  uint16_t wLength;
  uint16_t wNextNdpIndex;
  ndp16_datagram_t datagram[];
} ndp16_t;

typedef union TU_ATTR_PACKED {
  struct {
    nth16_t nth;
    ndp16_t ndp;
  };
  uint8_t data[CFG_TUD_NCM_IN_NTB_MAX_SIZE];
} transmit_ntb_t;

struct ecm_notify_struct
{
  tusb_control_request_t header;
  uint32_t downlink, uplink;
};

typedef struct
{
  uint8_t itf_num;      // Index number of Management Interface, +1 for Data Interface
  uint8_t itf_data_alt; // Alternate setting of Data Interface. 0 : inactive, 1 : active

  uint8_t ep_notif;
  uint8_t ep_in;
  uint8_t ep_out;

  const ndp16_t *ndp;
  uint8_t num_datagrams, current_datagram_index;

  enum {
    REPORT_SPEED,
    REPORT_CONNECTED,
    REPORT_DONE
  } report_state;
  bool report_pending;

  uint8_t  current_ntb;           // Index in transmit_ntb[] that is currently being filled with datagrams
  uint8_t  datagram_count;        // Number of datagrams in transmit_ntb[current_ntb]
  uint16_t next_datagram_offset;  // Offset in transmit_ntb[current_ntb].data to place the next datagram
  uint16_t ntb_in_size;           // Maximum size of transmitted (IN to host) NTBs; initially CFG_TUD_NCM_IN_NTB_MAX_SIZE
  uint8_t  max_datagrams_per_ntb; // Maximum number of datagrams per NTB; initially CFG_TUD_NCM_MAX_DATAGRAMS_PER_NTB

  uint16_t nth_sequence;          // Sequence number counter for transmitted NTBs

  bool transferring;

} ncm_interface_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+

CFG_TUD_MEM_SECTION CFG_TUSB_MEM_ALIGN tu_static const ntb_parameters_t ntb_parameters = {
    .wLength                 = sizeof(ntb_parameters_t),
    .bmNtbFormatsSupported   = 0x01,
    .dwNtbInMaxSize          = CFG_TUD_NCM_IN_NTB_MAX_SIZE,
    .wNdbInDivisor           = 4,
    .wNdbInPayloadRemainder  = 0,
    .wNdbInAlignment         = CFG_TUD_NCM_ALIGNMENT,
    .wReserved               = 0,
    .dwNtbOutMaxSize         = CFG_TUD_NCM_OUT_NTB_MAX_SIZE,
    .wNdbOutDivisor          = 4,
    .wNdbOutPayloadRemainder = 0,
    .wNdbOutAlignment        = CFG_TUD_NCM_ALIGNMENT,
    .wNtbOutMaxDatagrams     = 0
};

CFG_TUD_MEM_SECTION CFG_TUSB_MEM_ALIGN tu_static transmit_ntb_t transmit_ntb[2];

CFG_TUD_MEM_SECTION CFG_TUSB_MEM_ALIGN tu_static uint8_t receive_ntb[CFG_TUD_NCM_OUT_NTB_MAX_SIZE];

tu_static ncm_interface_t ncm_interface;

volatile uint32_t ncm_xfer_submit = 0, ncm_xfer_done = 0, ncm_xfer_fail = 0;
volatile uint32_t ncm_enum_gen = 0;   // 总线复位次数：自愈和看门狗按它区分「又一次 bring-up」
// 🔴 TinyUSB 的 NCM 假设 tud_network_xmit 只在 usbd 任务里调，而我们从独立的网桥任务调 ——
// 于是往 NTB 拷数据的同时，usbd 任务的完成回调可能正在换缓冲并把计数清零，拷进去的帧就
// 落到已经在发的那块上、计数也对不上，主机拆包时整批丢掉。实测这样丢了 20.9%。
static osal_mutex_def_t ncm_mutex_def;
static osal_mutex_t     ncm_mutex;
// 🔴 主循环卡死时卡点=4，也就是卡在下面这把锁上(无限等)。要定案就得知道是谁拿着不放：
// 1=recover(主循环) 2=xfer_cb(传输完成) 3=can_xmit 4=xmit(发包) 5=alt切换
extern volatile uint32_t g_sub;      // 唯一位置编号(定义在 main.cpp)。C 文件，include
#define MARK(n) (g_sub = (n))        // 不进 usbncm.h 那个 C++ 头，只能本地声明
volatile uint8_t ncm_lock_owner = 0;
volatile uint32_t ncm_lock_want = 0;  // 位掩码：第 who 位为 1 = 那个位置正卡在拿锁上
// 卡死时主循环是「拿着锁卡在函数体内」而不是在等锁，所以还要知道卡在哪一句。
// ⚠️ 只在「锁持有者=1」时可信：start_tx 也被发包路径调用，那时这个值是别的任务写的。
volatile uint8_t ncm_recover_step = 0;
#define RSTEP(n) (ncm_recover_step = (n))
volatile uint32_t ncm_lock_seq = 0;      // 每次成功拿锁递增，卡死时若不动=锁真被占死
// 🔴 ncm_lock_want 必须在【阻塞之前】写。owner 是拿到锁之后才写的，而 UNLOCK 是先清 0
// 再解锁 —— 所以"owner=0"既可能是没人占，也可能是持有者清完标志还没解锁就卡住了。
// 2026-09-22 就是因为分不开这两种，卡死现场里的 owner=0 给不出结论。
// 🔴 want 必须是位掩码，不能是单个值。2026-09-22 用单值时：A 正卡在拿锁上(want=A)，
// B 拿到锁会把 want 清 0 —— A 在等的痕迹被别人抹掉，现场只剩 want=0，判不出有人在等。
#define NCM_LOCK_AS(who) do { ncm_lock_want |= (1u << (who)); \
                              if (ncm_mutex) osal_mutex_lock(ncm_mutex, OSAL_TIMEOUT_WAIT_FOREVER); \
                              ncm_lock_want &= ~(1u << (who)); \
                              ncm_lock_owner = (who); ncm_lock_seq++; } while (0)
#define NCM_LOCK()    NCM_LOCK_AS(0)
#define NCM_UNLOCK()  do { ncm_lock_owner = 0; if (ncm_mutex) osal_mutex_unlock(ncm_mutex); } while (0)
// 死锁诊断用。transferring 长期为 1、且 submit-done 恒为 1，就是「主机不取包」的指纹 ——
// 2026-09-15 查这个卡死花了一整轮，就因为这几个值一个都没往外暴露。
// 只为验证看门狗：人为制造死锁指纹(transferring 卡住 + submit 比 done 多)。
// 真的会让发送停下来 —— 这正是要测的，30 秒后看门狗应当自愈重启。
// 必须用一个持久标志挡住完成回调：只置 transferring 是没用的 —— 当时途中的那个真实
// xfer 一完成就把它清了，死锁自动解除，压根造不出来(2026-09-15 第一次就栽在这)。
volatile bool ncm_force_stuck = false;
void ncm_dbg_force_stuck(void) { ncm_force_stuck = true; ncm_interface.transferring = true; ncm_xfer_submit++; }
uint32_t ncm_dbg_transferring(void) { return ncm_interface.transferring ? 1 : 0; }
uint8_t ncm_dbg_ep_in(void) { return ncm_interface.ep_in; }
bool dcdx_in_rearm(uint8_t ep_addr);   // dcd_esp32sx_patched.c
bool ncm_in_rearm(void) { return ncm_interface.ep_in && dcdx_in_rearm(ncm_interface.ep_in); }
uint32_t ncm_dbg_offset(void)       { return ncm_interface.next_datagram_offset; }
uint32_t ncm_dbg_dgcount(void)      { return ncm_interface.datagram_count; }
uint32_t ncm_dbg_ntbsize(void)      { return ncm_interface.ntb_in_size; }
uint32_t ncm_dbg_alt(void)          { return ncm_interface.itf_data_alt; }

/*
 * Set up the NTB state in ncm_interface to be ready to add datagrams.
 */
static void ncm_prepare_for_tx(void) {
  ncm_interface.datagram_count = 0;
  // datagrams start after all the headers
  ncm_interface.next_datagram_offset = sizeof(nth16_t) + sizeof(ndp16_t)
      + ((CFG_TUD_NCM_MAX_DATAGRAMS_PER_NTB + 1) * sizeof(ndp16_datagram_t));
}

/*
 * If not already transmitting, start sending the current NTB to the host and swap buffers
 * to start filling the other one with datagrams.
 */
static void ncm_start_tx(void);   // 定义在下面，恢复逻辑里要先用

// 把发送侧打回干净状态。调用者必须持锁。
static void ncm_reset_tx_state(void) {
  ncm_interface.transferring = false;
  ncm_interface.current_ntb  = 0;
  ncm_prepare_for_tx();
}

// 🔴 完成回调丢失的恢复 —— 这是 NTB 死锁的治本修法，不是看门狗那种整机重启。
// 判据用 usbd_edpt_busy() 而不是超时门限：它直接问 TinyUSB 端点还有没有在途传输。
// transferring 挂着、端点却已经不忙 = 那次完成回调不会再来了，这是确凿事实不是推测。
// 连续两次(间隔 >= 50ms)都成立才动手，躲开"回调正在路上"的窗口。
volatile uint32_t ncm_tx_recover = 0;
volatile uint32_t ncm_tx_restart = 0;
// 提交失败【那一刻】的硬件层状态。事后再查没用——卡死是瞬态的，而这是唯一能回答
// "软件以为端点空闲，硬件到底忙不忙"的地方。bit0=busy bit1=stalled bit2=mounted
// bit3=suspended bit4=alt==1
volatile uint32_t ncm_fail_ep_state = 0xFFFFFFFF;   // 0xFFFFFFFF = 从没失败过
static inline uint32_t ncm_ep_state_now(void) {
  uint32_t v = 0;
  if (usbd_edpt_busy(0, ncm_interface.ep_in))    v |= 1;
  if (usbd_edpt_stalled(0, ncm_interface.ep_in)) v |= 2;
  if (tud_mounted())                             v |= 4;
  if (tud_suspended())                           v |= 8;
  if (ncm_interface.itf_data_alt == 1)           v |= 16;
  return v;
}   // 提交失败后重踢发送的次数（非 0 = 撞上过 NTB 死锁）
volatile uint32_t ncm_dg_sent = 0;
volatile uint32_t ncm_ntb_pad = 0;
volatile uint32_t ncm_ntb_nopad = 0;   // 命中 64 倍数却补不下的次数(这些 NTB 必被丢)
volatile uint32_t ncm_xmit_max_ms = 0;   // 一次 xfer 提交到完成的最长耗时，用来校准下面的门限
volatile uint32_t ncm_ep_unstick = 0;   // 第一级自愈(踢端点)次数

// 清掉 usbd 层泄漏的 busy 位。⚠️ 不能用 usbd_edpt_close：它头一条是 TU_ASSERT(dcd_edpt_close)，
// 而这个弱符号在 ESP32 上没有实现，于是整个函数直接 return，连位都不清（反汇编确认）。
// stall+clear_stall 的 dcd 两侧都有实现，顺带 disable 端点、flush 发送 FIFO、重置数据位，
// 而且全程不惊动主机 —— 网卡不掉线、IP 不变，代价比重新枚举低两个数量级。
// 🔴 进 usbd 之前必须先放开 ncm_mutex。usbd_edpt_stall/clear_stall 要拿 usbd 自己的锁，
// 而 usbd 任务在 NCM 的回调里会反过来拿 ncm_mutex —— 攥着一把去要另一把就是 AB-BA 死锁。
// 2026-09-22 实测现场：手机热点用 reason=8 踢掉连接之后，主循环卡死在 RSTEP(61)（=下面
// stall 那一行，62 步从未到达），`锁持有者=1 recover(主循环自己)`，25 秒后被看门狗重启。
// 用户看到的"连手机热点一会儿就断、而且反复重启"就是这么来的 —— 踢人只是触发条件，
// 卡死是我们自己的。放开锁还有第二个好处：卡住时不会连带把整个 NCM 饿死。
static void ncm_ep_unstick_now(void) {
  // 🔴 第二道防线。usbd_edpt_stall 在设备还没配置好时不会返回 —— 主循环就在这里空转了
  // 25 秒(任务状态是"在跑"不是"阻塞"，所以不是等锁)。判据层已经加了 in_flight，这里再挡一次：
  // 自愈动作本身绝不能比它要治的故障更严重。
  if (!tud_ready()) return;
  bool held = (ncm_mutex != NULL);
  RSTEP(61);
  if (held) { ncm_lock_owner = 0; osal_mutex_unlock(ncm_mutex); }
  usbd_edpt_stall(0, ncm_interface.ep_in);
  RSTEP(62);
  usbd_edpt_clear_stall(0, ncm_interface.ep_in);
  RSTEP(63);
  if (held) { osal_mutex_lock(ncm_mutex, OSAL_TIMEOUT_WAIT_FOREVER); ncm_lock_owner = 1; }
  ncm_interface.transferring = false;
  ncm_ep_unstick++;
}

// 返回 NCM_HEAL_*。RECYCLE 那一级要动 ncm_glue 的重新枚举，交给调用方做。
// 🔑 不依赖我们自己写的 owner/want：直接问 FreeRTOS 这把锁在谁手里。我们的记账有
// 天然盲区(清标志和真正解锁之间有窗口、who=0 和"没人"撞号)，而这个是内核的真值。
const char *ncm_mutex_holder_name(void) {
  if (!ncm_mutex) return "(锁还没建)";
  TaskHandle_t h = xSemaphoreGetMutexHolder((SemaphoreHandle_t)ncm_mutex);
  if (!h) return "(没人持有)";
  const char *n = pcTaskGetName(h);
  return n ? n : "(无名任务)";
}

int ncm_tx_recover_if_lost(uint32_t now_ms) {
  static NcmTxSt st;
  static NcmHealSt hst;
  static uint32_t last_submit = 0, submit_at = 0;
  // 43x：RSTEP 是所有任务共用的全局，别的任务跑一遍就把它覆盖掉；g_sub 只被主循环这条
  // 路写，所以定位卡点只能靠它。粒度要细到单个会进驱动的调用。
  MARK(431);
  NCM_LOCK_AS(1);
  MARK(432);
  RSTEP(1);
  bool busy = usbd_edpt_busy(0, ncm_interface.ep_in);
  MARK(433);
  RSTEP(2);
  // 统计量：一次提交到完成的最长耗时，用来校准 NCM_EP_STUCK_MS 这个死值
  if (ncm_xfer_submit != last_submit) { last_submit = ncm_xfer_submit; submit_at = now_ms; }
  else if (ncm_interface.transferring && now_ms - submit_at > ncm_xmit_max_ms)
    ncm_xmit_max_ms = now_ms - submit_at;

  MARK(434);
  int act = ncm_tx_watch(ncm_interface.transferring, busy, ncm_interface.datagram_count,
                         ncm_interface.itf_data_alt, ncm_xfer_submit, now_ms, &st);
  if (act == NCM_TX_RESTART) {
    ncm_tx_restart++;
    RSTEP(3);
    MARK(435);
    ncm_start_tx();
    MARK(436);
  } else if (act == NCM_TX_FORCE_CLEAR) {
    ncm_interface.transferring = false;
    ncm_tx_recover++;
    RSTEP(4);
    MARK(437);
    if (ncm_interface.datagram_count && ncm_interface.itf_data_alt == 1) ncm_start_tx();
    MARK(438);
  }

  MARK(439);
  RSTEP(5);
  bool in_flight = ncm_interface.transferring && ncm_interface.itf_data_alt == 1;
  int heal = ncm_heal_step(in_flight, busy, ncm_xfer_done, now_ms, ncm_enum_gen, &hst);
  MARK(440);
  // ncm_ep_unstick_now() 已不再有调用点：它会忙等不返回，见 ncm_heal_step 的说明。
  // 函数本身留着(带 tud_ready 守卫)，万一将来查明能安全调用时不用重写。
  (void)heal;
  MARK(443);
  RSTEP(8);
  NCM_UNLOCK();
  MARK(444);
  RSTEP(0);
  return heal;
}

static void ncm_start_tx(void) {
  RSTEP(90);
  if (ncm_interface.transferring) {
    RSTEP(91);
    return;
  }

  transmit_ntb_t *ntb = &transmit_ntb[ncm_interface.current_ntb];
  size_t ntb_length = ncm_interface.next_datagram_offset;

  // 🔴 full-speed 批量端点一包 64 字节。NTB 长度正好是 64 的整数倍时，主机既收不到短包、
  // 这个驱动又不发零长包(Linux 侧 dmesg 明写 "CDC NCM (NO ZLP)")，那次 URB 就不结束 ——
  // 下一个 NTB 被追加进同一个 URB，而 cdc_ncm 只解析 wBlockLength 以内的内容，后面那整个
  // NTB 连同它的帧被当尾部填充丢掉。两端计数都显示"发成功/收完成"，所以查不出来。
  // 帧长 4 字节对齐后每 16 个长度命中一次 = 6.25%，和实测 8~13% 的丢包率对得上。
  // 补 4 字节把长度错开(NTB 允许尾部填充，wBlockLength 含它)。
  if ((ntb_length % 64) == 0) {
    if (ntb_length + 4 <= CFG_TUD_NCM_IN_NTB_MAX_SIZE) { ntb_length += 4; ncm_ntb_pad++; }
    else ncm_ntb_nopad++;   // 🔴 塞不下 ⇒ 这一整个 NTB 连同里面的帧都会被主机当填充丢掉
  }

  // Fill in NTB header
  ntb->nth.dwSignature = NTH16_SIGNATURE;
  ntb->nth.wHeaderLength = sizeof(nth16_t);
  ntb->nth.wSequence = ncm_interface.nth_sequence++;
  ntb->nth.wBlockLength = ntb_length;
  ntb->nth.wNdpIndex = sizeof(nth16_t);

  // Fill in NDP16 header and terminator
  ntb->ndp.dwSignature = NDP16_SIGNATURE_NCM0;
  ntb->ndp.wLength = sizeof(ndp16_t) + (ncm_interface.datagram_count + 1) * sizeof(ndp16_datagram_t);
  ntb->ndp.wNextNdpIndex = 0;
  ntb->ndp.datagram[ncm_interface.datagram_count].wDatagramIndex = 0;
  ntb->ndp.datagram[ncm_interface.datagram_count].wDatagramLength = 0;

  // Kick off an endpoint transfer
  // 🔴 返回值必须查：提交失败就不会有完成回调，而原来无条件置 transferring=true，
  // 于是之后每次 ncm_start_tx() 都在首行返回 —— 这就是那个只能靠硬复位解的死锁。
  RSTEP(92);
  if (!usbd_edpt_xfer(0, ncm_interface.ep_in, ntb->data, ntb_length)) {
    ncm_xfer_fail++;
    ncm_fail_ep_state = ncm_ep_state_now();
    RSTEP(93);
    return;                      // 保留本 NTB，下一次 can_xmit 会再来
  }
  RSTEP(94);
  ncm_xfer_submit++;
  ncm_dg_sent += ncm_interface.datagram_count;   // 真正通过 USB 发出去的 datagram 总数
  ncm_interface.transferring = true;

  // Swap to the other NTB and clear it out
  ncm_interface.current_ntb = 1 - ncm_interface.current_ntb;
  ncm_prepare_for_tx();
}

tu_static struct ecm_notify_struct ncm_notify_connected =
{
    .header = {
        .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_INTERFACE,
          .type      = TUSB_REQ_TYPE_CLASS,
          .direction = TUSB_DIR_IN
        },
        .bRequest      = CDC_NOTIF_NETWORK_CONNECTION,
        .wValue        = 1 /* Connected */,
        .wLength       = 0,
    },
};

tu_static struct ecm_notify_struct ncm_notify_speed_change =
{
    .header = {
        .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_INTERFACE,
          .type      = TUSB_REQ_TYPE_CLASS,
          .direction = TUSB_DIR_IN
        },
        .bRequest = CDC_NOTIF_CONNECTION_SPEED_CHANGE,
        .wLength = 8,
    },
    .downlink = 10000000,
    .uplink = 10000000,
};

void tud_network_recv_renew(void)
{
  if (!ncm_interface.num_datagrams)
  {
    usbd_edpt_xfer(0, ncm_interface.ep_out, receive_ntb, sizeof(receive_ntb));
    return;
  }

  const ndp16_t *ndp = ncm_interface.ndp;
  const int i = ncm_interface.current_datagram_index;
  ncm_interface.current_datagram_index++;
  ncm_interface.num_datagrams--;

  tud_network_recv_cb(receive_ntb + ndp->datagram[i].wDatagramIndex, ndp->datagram[i].wDatagramLength);
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+

void netd_init(void)
{
  if (!ncm_mutex) ncm_mutex = osal_mutex_create(&ncm_mutex_def);
  tu_memclr(&ncm_interface, sizeof(ncm_interface));
  ncm_interface.ntb_in_size = CFG_TUD_NCM_IN_NTB_MAX_SIZE;
  ncm_interface.max_datagrams_per_ntb = CFG_TUD_NCM_MAX_DATAGRAMS_PER_NTB;
  ncm_prepare_for_tx();
}

void netd_reset(uint8_t rhport)
{
  (void) rhport;

  netd_init();
  ncm_enum_gen++;
}

uint16_t netd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len)
{
  // confirm interface hasn't already been allocated
  TU_ASSERT(0 == ncm_interface.ep_notif, 0);

  //------------- Management Interface -------------//
  ncm_interface.itf_num = itf_desc->bInterfaceNumber;

  uint16_t drv_len = sizeof(tusb_desc_interface_t);
  uint8_t const * p_desc = tu_desc_next( itf_desc );

  // Communication Functional Descriptors
  while ( TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
  {
    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // notification endpoint (if any)
  if ( TUSB_DESC_ENDPOINT == tu_desc_type(p_desc) )
  {
    TU_ASSERT( usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *) p_desc), 0 );

    ncm_interface.ep_notif = ((tusb_desc_endpoint_t const *) p_desc)->bEndpointAddress;

    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  //------------- Data Interface -------------//
  // - CDC-NCM data interface has 2 alternate settings
  //   - 0 : zero endpoints for inactive (default)
  //   - 1 : IN & OUT endpoints for transfer of NTBs
  TU_ASSERT(TUSB_DESC_INTERFACE == tu_desc_type(p_desc), 0);

  do
  {
    tusb_desc_interface_t const * data_itf_desc = (tusb_desc_interface_t const *) p_desc;
    TU_ASSERT(TUSB_CLASS_CDC_DATA == data_itf_desc->bInterfaceClass, 0);

    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  } while((TUSB_DESC_INTERFACE == tu_desc_type(p_desc)) && (drv_len <= max_len));

  // Pair of endpoints
  TU_ASSERT(TUSB_DESC_ENDPOINT == tu_desc_type(p_desc), 0);

  TU_ASSERT(usbd_open_edpt_pair(rhport, p_desc, 2, TUSB_XFER_BULK, &ncm_interface.ep_out, &ncm_interface.ep_in) );

  drv_len += 2*sizeof(tusb_desc_endpoint_t);

  return drv_len;
}

static void ncm_report(void)
{
  uint8_t const rhport = 0;
  if (ncm_interface.report_state == REPORT_SPEED) {
    ncm_notify_speed_change.header.wIndex = ncm_interface.itf_num;
    usbd_edpt_xfer(rhport, ncm_interface.ep_notif, (uint8_t *) &ncm_notify_speed_change, sizeof(ncm_notify_speed_change));
    ncm_interface.report_state = REPORT_CONNECTED;
    ncm_interface.report_pending = true;
  } else if (ncm_interface.report_state == REPORT_CONNECTED) {
    ncm_notify_connected.header.wIndex = ncm_interface.itf_num;
    usbd_edpt_xfer(rhport, ncm_interface.ep_notif, (uint8_t *) &ncm_notify_connected, sizeof(ncm_notify_connected));
    ncm_interface.report_state = REPORT_DONE;
    ncm_interface.report_pending = true;
  }
}

TU_ATTR_WEAK void tud_network_link_state_cb(bool state)
{
  (void)state;
}

// Handle class control request
// return false to stall control endpoint (e.g unsupported request)
bool netd_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  if ( stage != CONTROL_STAGE_SETUP ) return true;

  switch ( request->bmRequestType_bit.type )
  {
    case TUSB_REQ_TYPE_STANDARD:
      switch ( request->bRequest )
      {
        case TUSB_REQ_GET_INTERFACE:
        {
          uint8_t const req_itfnum = (uint8_t) request->wIndex;
          TU_VERIFY(ncm_interface.itf_num + 1 == req_itfnum);

          tud_control_xfer(rhport, request, &ncm_interface.itf_data_alt, 1);
        }
          break;

        case TUSB_REQ_SET_INTERFACE:
        {
          uint8_t const req_itfnum = (uint8_t) request->wIndex;
          uint8_t const req_alt    = (uint8_t) request->wValue;

          // Only valid for Data Interface with Alternate is either 0 or 1
          TU_VERIFY(ncm_interface.itf_num + 1 == req_itfnum && req_alt < 2);

          if (req_alt != ncm_interface.itf_data_alt) {
            ncm_interface.itf_data_alt = req_alt;

            // 主机 down/up 接口(Linux 的 ifconfig、重新配置 cdc_ncm)会走 alt=0 -> alt=1。
            // alt=0 时若有在途传输，它的完成回调永远不会来，transferring 就此挂死，
            // alt=1 之后整条下行全堵。上游 #3436 是同一类问题。切 alt 一律重置发送侧。
            NCM_LOCK_AS(5);
            ncm_reset_tx_state();
            NCM_UNLOCK();

            if (ncm_interface.itf_data_alt) {
              if (!usbd_edpt_busy(rhport, ncm_interface.ep_out)) {
                tud_network_recv_renew(); // prepare for incoming datagrams
              }
              if (!ncm_interface.report_pending) {
                ncm_report();
              }
            }

            tud_network_link_state_cb(ncm_interface.itf_data_alt);
          }

          tud_control_status(rhport, request);
        }
          break;

          // unsupported request
        default: return false;
      }
      break;

    case TUSB_REQ_TYPE_CLASS:
      TU_VERIFY (ncm_interface.itf_num == request->wIndex);

      if (NCM_GET_NTB_PARAMETERS == request->bRequest)
      {
        tud_control_xfer(rhport, request, (void*)(uintptr_t) &ntb_parameters, sizeof(ntb_parameters));
      }

      break;

      // unsupported request
    default: return false;
  }

  return true;
}

static void handle_incoming_datagram(uint32_t len)
{
  uint32_t size = len;

  if (len == 0) {
    return;
  }

  TU_ASSERT(size >= sizeof(nth16_t), );

  const nth16_t *hdr = (const nth16_t *)receive_ntb;
  TU_ASSERT(hdr->dwSignature == NTH16_SIGNATURE, );
  TU_ASSERT(hdr->wNdpIndex >= sizeof(nth16_t) && (hdr->wNdpIndex + sizeof(ndp16_t)) <= len, );

  const ndp16_t *ndp = (const ndp16_t *)(receive_ntb + hdr->wNdpIndex);
  TU_ASSERT(ndp->dwSignature == NDP16_SIGNATURE_NCM0 || ndp->dwSignature == NDP16_SIGNATURE_NCM1, );
  TU_ASSERT(hdr->wNdpIndex + ndp->wLength <= len, );

  int num_datagrams = (ndp->wLength - 12) / 4;
  ncm_interface.current_datagram_index = 0;
  ncm_interface.num_datagrams = 0;
  ncm_interface.ndp = ndp;
  for (int i = 0; i < num_datagrams && ndp->datagram[i].wDatagramIndex && ndp->datagram[i].wDatagramLength; i++)
  {
    ncm_interface.num_datagrams++;
  }

  tud_network_recv_renew();
}

bool netd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) rhport;
  (void) result;

  /* new datagram receive_ntb */
  if (ep_addr == ncm_interface.ep_out )
  {
    handle_incoming_datagram(xferred_bytes);
  }

  /* data transmission finished */
  if (ep_addr == ncm_interface.ep_in )
  {
    NCM_LOCK_AS(2);
    if (ncm_interface.transferring && !ncm_force_stuck) {
      ncm_interface.transferring = false;
      ncm_xfer_done++;
    }

    // If there are datagrams queued up that we tried to send while this NTB was being emitted, send them now
    if (ncm_interface.datagram_count && ncm_interface.itf_data_alt == 1) {
      ncm_start_tx();
    }
    NCM_UNLOCK();
  }

  if (ep_addr == ncm_interface.ep_notif )
  {
    ncm_interface.report_pending = false;
    ncm_report();
  }

  return true;
}

// poll network driver for its ability to accept another packet to transmit
// 卡死时到底卡在哪个条件上 —— 三个原因分开计数，别再靠猜
volatile uint32_t ncm_nx_alt, ncm_nx_count, ncm_nx_size;
volatile uint32_t ncm_xmit_reject;   // 持锁重查时被拒(can_xmit 之后状态变了)

bool tud_network_can_xmit(uint16_t size)
{
  bool ok = true;
  NCM_LOCK_AS(3);
  if (ncm_interface.itf_data_alt != 1) { ncm_nx_alt++; ok = false; }
  else if (ncm_interface.datagram_count >= ncm_interface.max_datagrams_per_ntb) {
    ncm_nx_count++; ok = false;
  }
  else if (ncm_interface.next_datagram_offset + size > ncm_interface.ntb_in_size) {
    ncm_nx_size++; ok = false;
  }
  NCM_UNLOCK();
  return ok;
}

void tud_network_xmit(void *ref, uint16_t arg)
{
  NCM_LOCK_AS(4);
  // 持锁后必须重查一次：can_xmit 和这里之间状态可能已经变了(检查与使用之间的窗口)
  if (ncm_interface.itf_data_alt != 1 ||
      ncm_interface.datagram_count >= ncm_interface.max_datagrams_per_ntb) {
    ncm_xmit_reject++;
    NCM_UNLOCK();
    return;
  }
  transmit_ntb_t *ntb = &transmit_ntb[ncm_interface.current_ntb];
  size_t next_datagram_offset = ncm_interface.next_datagram_offset;

  uint16_t size = tud_network_xmit_cb(ntb->data + next_datagram_offset, ref, arg);

  ntb->ndp.datagram[ncm_interface.datagram_count].wDatagramIndex = ncm_interface.next_datagram_offset;
  ntb->ndp.datagram[ncm_interface.datagram_count].wDatagramLength = size;

  ncm_interface.datagram_count++;
  next_datagram_offset += size;

  // round up so the next datagram is aligned correctly
  next_datagram_offset += (CFG_TUD_NCM_ALIGNMENT - 1);
  next_datagram_offset -= (next_datagram_offset % CFG_TUD_NCM_ALIGNMENT);

  ncm_interface.next_datagram_offset = next_datagram_offset;

  ncm_start_tx();
  NCM_UNLOCK();
}

#endif
