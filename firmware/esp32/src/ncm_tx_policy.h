// NCM 发送侧看门狗的纯判定。抽出来是为了能在电脑上跑 test_ncm_tx.cpp ——
// 这段逻辑有两种卡死形态，而它们的判据是【相反】的，混在一起写就是今天那个 bug 的来源。
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define NCM_TX_CLEAR_MS     50   // 端点空闲要连续成立这么久才认(躲开回调正在路上的窗口)
#define NCM_TX_IDLE_MS      50   // 提交失败后多久重踢。正常路径下这个状态根本不该持续存在
#define NCM_EP_STUCK_MS    200   // 端点忙而完成计数不动，持续这么久 = 卡死。一个 NTB 正常只要 3ms
#define NCM_HEAL_GAP_MS    300   // 轻自愈后给它多久见效，再升级到重的
#define NCM_RECYCLE_COOL_MS 60000// 两次重新枚举的最小间隔

typedef struct { uint32_t idle_since, clr_since, clr_submit; uint8_t idle_armed, clr_armed; } NcmTxSt;
typedef struct { uint32_t last_done, since, acted_at, last_recycle, gen, base; uint8_t level, armed; } NcmHealSt;
enum { NCM_TX_NONE = 0, NCM_TX_RESTART, NCM_TX_FORCE_CLEAR };
enum { NCM_HEAL_NONE = 0, NCM_HEAL_UNSTICK, NCM_HEAL_RECYCLE, NCM_HEAL_REARM };

// 整机复位看门狗的判定。只在「这次枚举以来主机发过帧」之后计时：usbnet 接口 up 才轮询 IN，
// 冷启动时主机 20 多秒不取包是正常的；主机已经在发、却不取我们的包，才是重新枚举也救不回的真卡死。
typedef struct { uint32_t gen, rx_base, last_done, since; uint8_t init, armed; } NcmWdSt;
static inline bool ncm_wd_step(bool transferring, uint32_t done, uint32_t rx, uint32_t now_ms,
                               uint32_t gen, uint32_t limit_ms, NcmWdSt *st) {
  if (!st->init || gen != st->gen) {
    st->init = 1; st->gen = gen; st->rx_base = rx; st->last_done = done; st->armed = 0;
    return false;
  }
  bool stuck = transferring && done == st->last_done && rx != st->rx_base;
  st->last_done = done;
  if (!stuck) { st->armed = 0; return false; }
  // 🔴 别用 since=now|1 省掉 armed：同一毫秒里会调好几次，now 为偶数时 since 比 now 大 1，差值回绕成 42 亿当场开火
  if (!st->armed) { st->armed = 1; st->since = now_ms; return false; }
  return now_ms - st->since >= limit_ms;
}

// transferring=true 而端点已空闲 ⇒ 那次完成回调不会再来了，清掉标志是确凿的。
// ⚠️ 这里曾用 500ms 超时代替 ep_busy，等于拿一个软件标志去猜另一层的状态：传输其实还在途
//    就把 transferring 清了，之后每次 usbd_edpt_xfer 都撞在它自己的 busy 位上必失败。
// transferring=false 而 NTB 里有货 ⇒ 上次提交失败，而重试只挂在 tud_network_xmit 末尾、
//   NTB 一满 can_xmit 恒假就再也走不到那条路 ⇒ 必须由这里重踢（RESTART）
static inline int ncm_tx_watch(bool transferring, bool ep_busy, uint32_t dg_count,
                               int alt, uint32_t submit, uint32_t now_ms, NcmTxSt *st) {
  if (transferring) {
    st->idle_armed = 0;
    if (ep_busy) { st->clr_armed = 0; return NCM_TX_NONE; }
    // ⚠️ 端点一空就动手是错的：usbd 任务先清 busy、再调 NCM 的回调，这中间有个窗口，
    //    主循环恰好插进去就会看到假的"回调丢了"，于是误清标志并重发 —— 实测大包丢 40%。
    // ⚠️ 计时必须锁定【同一次传输】：窗口是断续出现的，只按时长累计照样会凑满 50ms 误触发。
    if (!st->clr_armed || st->clr_submit != submit) {
      st->clr_armed = 1; st->clr_submit = submit; st->clr_since = now_ms;
      return NCM_TX_NONE;
    }
    if (now_ms - st->clr_since < NCM_TX_CLEAR_MS) return NCM_TX_NONE;
    st->clr_armed = 0;
    return NCM_TX_FORCE_CLEAR;
  }
  st->clr_armed = 0;                         // 已经不在途 ⇒ 上面那个窗口的计时作废
  if (!dg_count || alt != 1) { st->idle_armed = 0; return NCM_TX_NONE; }
  if (!st->idle_armed) { st->idle_armed = 1; st->idle_since = now_ms; return NCM_TX_NONE; }
  if (now_ms - st->idle_since < NCM_TX_IDLE_MS) return NCM_TX_NONE;
  st->idle_armed = 0;
  return NCM_TX_RESTART;
}

// 端点卡死的分级自愈。判据必须用【完成计数】不动而不是提交计数：提交成功只说明数据交给了
// TinyUSB，主机不来取时提交照样成功，只有 done 增长才证明链路真的通了。
// ⚠️ usbd 层的 busy 位一旦泄漏成 1，usbd_edpt_xfer 第一条指令就 return false（反汇编确认），
//    NCM 层重试多少次都注定失败 —— 实测连续 4011 次。所以自愈必须清掉那个位，重试是没用的。
// ⚠️ recycle 的冷却不能省：2026-09-14 在转发任务里每帧都调，55 次之后链路彻底起不来。
// in_flight = 确实有一个 NTB 提交出去还没回来，且主机已经选了数据接口的 alt setting。
// 🔴 少了这个前提，开机时就会误判：主机还没选 alt、一个传输都没有，而 usbd_edpt_busy 返回真、
//    done 恒为 0 —— 两个判据同时成立，200ms 后就去 stall 一个还没打开的端点，当场挂死主循环
//    25 秒，被看门狗重启。2026-09-22 冷启动白白吃掉 26 秒就是这么来的（卡点 4.441）。
// gen = 第几次总线复位（主机每次枚举都会先复位）。每次枚举都是一次新的 bring-up，状态要归零。
static inline int ncm_heal_step(bool in_flight, bool ep_busy, uint32_t done, uint32_t now_ms,
                                uint32_t gen, NcmHealSt *st) {
  if (gen != st->gen) {
    st->gen = gen; st->base = done; st->last_done = done; st->armed = 0; st->level = 0; st->acted_at = 0;
  }
  if (!in_flight) { st->armed = 0; return NCM_HEAL_NONE; }
  // 🔴 主机一次都还没取走过包之前，"端点忙且完成计数不动"没有任何诊断价值 —— 那正是
  //    bring-up 的正常样子：我们提交了一个 NTB，主机的 cdc_ncm 还没开始轮询而已。
  //    2026-09-22 实测：in_flight 和 alt==1 在开机那一刻【都成立】，所以只加 in_flight
  //    挡不住；自愈照样触发，而它的动作 usbd_edpt_stall 在这个状态下会忙等到天荒地老
  //    （任务状态是"在跑"不是"阻塞"，八秒纹丝不动），主循环被拖死 25 秒。
  // 🔴 必须比「这次枚举时的计数」，不能比 0：done 是开机累计的。09-28 拔线 90 分钟后插回，
  //    done 已是 8519，「主机还没取过包」这道防线形同虚设，3 秒内就误判成卡死去重新枚举，
  //    把刚建好的连接弄坏，30 秒后看门狗重启整块芯片。
  if (done == st->base) { st->armed = 0; return NCM_HEAL_NONE; }
  if (done != st->last_done) {                 // 主机真的取走了包 ⇒ 确实好了，一切归零
    st->last_done = done; st->armed = 0; st->level = 0; st->acted_at = 0;
    return NCM_HEAL_NONE;
  }
  // ⚠️ 端点空下来【不能】清 level：第一级自愈后若 NTB 恰好没货，busy 就会短暂为假，
  //    据此归零会让状态机永远升不到重新枚举，在主机真不取包时退化成每 200ms 自激一次。
  if (!ep_busy) { st->armed = 0; return NCM_HEAL_NONE; }
  if (!st->armed) { st->armed = 1; st->since = now_ms; return NCM_HEAL_NONE; }
  if (now_ms - st->since < NCM_EP_STUCK_MS) return NCM_HEAL_NONE;
  if (st->acted_at && now_ms - st->acted_at < NCM_HEAL_GAP_MS) return NCM_HEAL_NONE;
  if (st->level >= NCM_HEAL_UNSTICK) {
    if (st->last_recycle && now_ms - st->last_recycle < NCM_RECYCLE_COOL_MS) return NCM_HEAL_NONE;
    st->last_recycle = now_ms; st->acted_at = now_ms; st->level = NCM_HEAL_RECYCLE;
    return NCM_HEAL_RECYCLE;
  }
  // 🔴 不再返回 UNSTICK。它的动作 usbd_edpt_stall 在这块芯片上会忙等不返回，把主循环
  //    拖死 25 秒、被看门狗重启整块芯片 —— 自愈比它要治的故障严重得多。
  //    2026-09-22 连着六次冷启动都卡在这一句(卡点 4.441，任务状态"在跑"不是"阻塞")；
  //    先后加了 in_flight 和 done>0 两道前置条件，只是把挂死时刻从 ESP32 的 1 秒推到
  //    39 秒，挡不住。而正常运行中它【从来没有真正需要过】：所有实测 N 输出里
  //    "踢端点" 恒为 0 次。⇒ 这一级降级成"只记一次"，下一轮直接走重新枚举。
  //    真正的轻量恢复由 ncm_tx_watch 的 FORCE_CLEAR 负责(清标志+重新提交)，不碰端点。
  // 11-10 第一级改成 REARM：补过的驱动里 dcdx_in_rearm()，按 PKTCNT 续传没发出去的部分，等待都有上限，
  //    不会像 stall 那样忙等。救不回来 HEAL_GAP 之后照旧升到重新枚举。
  st->level = NCM_HEAL_UNSTICK; st->acted_at = now_ms;
  return NCM_HEAL_REARM;
}
