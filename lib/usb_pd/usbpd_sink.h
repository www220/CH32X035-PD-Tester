#pragma once

#include <stdbool.h>
#include <string.h>
#include "usbpd_def.h"

// 调试打印
#define CONFIG_PD_DEBUG_ENABLE

#ifdef CONFIG_PD_DEBUG_ENABLE
#define pd_printf(format, ...) LOG(format, ##__VA_ARGS__)
#else
#define pd_printf(x...)
#endif

// PD 状态机
// TODO: 按照 Table 8.154 Policy Engine States 规范命名 PE_SNK_XXX
// TODO: 实现 wait 状态的超时退出
typedef enum {
    PD_STATE_DISCONNECTED = 0,
    PD_STATE_CHECK_CONNECT,
    PD_STATE_CONNECT,

    PD_STATE_IDLE,
} pd_state_t;

typedef struct {
    volatile USBPD_SpecificationRevision_t pd_version;    // PD Specification Revision，需在收到 Source_Capabilities 后设置为和 Source 相同的 version
    volatile pd_state_t pd_state;                         // PD 状态机当前状态
    volatile pd_state_t pd_last_state;                    // PD 状态机之前状态
    volatile uint8_t cc_none_times;                       // cc 未连接计数
    volatile uint8_t cc1_connect_times;                   // cc1 检测计数
    volatile uint8_t cc2_connect_times;                   // cc2 检测计数
    volatile uint8_t sink_message_id;                     // Sink 消息 ID
    volatile uint8_t cable_message_id;                    // Cable 消息 ID，用于模拟 E-Marker
    volatile uint8_t port_data_role;                      // 0b UFP, 1b DFP
} pd_control_t;

/******************************************************************************
 * Public Interface Functions
 *****************************************************************************/

/**
 * @brief 初始化 PD Sink
 */
void usbpd_sink_init(void);

/**
 * @brief 发送 HARD_RESET
 */
void usbpd_sink_hard_reset(void);
