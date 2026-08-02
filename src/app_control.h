#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "qc.h"
#include "app_button.h"

typedef enum {
    POWER_MODE_NONE = 0,
    POWER_MODE_PD,
    POWER_MODE_QC2,
} power_mode_t;

/**
 * @brief 电源状态信息
 */
typedef struct {
    power_mode_t power_mode;  // 当前电源模式

    char power_mode_name[4];  // 电源模式名称
    char power_mode_desc[4];  // 电源模式描述

    uint8_t pd_position;          // PD PDO 位置 (1-based)
    bool is_epr_ready;            // PD 是否已进入 EPR 模式
    usb_qc_voltage_t qc_voltage;  // QC 电压档位

    uint16_t negotiate_voltage;      // 当前协商电压
    uint16_t negotiate_current;      // 当前协商电流
    uint16_t negotiate_epr_avs_pdp;  // epr avs pdp（w）

    uint16_t vbus_voltage;  // VBUS 电压

    bool is_edit_mode;
    uint16_t edit_voltage;
} power_state_t;

/**
 * @brief 存储信息
 */
typedef struct {
    uint8_t power_mode;             // 当前电源模式
    uint8_t is_epr_ready;           // EPR 模式
    uint8_t pdo_type;               // PDO 类型
    uint8_t apdo_subtype;           // APDO 子类型
    uint8_t qc_voltage;             // QC 电压档位
    uint8_t rotate_mode;            // 当前旋转模式
    uint16_t voltage;               // 当前协商电压
    uint8_t rsv[8];                 // 保留字段
    float vbus_calc_point[8];       // 校准的电压数据
    uint32_t save_state_crc;        // CRC32校验数据
} save_state_t;
extern save_state_t* g_save_state;
void Flash_Sync_State(void);
void Flash_Write_State(void);
uint32_t CRC32_Calculate(const uint8_t *data, int len);
// 0x1      表示进入校准状态
// 0x2      表示输出电压状态
// 0x4      表示输出状态有变化点击按键应该修改变化
// 0x8      表示已经同步flash诱骗状态
// 0x10     表示旋转状态
extern uint32_t save_state_flag;

/**
 * @brief 初始化控制层
 */
void app_control_init(void);

/**
 * @brief 处理按键事件
 * @param event 按键事件类型
 */
void app_control_handle_button(button_event_t event);

/**
 * @brief 获取当前电源状态
 * @return 电源状态结构体
 */
power_state_t app_control_get_power_state(void);
