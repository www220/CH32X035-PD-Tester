#include "app_control.h"
#include "utils_print.h"
#include "utils_delay.h"
#include "usbpd_sink.h"
#include "qc.h"
#include "adc.h"

uint32_t save_state_flag;
static uint8_t flash_buf[256];
save_state_t* g_save_state = (save_state_t*)flash_buf;

static power_state_t g_power_state = {
    .power_mode = POWER_MODE_NONE,
    .pd_position = 1,
    .is_epr_ready = false,
    .qc_voltage = USB_QC_VOLTAGE_5V,
    .negotiate_voltage = 0,
    .negotiate_current = 0,
    .vbus_voltage = 0,
    .is_edit_mode = false,
    .edit_voltage = 0,
};

// APDO 电压记忆（使用 position 作为索引，最多支持 11 个 PDO）
static uint16_t g_apdo_voltage_memory[12] = {0};  // position 1-11，索引 0 不使用

static bool is_apdo(uint8_t position);

/**
 * @brief 获取 APDO 档位的记忆电压（如果存在）
 * @param position PDO position (1-based)
 * @return 记忆的电压值 (mV)，如果没有记忆则返回 0
 */
static uint16_t get_apdo_memory_voltage(uint8_t position) {
    if (position >= 1 && position <= 11) {
        return g_apdo_voltage_memory[position];
    }
    return 0;
}

static bool pd_position_exists(uint8_t position) {
    const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

    for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
        if (pdos_ptr->pdo[i].position == position) {
            return true;
        }
    }

    return false;
}

static void sync_pd_ui_position(void) {
    if (g_power_state.power_mode != POWER_MODE_PD) {
        return;
    }

    if (pd_position_exists(g_power_state.pd_position)) {
        return;
    }

    uint8_t fallback_position = usbpd_sink_get_position();
    if (!pd_position_exists(fallback_position)) {
        const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();
        fallback_position = pdos_ptr->pdo_count > 0 ? pdos_ptr->pdo[0].position : 0;
    }

    if (fallback_position == 0) {
        g_power_state.is_edit_mode = false;
        g_power_state.edit_voltage = 0;
        return;
    }

    g_power_state.pd_position = fallback_position;

    if (!is_apdo(g_power_state.pd_position)) {
        g_power_state.is_edit_mode = false;
        g_power_state.edit_voltage = 0;
    }
}

/**
 * @brief 保存 APDO 档位的编辑电压
 * @param position PDO position (1-based)
 * @param voltage 电压值 (mV)
 */
static void save_apdo_memory_voltage(uint8_t position, uint16_t voltage) {
    if (position >= 1 && position <= 11) {
        g_apdo_voltage_memory[position] = voltage;
        LOG("Saved APDO voltage memory: position=%u, voltage=%umV\n", position, voltage);
    }
}

static uint16_t get_apdo_voltage_step(USBPD_APDO_Subtype_t apdo_subtype) {
    switch (apdo_subtype) {
        case APDO_TYPE_SPR_PPS:
            return 20;  // SPR PPS: 20mV steps
        case APDO_TYPE_EPR_AVS:
            return 100;  // EPR AVS: 100mV steps
        case APDO_TYPE_SPR_AVS:
            return 100;  // SPR AVS: 100mV steps
        default:
            return 100;
    }
}

/**
 * @brief 获取 APDO 的电压范围和步进
 */
static bool get_apdo_voltage_range(uint8_t position, uint16_t *min_voltage, uint16_t *max_voltage, uint16_t *step) {
    const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

    for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
        if (pdos_ptr->pdo[i].position == position) {
            if (pdos_ptr->pdo[i].pdo_type == PDO_TYPE_APDO) {
                *step = get_apdo_voltage_step(pdos_ptr->pdo[i].apdo_subtype);

                switch (pdos_ptr->pdo[i].apdo_subtype) {
                    case APDO_TYPE_SPR_PPS:
                        *min_voltage = pdos_ptr->pdo[i].pps.min_voltage;
                        *max_voltage = pdos_ptr->pdo[i].pps.max_voltage;
                        return true;
                    case APDO_TYPE_SPR_AVS:
                        *min_voltage = pdos_ptr->pdo[i].spr_avs.min_voltage;
                        *max_voltage = pdos_ptr->pdo[i].spr_avs.max_voltage;
                        return true;
                    case APDO_TYPE_EPR_AVS:
                        *min_voltage = pdos_ptr->pdo[i].epr_avs.min_voltage;
                        *max_voltage = pdos_ptr->pdo[i].epr_avs.max_voltage;
                        return true;
                    default:
                        return false;
                }
            }
            return false;  // 不是 APDO
        }
    }
    return false;  // 未找到
}

/**
 * @brief 检查当前 PDO 是否为 APDO
 */
static bool is_apdo(uint8_t position) {
    const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

    for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
        if (pdos_ptr->pdo[i].position == position) {
            return pdos_ptr->pdo[i].pdo_type == PDO_TYPE_APDO;
        }
    }
    return false;
}

/**
 * @brief 初始化控制层
 */
void app_control_init(void) {
    adc_init();

    // 读取falsh解析校准数据
    uint32_t address = FLASH_BASE + 0xC000 - 256;
    memset(&flash_buf, 0xff, sizeof(flash_buf));
    memcpy(&flash_buf, (uint32_t *)address, sizeof(save_state_t));
    if (g_save_state->save_state_crc == CRC32_Calculate(flash_buf, sizeof(save_state_t) - 4)) {
        for (int i = 0; i < VBUS_CAL_POINTS_COUNT; i++) {
            if (VBUS_CAL_POINTS[i].actual - 1000 <= g_save_state->vbus_calc_point[i] 
                && g_save_state->vbus_calc_point[i] <= VBUS_CAL_POINTS[i].actual + 1000) {
                // 电压校准数据正确
                VBUS_CAL_POINTS[i].measured = g_save_state->vbus_calc_point[i];
            }
        }
    }else{
        // 默认不必处理诱骗逻辑
        memset(g_save_state, 0, sizeof(save_state_t));
    }

    // 检测启动模式
    g_power_state.power_mode = POWER_MODE_NONE;

    // 初始化 PD Sink
    usbpd_sink_init();
    for (uint16_t t = 0; t < 500; t++) {
        if (usbpd_sink_get_ready()) {
            g_power_state.power_mode = POWER_MODE_PD;
            break;
        }
        delay_ms(1);
    }

    // 检测 BC1.2
    if (g_power_state.power_mode == POWER_MODE_NONE) {
        usb_bc_type_t bc_type = usb_bc_check();
        if (bc_type == USB_BC_TYPE_DCP) {
            g_power_state.power_mode = POWER_MODE_QC2;
        }
    }

    // 初始化 QC
    if (g_power_state.power_mode == POWER_MODE_QC2) {
        usb_qc_request(USB_QC_VOLTAGE_5V);
        delay_ms(1250);
    }

    // 初始化 PD position
    if (g_power_state.power_mode == POWER_MODE_PD) {
        g_power_state.pd_position = usbpd_sink_get_position();
    }

    // 读取flash参数同步诱骗参数
    Flash_Sync_State();
}

/**
 * @brief 处理左键 - 向前切换档位或减小电压
 */
static void handle_left_button(void) {
    LOG("Control: handle left\n");

    // 编辑模式下：减小电压
    if (g_power_state.is_edit_mode && g_power_state.power_mode == POWER_MODE_PD) {
        uint16_t min_voltage, max_voltage, step;
        if (get_apdo_voltage_range(g_power_state.pd_position, &min_voltage, &max_voltage, &step)) {
            // 调到最小值后，再减则跳到最大值
            if (g_power_state.edit_voltage <= min_voltage) {
                g_power_state.edit_voltage = max_voltage;
            } else {
                // 按步进减小电压
                if (g_power_state.edit_voltage >= min_voltage + step) {
                    g_power_state.edit_voltage -= step;
                } else {
                    g_power_state.edit_voltage = min_voltage;
                }
            }
            LOG("Edit voltage decreased to: %u mV\n", g_power_state.edit_voltage);
            save_state_flag |= 0x4;
        }
        return;
    }

    // 非编辑模式：切换档位
    if (g_power_state.power_mode == POWER_MODE_QC2) {
        // QC 模式：向前切换
        if (g_power_state.qc_voltage == USB_QC_VOLTAGE_5V) {
            g_power_state.qc_voltage = USB_QC_VOLTAGE_20V;  // 循环到最高档
        } else {
            g_power_state.qc_voltage--;
        }
        save_state_flag |= 0x4;
    }

    if (g_power_state.power_mode == POWER_MODE_PD) {
        // PD 模式：仅更新 position，不实际切换
        const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

        // 找到当前 position 在数组中的索引
        int8_t current_index = -1;
        for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
            if (pdos_ptr->pdo[i].position == g_power_state.pd_position) {
                current_index = i;
                break;
            }
        }

        // 向前移动
        if (current_index > 0) {
            current_index--;
        } else {
            current_index = pdos_ptr->pdo_count - 1;  // 循环到最后
        }

        g_power_state.pd_position = pdos_ptr->pdo[current_index].position;
        save_state_flag |= 0x4;
    }
}

/**
 * @brief 处理右键 - 向后切换档位或增加电压
 */
static void handle_right_button(void) {
    LOG("Control: handle right\n");

    // 编辑模式下：增加电压
    if (g_power_state.is_edit_mode && g_power_state.power_mode == POWER_MODE_PD) {
        uint16_t min_voltage, max_voltage, step;
        if (get_apdo_voltage_range(g_power_state.pd_position, &min_voltage, &max_voltage, &step)) {
            // 调到最大值后，再加则跳到最小值
            if (g_power_state.edit_voltage >= max_voltage) {
                g_power_state.edit_voltage = min_voltage;
            } else {
                // 按步进增加电压
                if (g_power_state.edit_voltage <= max_voltage - step) {
                    g_power_state.edit_voltage += step;
                } else {
                    g_power_state.edit_voltage = max_voltage;
                }
            }
            LOG("Edit voltage increased to: %umV\n", g_power_state.edit_voltage);
            save_state_flag |= 0x4;
        }
        return;
    }

    // 非编辑模式：切换档位
    if (g_power_state.power_mode == POWER_MODE_QC2) {
        // QC 模式：向后切换
        g_power_state.qc_voltage++;
        if (g_power_state.qc_voltage >= USB_QC_VOLTAGE_MAX) {
            g_power_state.qc_voltage = USB_QC_VOLTAGE_5V;  // 循环到最低档
        }
        save_state_flag |= 0x4;
    }

    if (g_power_state.power_mode == POWER_MODE_PD) {
        // PD 模式：仅更新 position，不实际切换
        const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

        // 找到当前 position 在数组中的索引
        int8_t current_index = -1;
        for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
            if (pdos_ptr->pdo[i].position == g_power_state.pd_position) {
                current_index = i;
                break;
            }
        }

        // 向后移动
        if (current_index < pdos_ptr->pdo_count - 1) {
            current_index++;
        } else {
            current_index = 0;  // 循环到第一个
        }

        g_power_state.pd_position = pdos_ptr->pdo[current_index].position;
        save_state_flag |= 0x4;
    }
}

/**
 * @brief 处理下键 - 请求电压
 */
static void handle_down_button(void) {
    LOG("Control: handle down\n");
    if ((save_state_flag & 0x2) && !(save_state_flag & 0x4)) {
        // 实现按一下开启按一下关闭的功能
        save_state_flag &= ~0x2;
        return;
    }

    if (g_power_state.power_mode == POWER_MODE_QC2) {
        // QC 模式：请求选中的电压
        usb_qc_request(g_power_state.qc_voltage);
        save_state_flag |= 0x2;
        save_state_flag &= ~0x4;
    }

    if (g_power_state.power_mode == POWER_MODE_PD) {
        if (g_power_state.is_edit_mode) {
            // 编辑模式：请求编辑的电压
            bool success = usbpd_sink_set_apdo_position_with_voltage(g_power_state.pd_position, g_power_state.edit_voltage);
            if (success) {
                LOG("Request APDO voltage: %umV\n", g_power_state.edit_voltage);
                // 保存编辑的电压到记忆中
                save_state_flag |= 0x2;
                save_state_flag &= ~0x4;
                save_apdo_memory_voltage(g_power_state.pd_position, g_power_state.edit_voltage);
            } else {
                LOG("Failed to request APDO voltage\n");
                save_state_flag &= ~0x2;
                save_state_flag &= ~0x4;
            }
        } else {
            // 非编辑模式：切换到选中的 position
            const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

            // 查找选中的 PDO
            for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
                if (pdos_ptr->pdo[i].position == g_power_state.pd_position) {
                    if (pdos_ptr->pdo[i].pdo_type == PDO_TYPE_FIXED_SUPPLY) {
                        bool success = usbpd_sink_set_fpdo_position(g_power_state.pd_position);
                        save_state_flag = success ? save_state_flag|0x2 : save_state_flag & ~0x2;
                        save_state_flag &= ~0x4;
                    } else if (pdos_ptr->pdo[i].pdo_type == PDO_TYPE_APDO) {
                        // APDO：优先使用记忆电压，否则使用最小电压
                        uint16_t request_voltage = get_apdo_memory_voltage(g_power_state.pd_position);

                        if (request_voltage == 0) {
                            // 没有记忆，使用最小电压
                            switch (pdos_ptr->pdo[i].apdo_subtype) {
                                case APDO_TYPE_SPR_PPS:
                                    request_voltage = pdos_ptr->pdo[i].pps.min_voltage;
                                    request_voltage = request_voltage < 4500 ? 4500 : request_voltage;  // 防止电压过低
                                    break;
                                case APDO_TYPE_EPR_AVS:
                                    request_voltage = pdos_ptr->pdo[i].epr_avs.min_voltage;
                                    break;
                                case APDO_TYPE_SPR_AVS:
                                    request_voltage = pdos_ptr->pdo[i].spr_avs.min_voltage;
                                    break;
                                default:
                                    break;
                            }
                        }

                        if (request_voltage > 0) {
                            bool success = usbpd_sink_set_apdo_position_with_voltage(g_power_state.pd_position, request_voltage);
                            if (success) {
                                // 保存请求的电压到记忆中
                                save_state_flag |= 0x2;
                                save_state_flag &= ~0x4;
                                save_apdo_memory_voltage(g_power_state.pd_position, request_voltage);
                            }
                            else
                            {
                                // 请求电压失败了
                                save_state_flag &= ~0x2;
                                save_state_flag &= ~0x4;
                            }
                        }
                    }
                    break;
                }
            }
        }
    }
}

/**
 * @brief 处理下键长按 - 调试打印
 */
static void handle_down_long_button(void) {
    LOG("Control: handle down long press\n");
    usbpd_sink_debug_available_pdos();
    // 写入flash参数
    Flash_Write_State();
}

/**
 * @brief 处理下键双击 - 进入/退出编辑模式
 */
static void handle_down_double_click(void) {
    LOG("Control: handle down double click\n");

    if (g_power_state.power_mode != POWER_MODE_PD) {
        return;  // 仅 PD 模式支持编辑
    }

    // 检查是否为 APDO
    if (!is_apdo(g_power_state.pd_position)) {
        LOG("Select PDO is not APDO, cannot enter edit mode\n");
        return;
    }

    // 切换编辑模式
    g_power_state.is_edit_mode = !g_power_state.is_edit_mode;

    if (g_power_state.is_edit_mode) {
        // 进入编辑模式：优先使用记忆电压，否则使用最小值
        uint16_t min_voltage, max_voltage, step;
        if (get_apdo_voltage_range(g_power_state.pd_position, &min_voltage, &max_voltage, &step)) {
            // 尝试获取记忆的电压
            uint16_t memory_voltage = get_apdo_memory_voltage(g_power_state.pd_position);

            if (memory_voltage >= min_voltage && memory_voltage <= max_voltage) {
                // 使用记忆的电压
                g_power_state.edit_voltage = memory_voltage;
                LOG("Enter edit mode, use memory voltage: %umV\n", g_power_state.edit_voltage);
            } else {
                // 没有有效记忆，使用最小电压
                g_power_state.edit_voltage = min_voltage;
                LOG("Enter edit mode, use min voltage: %umV\n", g_power_state.edit_voltage);
            }
        } else {
            g_power_state.is_edit_mode = false;  // 获取范围失败，退出编辑模式
            LOG("Failed to get voltage range, cannot enter edit mode\n");
        }
    } else {
        // 退出编辑模式
        LOG("Exit edit mode\n");
        g_power_state.edit_voltage = 0;
    }
}

/**
 * @brief 处理按键事件（外部接口）
 */
void app_control_handle_button(button_event_t event) {
    switch (event) {
        case BUTTON_EVENT_LEFT:
            handle_left_button();
            break;
        case BUTTON_EVENT_RIGHT:
            handle_right_button();
            break;
        case BUTTON_EVENT_DOWN:
            handle_down_button();
            break;
        case BUTTON_EVENT_DOWN_LONG:
            handle_down_long_button();
            break;
        case BUTTON_EVENT_DOWN_DOUBLE_CLICK:
            handle_down_double_click();
            break;
        default:
            break;
    }
}

/**
 * @brief 获取当前电源状态
 */
power_state_t app_control_get_power_state(void) {
    power_state_t state = {0};
    memcpy(&state, &g_power_state, sizeof(power_state_t));

    sync_pd_ui_position();
    memcpy(&state, &g_power_state, sizeof(power_state_t));

    // 读取 VBUS 电压
    state.vbus_voltage = adc_get_vbus_mv();
    state.power_mode = g_power_state.power_mode;
    strcpy(state.power_mode_name, "N/A");
    strcpy(state.power_mode_desc, "");

    // QC 模式
    if (state.power_mode == POWER_MODE_QC2) {
        strcpy(state.power_mode_name, "QC2");
        state.qc_voltage = g_power_state.qc_voltage;
        state.negotiate_voltage = QC_VOLTAGE_MAP[g_power_state.qc_voltage];
        state.negotiate_current = 0;  // QC 模式不显示电流
    }

    // PD 模式
    if (state.power_mode == POWER_MODE_PD) {
        strcpy(state.power_mode_name, state.is_epr_ready ? "EPR" : "SPR");
        strcpy(state.power_mode_desc, "");

        const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();

        // 使用 g_power_state.pd_position（用户选择的 position）
        state.pd_position = g_power_state.pd_position;
        state.is_epr_ready = usbpd_sink_get_epr_ready();
        state.is_edit_mode = g_power_state.is_edit_mode;
        state.edit_voltage = g_power_state.edit_voltage;

        // 查找选中的 PDO 信息（用于预览）
        for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
            if (pdos_ptr->pdo[i].position == g_power_state.pd_position) {
                switch (pdos_ptr->pdo[i].pdo_type) {
                    case PDO_TYPE_FIXED_SUPPLY:
                        strcpy(state.power_mode_name, pdos_ptr->pdo[i].fixed.voltage > 20000 ? "EPR" : "SPR");
                        strcpy(state.power_mode_desc, "FIX");
                        state.negotiate_voltage = pdos_ptr->pdo[i].fixed.voltage;
                        state.negotiate_current = pdos_ptr->pdo[i].fixed.current;
                        break;
                    case PDO_TYPE_APDO:
                        if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_SPR_PPS) {
                            strcpy(state.power_mode_name, "SPR");
                            strcpy(state.power_mode_desc, "PPS");
                            // 编辑模式显示编辑电压，否则显示记忆电压（如无记忆则显示最小电压）
                            if (state.is_edit_mode) {
                                state.negotiate_voltage = state.edit_voltage;
                            } else {
                                uint16_t memory_voltage = get_apdo_memory_voltage(g_power_state.pd_position);
                                state.negotiate_voltage = memory_voltage > 0 ? memory_voltage : pdos_ptr->pdo[i].pps.min_voltage;
                            }
                            state.negotiate_current = pdos_ptr->pdo[i].pps.current;
                        }
                        if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_EPR_AVS) {
                            strcpy(state.power_mode_name, "EPR");
                            strcpy(state.power_mode_desc, "AVS");
                            // 编辑模式显示编辑电压，否则显示记忆电压（如无记忆则显示最小电压）
                            if (state.is_edit_mode) {
                                state.negotiate_voltage = state.edit_voltage;
                            } else {
                                uint16_t memory_voltage = get_apdo_memory_voltage(g_power_state.pd_position);
                                state.negotiate_voltage = memory_voltage > 0 ? memory_voltage : pdos_ptr->pdo[i].epr_avs.min_voltage;
                            }
                            state.negotiate_current = (uint16_t)(((uint32_t)pdos_ptr->pdo[i].epr_avs.pdp * 1000000U) / state.negotiate_voltage);
                            state.negotiate_current = state.negotiate_current > 5000 ? 5000 : state.negotiate_current;
                            state.negotiate_epr_avs_pdp = pdos_ptr->pdo[i].epr_avs.pdp;
                        }
                        if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_SPR_AVS) {
                            strcpy(state.power_mode_name, "SPR");
                            strcpy(state.power_mode_desc, "AVS");
                            // 编辑模式显示编辑电压，否则显示记忆电压（如无记忆则显示最小电压）
                            if (state.is_edit_mode) {
                                state.negotiate_voltage = state.edit_voltage;
                            } else {
                                uint16_t memory_voltage = get_apdo_memory_voltage(g_power_state.pd_position);
                                state.negotiate_voltage = memory_voltage > 0 ? memory_voltage : pdos_ptr->pdo[i].spr_avs.min_voltage;
                            }
                            state.negotiate_current = state.negotiate_voltage > 15000 ? pdos_ptr->pdo[i].spr_avs.max_current_15v_20v : pdos_ptr->pdo[i].spr_avs.max_current_9v_15v;
                        }
                        break;
                    default:
                        state.negotiate_voltage = 0;
                        state.negotiate_current = 0;
                        break;
                }
                break;
            }
        }
    }

    return state;
}

/**
 * @brief  CRC32 按位计算函数（直接计算法，不使用查表）
 * @param  data: 待计算数据的指针
 * @param  len:  待计算数据的长度（字节数）
 * @retval 计算得到的32位CRC校验值
 */
uint32_t CRC32_Calculate(const uint8_t *data, int len)
{
    uint32_t crc = 0xFFFFFFFF;  // 初始值
    for (int i = 0; i < len; i++) {
        uint8_t byte = data[i];
        crc ^= byte;  // 将当前字节与CRC低8位异或

        // 对当前字节的8个bit进行逐位计算
        for (int j = 0; j < 8; j++) {
            if (crc & 0x00000001) {             // 检查最低位是否为1
                crc = (crc >> 1) ^ 0xEDB88320;  // 右移一位并与多项式异或
            } else {
                crc >>= 1;  // 右移一位
            }
        }
    }
    return ~crc;  // 最终取反输出
}

/**
 * @brief  读取Flash同步诱骗参数
 */
void Flash_Sync_State(void)
{
    // PD模式下参数恢复
    int pd_maxpower = -1;
    uint8_t pd_position = 0xff;
    if (g_save_state->power_mode == POWER_MODE_PD && g_power_state.power_mode == POWER_MODE_PD) {
        const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();
        for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
            if (pdos_ptr->pdo[i].pdo_type == PDO_TYPE_FIXED_SUPPLY && g_save_state->pdo_type == PDO_TYPE_FIXED_SUPPLY
                && pdos_ptr->pdo[i].fixed.voltage - 10 <= g_save_state->voltage
                && g_save_state->voltage <= pdos_ptr->pdo[i].fixed.voltage + 10) {
                // 固定档位电压合适就可以
                pd_position = pdos_ptr->pdo[i].position;
                save_state_flag |= 0x8;
                break;
            }
            if (pdos_ptr->pdo[i].pdo_type == PDO_TYPE_APDO && g_save_state->pdo_type == PDO_TYPE_APDO) {
                if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_SPR_PPS && g_save_state->apdo_subtype == APDO_TYPE_SPR_PPS
                    && pdos_ptr->pdo[i].pps.min_voltage <= g_save_state->voltage
                    && g_save_state->voltage <= pdos_ptr->pdo[i].pps.max_voltage) {
                    // 选择最大的功率
                    int maxpower = g_save_state->voltage * pdos_ptr->pdo[i].pps.current;
                    if (pd_maxpower < 0 || pd_maxpower <= maxpower) {
                        pd_maxpower = maxpower;
                        pd_position = pdos_ptr->pdo[i].position;
                    }
                }
                if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_EPR_AVS && g_save_state->apdo_subtype == APDO_TYPE_EPR_AVS
                    && pdos_ptr->pdo[i].epr_avs.min_voltage <= g_save_state->voltage
                    && g_save_state->voltage <= pdos_ptr->pdo[i].epr_avs.max_voltage) {
                    // 选择最大的功率
                    int maxpower = pdos_ptr->pdo[i].epr_avs.pdp;
                    if (pd_maxpower < 0 || pd_maxpower <= maxpower) {
                        pd_maxpower = maxpower;
                        pd_position = pdos_ptr->pdo[i].position;
                    }
                }
                if (pdos_ptr->pdo[i].apdo_subtype == APDO_TYPE_SPR_AVS && g_save_state->apdo_subtype == APDO_TYPE_SPR_AVS
                    && pdos_ptr->pdo[i].spr_avs.min_voltage <= g_save_state->voltage
                    && g_save_state->voltage <= pdos_ptr->pdo[i].spr_avs.max_voltage) {
                    // 选择最大的功率
                    int maxpower = g_save_state->voltage * (g_save_state->voltage > 15000 ? pdos_ptr->pdo[i].spr_avs.max_current_15v_20v : pdos_ptr->pdo[i].spr_avs.max_current_9v_15v);
                    if (pd_maxpower < 0 || pd_maxpower <= maxpower) {
                        pd_maxpower = maxpower;
                        pd_position = pdos_ptr->pdo[i].position;
                    }
                }
            }
        }
    }
    if (pd_position != 0xff) {
        // 找到了合适的档位恢复申请档位
        g_power_state.pd_position = pd_position;
        if (g_save_state->pdo_type != PDO_TYPE_FIXED_SUPPLY) {
            // 恢复申请电压
            g_power_state.edit_voltage = g_save_state->voltage;
            save_apdo_memory_voltage(g_power_state.pd_position, g_save_state->voltage);
            save_state_flag |= 0x8;
        }
    }
    // QC模式下参数恢复
    if (g_save_state->power_mode == POWER_MODE_QC2 && g_power_state.power_mode == POWER_MODE_QC2) {
        if (g_save_state->qc_voltage <= USB_QC_VOLTAGE_20V) {
            // 恢复申请电压
            g_power_state.qc_voltage = g_save_state->qc_voltage;
            save_state_flag |= 0x8;
        }
    }
}

/**
 * @brief  写入Flash，写入前计算CRC32数据
 */
void Flash_Write_State(void)
{
    uint32_t address = FLASH_BASE + 0xC000 - 256;
    // 存储当前的诱骗数据
    power_state_t state = app_control_get_power_state();
    if (state.power_mode != POWER_MODE_NONE) {
        // 工作在诱骗状态才能存储
        g_save_state->power_mode = state.power_mode;
        g_save_state->is_epr_ready = state.is_epr_ready;
        g_save_state->pdo_type = PDO_TYPE_FIXED_SUPPLY;
        g_save_state->apdo_subtype = APDO_TYPE_SPR_PPS;
        if (state.power_mode == POWER_MODE_PD) {
            const pd_available_pdos_t *pdos_ptr = usbpd_sink_get_available_pdos();
            for (uint8_t i = 0; i < pdos_ptr->pdo_count; i++) {
                if (pdos_ptr->pdo[i].position == state.pd_position) {
                    // 存储PDO类型信息
                    g_save_state->pdo_type = pdos_ptr->pdo[i].pdo_type;
                    g_save_state->apdo_subtype = pdos_ptr->pdo[i].apdo_subtype;
                    break;
                }
            }
        }
        g_save_state->qc_voltage = state.qc_voltage;
        g_save_state->rotate_mode = (save_state_flag & 0x10) ? 1 : 0;
        g_save_state->voltage = state.negotiate_voltage;
    }
    // 存储当前的校准数据
    float vbus_uncalibrated = (float)adc_get_raw() * VBUS_CONVERT_SCALE;
    for (int i = 0; (state.power_mode == POWER_MODE_NONE) && (save_state_flag & 0x1) && (i < VBUS_CAL_POINTS_COUNT); i++) {
        if (VBUS_CAL_POINTS[i].actual - 1000 <= vbus_uncalibrated && vbus_uncalibrated <= VBUS_CAL_POINTS[i].actual + 1000) {
            // 校准时才能存储
            VBUS_CAL_POINTS[i].measured = g_save_state->vbus_calc_point[i] = vbus_uncalibrated;
            break;
        }
    }
    g_save_state->save_state_crc = CRC32_Calculate(flash_buf, sizeof(save_state_t) - 4);
    FLASH_Unlock_Fast();
    FLASH_ErasePage_Fast(address);
    for (int i = 0; i < 256; i += 4) {
        // 地址从目标地址开始，每次增加4字节
        FLASH_BufLoad(address + i, *(uint32_t *)(&flash_buf[i]));
    }
    FLASH_ProgramPage_Fast(address);
    FLASH_Lock_Fast();
    // 闪一下屏幕作为回应
    extern void app_ui_clear(void);
    app_ui_clear();
}