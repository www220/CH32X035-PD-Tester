#ifndef CONFIG_NO_SCREEN_MAX_POWER_MODE

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ch32x035.h"
#include "utils_print.h"
#include "utils_delay.h"

#include "app_control.h"
#include "app_button.h"
#include "app_ui.h"

/**
 * @brief 按键事件处理器（传递按键事件到 app_control / app_ui）
 */
static void button_event_handler(button_event_t event) {
    // 特殊处理：控制 UI 旋转（校准状态下作为确认按钮，输出状态下作为确认按钮）
    if (event == BUTTON_EVENT_DOWN_LONG && !(save_state_flag & 0x1) && !(save_state_flag &0x2)) {
        app_ui_toggle_rotation();
        if (save_state_flag & 0x10) save_state_flag &= ~0x10; else save_state_flag |= 0x10;
    }
    // 将按键事件传递给 app_control 处理
    app_control_handle_button(event);
}

int main(void) {
    // 系统初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    delay_init();
    print_init(921600);
    LOG("USB PD/QC Tester\n");

    // 初始化各层模块
    app_control_init();  // 此处包含 pd 初始化，需要尽量靠前，上电后需要抢在 E-Marker 前回复第一个 SOP1 VDM
    if (g_save_state->rotate_mode) {
        // 屏幕方向旋转
        app_ui_toggle_rotation();
        save_state_flag |= 0x10;
    } else {
        // 默认屏幕方向正常
        app_ui_init();
    }
    app_button_init(button_event_handler);

    while (1) {
        // 按键处理
        static uint32_t last_btn_millis = 0;
        if (millis() - last_btn_millis >= 5) {
            last_btn_millis = millis();
            app_button_process();
        }

        // UI 更新
        static uint32_t last_ui_millis = 0;
        if (millis() - last_ui_millis >= 100) {
            last_ui_millis = millis();

            // 从 app_control 获取状态
            power_state_t state = app_control_get_power_state();

            // 转换为 UI 数据格式
            ui_display_data_t ui_data = {0};
            // 可以进入校准模式
            if (state.vbus_voltage >= 30000) save_state_flag |= 0x1;
            if ((state.power_mode == POWER_MODE_NONE) && (save_state_flag & 0x1)) strcpy(state.power_mode_name, "CAL");
            // 输出功率时点亮输出灯
            if (save_state_flag & 0x2) GPIO_SetBits(GPIOA, GPIO_Pin_6); else GPIO_ResetBits(GPIOA, GPIO_Pin_6);
            // 读取flash参数同步诱骗参数
            if ((g_save_state->power_mode && g_save_state->power_mode == state.power_mode)
                && (g_save_state->power_mode == POWER_MODE_PD && g_save_state->is_epr_ready && state.is_epr_ready) 
                && !(save_state_flag & 0x8)) {
                // epr模式需要一点时间进入开机不会直接就同步成功
                Flash_Sync_State();
            }
            //
            ui_data.vbus_voltage_mv = state.vbus_voltage;
            ui_data.negotiate_voltage_mv = state.negotiate_voltage;
            ui_data.negotiate_current_ma = state.negotiate_current;
            ui_data.negotiate_epr_avs_pdp = state.negotiate_epr_avs_pdp;
            ui_data.is_edit_mode = state.is_edit_mode;
            strncpy(ui_data.mode_name, state.power_mode_name, sizeof(ui_data.mode_name) - 1);
            ui_data.mode_name[sizeof(ui_data.mode_name) - 1] = '\0';
            strncpy(ui_data.mode_desc, state.power_mode_desc, sizeof(ui_data.mode_desc) - 1);
            ui_data.mode_desc[sizeof(ui_data.mode_desc) - 1] = '\0';

            // 更新 UI
            app_ui_update(&ui_data);
        }
    }
}

#else

#include <stdbool.h>
#include <stdint.h>
#include "ch32x035.h"
#include "utils_print.h"
#include "utils_delay.h"
#include "usbpd_sink.h"

int main(void) {
    // 系统初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    delay_init();
    print_init(921600);
    LOG("USB PD/QC Tester\n");

    // 初始化 I2C 引脚
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
        GPIO_InitTypeDef GPIO_InitStructure = {0};
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_18 | GPIO_Pin_19;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    }

    // 初始化 PD Sink
    usbpd_sink_init();

    // 等待 PD Sink 就绪
    while (!usbpd_sink_get_ready()) {
        delay_ms(10);
    }

    delay_ms(2000);

    // 查找最大功率 PDO
    uint8_t position;
    uint16_t voltage_mv;
    bool is_fpdo;

    if (usbpd_sink_find_max_power_pdo(&position, &voltage_mv, &is_fpdo)) {
        LOG("Max Power PDO: position=%u, voltage=%umV, is_fpdo=%d\n", position, voltage_mv, is_fpdo);

        if (is_fpdo) {
            usbpd_sink_set_fpdo_position(position);
        } else {
            usbpd_sink_set_apdo_position_with_voltage(position, voltage_mv);
        }
    } else {
        LOG("No PDO available\n");
    }

    while (1) {
        delay_ms(100);
    }
}

#endif