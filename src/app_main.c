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
    LOG("USB PD EMarker\n");

    // 初始化 PD Sink
    usbpd_sink_init();
    while (1) {
        delay_ms(100);
    }
}