#include "usbpd_sink.h"
#include "usbpd_def.h"
#include "usbpd_vdm.h"
#include <stdint.h>
#include "ch32x035.h"
#include "utils_print.h"
#include "utils_delay.h"

static pd_control_t pd_control_g;
static uint8_t usbpd_rx_buffer[USBPD_DATA_MAX_LEN] __attribute__((aligned(4)));
static uint8_t usbpd_tx_buffer[USBPD_DATA_MAX_LEN] __attribute__((aligned(4)));

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/******************************************************************************
 * Basic Function
 *****************************************************************************/

static void usbpd_sink_rx_mode(void) {
    // 设置 CC 以正常 VDD 电压驱动输出
    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;

    // 清除所有中断标志位
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;

    // 中断使能
    USBPD->CONFIG |= IE_TX_END | IE_RX_ACT | IE_RX_RESET;
    // PD 引脚输入滤波使能？
    USBPD->CONFIG |= PD_FILT_ED;
    // DMA 使能
    USBPD->CONFIG |= PD_DMA_EN;

    // 设置为接收模式
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;     // BMC 接收采样时钟计数器
    USBPD->DMA = (uint32_t)usbpd_rx_buffer;  // DMA Buffer

    // 开始接收
    USBPD->CONTROL &= ~PD_TX_EN;  // PD 接收使能
    USBPD->CONTROL |= BMC_START;  // BMC 开始信号

    NVIC_EnableIRQ(USBPD_IRQn);
}

static void usbpd_sink_state_reset(void) {
    NVIC_DisableIRQ(USBPD_IRQn);

    // 重置 PD 版本
    pd_control_g.pd_version = DEF_PD_REVISION_30;

    // 重置状态机
    pd_control_g.pd_state = PD_STATE_CHECK_CONNECT;
    pd_control_g.pd_last_state = PD_STATE_CHECK_CONNECT;

    // 重置 CC 计数
    pd_control_g.cc_none_times = 0;
    pd_control_g.cc1_connect_times = 0;
    pd_control_g.cc2_connect_times = 0;

    // 重置 Message ID
    pd_control_g.sink_message_id = 0;
    pd_control_g.cable_message_id = 0;

    pd_control_g.port_data_role = 0;
}

static void usbpd_sink_phy_send_data(const uint8_t *tx_buffer, uint8_t tx_length, uint8_t sop) {
    delay_us(90);  // 确保 GoodCRC 已发送

    // 设置 CC 以低电压驱动输出
    if ((USBPD->CONFIG & CC_SEL) == CC_SEL) {
        USBPD->PORT_CC2 |= CC_LVE;
    } else {
        USBPD->PORT_CC1 |= CC_LVE;
    }

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
    USBPD->TX_SEL = sop;
    USBPD->DMA = (uint32_t)tx_buffer;
    USBPD->BMC_TX_SZ = tx_length;

    USBPD->STATUS |= IF_TX_END;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->CONTROL |= BMC_START;

    // 等待发送完成
    while ((USBPD->STATUS & IF_TX_END) == 0);
    USBPD->STATUS |= IF_TX_END;

    usbpd_sink_rx_mode();
}

/******************************************************************************
 * Initialization Function
 *****************************************************************************/

static void timer_init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;  // 1ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 48 - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_SetPriority(TIM3_IRQn, 0x10);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM_Cmd(TIM3, ENABLE);
}

void usbpd_sink_init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;

    // 清除全部状态
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;

    // PD Sink
    USBPD->PORT_CC1 = 0;
    USBPD->PORT_CC2 = 0;

    usbpd_sink_rx_mode();

    timer_init();
}

/******************************************************************************
 * SPR Mode Function
 * EPR Mode Function
 *****************************************************************************/

// TODO: 构建发送请求分散到多个函数
// NOT_SUPPORTED
// REJECT
// VDM_ACK_DISCOVER_IDENTITY
// VDM_ACK_DISCOVER_SVIDS
// SEND_EPR_ENTER
// SEND_EPR_SRC_CAP_REQ_CHUNK

static void usbpd_sink_send_goodcrc(uint8_t message_id, bool is_cable) {
    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageType = USBPD_CONTROL_MSG_GOODCRC;
    header.MessageHeader.PortDataRole = pd_control_g.port_data_role;
    header.MessageHeader.SpecificationRevision = pd_control_g.pd_version;
    header.MessageHeader.PortPowerRole_CablePlug = is_cable;  // Cable Plug
    header.MessageHeader.MessageID = message_id;              // GoodCRC 回复相同的 MessageID
    header.MessageHeader.NumberOfDataObjects = 0;
    header.MessageHeader.Extended = 0;

    *(uint16_t *)&usbpd_tx_buffer[0] = header.d16;
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 2, is_cable ? UPD_SOP1 : UPD_SOP0);
}

void usbpd_sink_hard_reset(void) {
    usbpd_sink_phy_send_data(usbpd_tx_buffer, 0, UPD_HARD_RESET);
    usbpd_sink_state_reset();
}

/**
 * @brief PD 状态机处理函数
 * @note 需在 TIM3_IRQHandler 中调用
 */
static void usbpd_sink_state_process(void) {
    pd_state_t _pd_state = pd_control_g.pd_state;

    if (pd_control_g.pd_last_state != _pd_state) {
        pd_printf("PD State: %d -> %d\n", pd_control_g.pd_last_state, pd_control_g.pd_state);
    }

    switch (pd_control_g.pd_state) {
        case PD_STATE_DISCONNECTED: {
            usbpd_sink_state_reset();
            break;
        }
        case PD_STATE_CONNECT: {
            // 确保进入此状态后只执行一次
            if (pd_control_g.pd_last_state != pd_control_g.pd_state) {
                usbpd_sink_rx_mode();
            }
            break;
        }
        default: {
            break;
        }
    }

    pd_control_g.pd_last_state = _pd_state;
}

/**
 * @brief 解析 PD SOP1 数据包，并回复 GoodCRC
 * @note 需在 USBPD_IRQHandler 中调用
 */
static void usbpd_sink_protocol_analysis_sop1(const uint8_t *rx_buffer, uint8_t rx_length) {
    // Header
    USBPD_MessageHeader_t header = {0};
    header.d16 = *(uint16_t *)rx_buffer;

    // 如果当前收到消息非 GoodCRC，则需先回复 GoodCRC
    bool is_goodcrc_msg = (header.MessageHeader.Extended == 0) &&
                          (header.MessageHeader.NumberOfDataObjects == 0) &&
                          (header.MessageHeader.MessageType == USBPD_CONTROL_MSG_GOODCRC);
    if (!is_goodcrc_msg) {
        usbpd_sink_send_goodcrc(header.MessageHeader.MessageID, true);
    }

    // Non-Extended message
    if (header.MessageHeader.Extended == 0) {

        // Control Message (A Message is defined as a Control Message when the Number of Data Objects field in the Message Header is set to zero)
        if (header.MessageHeader.NumberOfDataObjects == 0) {
            switch (header.MessageHeader.MessageType) {
                case USBPD_CONTROL_MSG_GOODCRC: {
                    // 消息 ID 自增
                    pd_control_g.cable_message_id = (pd_control_g.cable_message_id + 1) & 0x07;
                    break;
                }
                default: {
                    break;
                }
            }
        }

        // Data Message
        if (header.MessageHeader.NumberOfDataObjects > 0) {
            // MessageType
            switch (header.MessageHeader.MessageType) {
                case USBPD_DATA_MSG_VENDOR_DEFINED: {
                    USBPD_StructuredVDMHeader_t vdm_header = {0};
                    vdm_header.d32 = *(uint32_t *)&rx_buffer[2];

                    // 需要抢先在 E-Marker 前回复，开日志的话，会导致比 E-Marker 回复慢
                    // pd_printf("SOP1 VDM:\n");
                    // pd_printf("  Header: 0x%08x\n", vdm_header.d32);
                    // pd_printf("  SVID: 0x%04x\n", vdm_header.StructuredVDMHeader.SVID);
                    // pd_printf("  Type: %d\n", vdm_header.StructuredVDMHeader.VDMType);
                    // pd_printf("  Version: %d %d\n", vdm_header.StructuredVDMHeader.StructuredVDMVersionMajor, vdm_header.StructuredVDMHeader.StructuredVDMVersionMinor);
                    // pd_printf("  Object Position: %d\n", vdm_header.StructuredVDMHeader.ObjectPosition);
                    // pd_printf("  Command Type: %d\n", vdm_header.StructuredVDMHeader.CommandType);
                    // pd_printf("  Command: %d\n", vdm_header.StructuredVDMHeader.Command);

                    if ((vdm_header.StructuredVDMHeader.VDMType == 1 /* Structured_VDM */) && (vdm_header.StructuredVDMHeader.CommandType == 0b00 /* REQ */)) {
                        // VDM Command
                        switch (vdm_header.StructuredVDMHeader.Command) {
                            case 1: {  //  Discover Identity
                                USBPD_MessageHeader_t _header;
                                _header.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
                                _header.MessageHeader.PortDataRole = 0;
                                _header.MessageHeader.SpecificationRevision = USBPD_SPECIFICATION_REV3;
                                _header.MessageHeader.PortPowerRole_CablePlug = 1;
                                _header.MessageHeader.MessageID = pd_control_g.cable_message_id;
                                _header.MessageHeader.NumberOfDataObjects = 5;
                                _header.MessageHeader.Extended = 0;

                                *(uint16_t *)&usbpd_tx_buffer[0] = _header.d16;

                                // 酷态科磁吸短线 E-Marker
                                // *(uint32_t *)&usbpd_tx_buffer[2] = 0xFF00A041;
                                // *(uint32_t *)&usbpd_tx_buffer[6] = 0x18002B01;
                                // *(uint32_t *)&usbpd_tx_buffer[10] = 0x00000000;
                                // *(uint32_t *)&usbpd_tx_buffer[14] = 0x40916D14;
                                // *(uint32_t *)&usbpd_tx_buffer[18] = 0x000A0640;

                                // 联想 C135 E-Marker
                                *(uint32_t *)&usbpd_tx_buffer[2] = usbpd_vdm_pack_structured_header(USBPD_SID_USBIF,
                                                                                                      1,
                                                                                                      0,
                                                                                                      0,
                                                                                                      VDM_COMMAND_TYPE_ACK,
                                                                                                      VDM_COMMAND_DISCOVER_IDENTITY);  // VDM Header
                                *(uint32_t *)&usbpd_tx_buffer[6] = 0x180017EF;   // ID Header VDO
                                *(uint32_t *)&usbpd_tx_buffer[10] = 0x00000000;  // Cert Stat VDO
                                *(uint32_t *)&usbpd_tx_buffer[14] = 0xA4AA0000;  // Product VDO
                                // *(uint32_t *)&usbpd_tx_buffer[18] = 0x110A2643;  // Product Type VDO(s)

                                USBPD_PassiveCableVDO_t vdo = {0};
                                vdo.PassiveCableVDO.USBHighestSpeed = 0b100;      // 000b = [USB 2.0] only, no SuperSpeed support
                                                                                  // 001b = [USB 3.2] Gen1
                                                                                  // 010b = [USB 3.2]/[USB4] Gen2
                                                                                  // 011b = [USB4] Gen3
                                                                                  // 100b = [USB4] Gen4
                                vdo.PassiveCableVDO.VBUSCurrentHandling = 0b10;   // 01b = 3A
                                                                                  // 10b = 5A
                                vdo.PassiveCableVDO.MaxVBUSVoltage = 0b11;        // 00b – 20V
                                                                                  // 01b – 30V (Deprecated)
                                                                                  // 10b – 40V (Deprecated)
                                                                                  // 11b – 50V
                                vdo.PassiveCableVDO.CableTerminationType = 0b00;  // 00b = VCONN not required. Cable Plugs that only support Discover Identity Commands Shall set these bits to 00b.
                                                                                  // 01b = VCONN required
                                vdo.PassiveCableVDO.CableLatency = 0b1000;        // 0001b – <10ns (~1m)
                                                                                  // 0010b – 10ns to 20ns (~2m)
                                                                                  // 0011b – 20ns to 30ns (~3m)
                                                                                  // 0100b – 30ns to 40ns (~4m)
                                                                                  // 0101b – 40ns to 50ns (~5m)
                                                                                  // 0110b – 50ns to 60ns (~6m)
                                                                                  // 0111b – 60ns to 70ns (~7m)
                                                                                  // 1000b – > 70ns (>~7m)
                                vdo.PassiveCableVDO.EPRCapable = 0b1;             // 0b – Cable is not EPR Capable
                                                                                  // 1b = Cable is EPR Capable
                                vdo.PassiveCableVDO.USBTypeCPlug = 0b10;          // 10b = USB Type-C
                                                                                  // 11b = Captive
                                vdo.PassiveCableVDO.VDOVersion = 0b000;           // Version 1.0 = 000b
                                vdo.PassiveCableVDO.FirmwareVersion = 1;
                                vdo.PassiveCableVDO.HWVersion = 1;

                                *(uint32_t *)&usbpd_tx_buffer[18] = vdo.d32;

                                // pd_printf("PassiveCableVDO:\n");
                                // pd_printf("  USBHighestSpeed: %d\n", vdo.PassiveCableVDO.USBHighestSpeed);
                                // pd_printf("  VBUSCurrentHandling: %d\n", vdo.PassiveCableVDO.VBUSCurrentHandling);
                                // pd_printf("  MaxVBUSVoltage: %d\n", vdo.PassiveCableVDO.MaxVBUSVoltage);
                                // pd_printf("  CableTerminationType: %d\n", vdo.PassiveCableVDO.CableTerminationType);
                                // pd_printf("  CableLatency: %d\n", vdo.PassiveCableVDO.CableLatency);
                                // pd_printf("  EPRCapable: %d\n", vdo.PassiveCableVDO.EPRCapable);
                                // pd_printf("  USBTypeCPlug: %d\n", vdo.PassiveCableVDO.USBTypeCPlug);
                                // pd_printf("  VDOVersion: %d\n", vdo.PassiveCableVDO.VDOVersion);
                                // pd_printf("  FirmwareVersion: %d\n", vdo.PassiveCableVDO.FirmwareVersion);
                                // pd_printf("  HWVersion: %d\n", vdo.PassiveCableVDO.HWVersion);

                                usbpd_sink_phy_send_data(usbpd_tx_buffer, (2 + 4 * 5), UPD_SOP1);
                                break;
                            }
                            case 2:  // Discover SVIDs
                            default: {
                                break;
                            }
                        }
                    }
                    break;
                }
                default: {
                    break;
                }
            }
        }
    }
}

/******************************************************************************
 * Interrupt Handler
 *****************************************************************************/

void USBPD_IRQHandler(void) {
    if (USBPD->STATUS & IF_RX_ACT) {
        USBPD->STATUS |= IF_RX_ACT;

        if (USBPD->BMC_BYTE_CNT >= (2 /*Header*/)) {
            if ((USBPD->STATUS & BMC_AUX_Mask) == BMC_AUX_SOP1_HRST) {  //  SOP1 用于实现模拟 E-Marker
                usbpd_sink_protocol_analysis_sop1(usbpd_rx_buffer, USBPD->BMC_BYTE_CNT);
            }
        }
    }

    if (USBPD->STATUS & IF_TX_END) {
        USBPD->STATUS |= IF_TX_END;
        usbpd_sink_rx_mode();
    }

    if (USBPD->STATUS & IF_RX_RESET) {
        USBPD->STATUS |= IF_RX_RESET;
        usbpd_sink_state_reset();
    }
}

void TIM3_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    // 在 PD_STATE_CHECK_CONNECT 状态时检测 CC 连接
    if (pd_control_g.pd_state == PD_STATE_CHECK_CONNECT) {
        pd_control_g.cc1_connect_times = 0;
        pd_control_g.cc2_connect_times = 0;
        pd_control_g.pd_state = PD_STATE_CONNECT;
        USBPD->CONFIG |= CC_SEL;
        pd_printf("Attach: CC2\n");
    }

    // 处理状态机
    usbpd_sink_state_process();
}
