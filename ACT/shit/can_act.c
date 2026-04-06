#include "can_act.h"
#include "drive_mode.h"   /* updateVehicleSpeed, updateBrakeStateCan */
#include "IfxPort.h"
#include "IfxStm.h"
#include "Bsp.h"
#include "mpu6050.h"

/* ── 전역 정의 ── */
IfxCan_Can      g_mcmcan;
IfxCan_Can_Node g_canNode0;

volatile uint8  g_canBrakeCmd  = BRAKE_CMD_RELEASE;
volatile uint8  g_canGearState = CAN_GEAR_P;
volatile uint8  g_canCmdValid  = 0U;
volatile uint32 g_lastRxTick   = 0U;

volatile uint32 g_vehicleSpeed  = 0U;
volatile uint8  g_brakeStateCan = BRAKE_CMD_RELEASE;

volatile uint8  g_txSuccess = 0U, g_txBusy = 0U, g_txFail = 0U;
volatile uint32 g_txCount   = 0U;
volatile uint8  g_rxNew     = 0U, g_rxFail = 0U;
volatile uint32 g_rxCount   = 0U;
volatile uint32 g_rxData0   = 0U, g_rxData1 = 0U;

/* ── 내부 전용 ── */
static void initStandardFilter(uint32 filterId, uint8 rxBufferNumber)
{
    IfxCan_Filter filter;
    filter.number                = 0;
    filter.elementConfiguration  = IfxCan_FilterElementConfiguration_storeInRxBuffer;
    filter.type                  = IfxCan_FilterType_classic;
    filter.id1                   = filterId;
    filter.id2                   = 0x7FF;
    filter.rxBufferOffset        = (IfxCan_RxBufferId)rxBufferNumber;
    IfxCan_Can_setStandardFilter(&g_canNode0, &filter);
}

void initCanAct(void)
{
    IfxPort_setPinModeOutput(CAN_STB_PORT, CAN_STB_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(CAN_STB_PORT, CAN_STB_PIN);

    {
        IfxCan_Can_Config canConfig;
        IfxCan_Can_initModuleConfig(&canConfig, &MODULE_CAN0);
        IfxCan_Can_initModule(&g_mcmcan, &canConfig);
    }
    {
        IfxCan_Can_NodeConfig canNodeConfig;
        IfxCan_Can_initNodeConfig(&canNodeConfig, &g_mcmcan);

        canNodeConfig.nodeId              = IfxCan_NodeId_0;
        canNodeConfig.baudRate.baudrate   = 500000;
        canNodeConfig.frame.type          = IfxCan_FrameType_transmitAndReceive;

        static const IfxCan_Can_Pins canPins = {
            &IfxCan_TXD00_P20_8_OUT, IfxPort_OutputMode_pushPull,
            &IfxCan_RXD00B_P20_7_IN, IfxPort_InputMode_pullUp,
            IfxPort_PadDriver_cmosAutomotiveSpeed1
        };
        canNodeConfig.pins = &canPins;

        canNodeConfig.txConfig.txMode                   = IfxCan_TxMode_dedicatedBuffers;
        canNodeConfig.txConfig.dedicatedTxBuffersNumber = 1;
        canNodeConfig.txConfig.txBufferDataFieldSize    = IfxCan_DataFieldSize_8;
        canNodeConfig.txConfig.txEventFifoSize          = 0;

        canNodeConfig.rxConfig.rxMode                   = IfxCan_RxMode_dedicatedBuffers;
        canNodeConfig.rxConfig.rxBufferDataFieldSize    = IfxCan_DataFieldSize_8;
        canNodeConfig.rxConfig.rxFifo0DataFieldSize     = IfxCan_DataFieldSize_8;
        canNodeConfig.rxConfig.rxFifo1DataFieldSize     = IfxCan_DataFieldSize_8;

        IfxCan_Can_initNode(&g_canNode0, &canNodeConfig);
    }

    initStandardFilter(CAN_RX_ID, 0);
}

void receive_main_command(void)
{
    IfxCan_Message rxMsg;
    uint32 rxData[2] = {0U, 0U};

    g_rxFail = 0U;
    g_rxNew  = 0U;

    if (IfxCan_Can_isNewDataReceived(&g_canNode0, (IfxCan_RxBufferId)0) == FALSE)
    {
        g_rxFail = 1U;
        return;
    }

    IfxCan_Can_initMessage(&rxMsg);
    rxMsg.messageId       = 0xFFFFFFFFU;
    rxMsg.bufferNumber    = 0;
    rxMsg.frameMode       = IfxCan_FrameMode_standard;
    rxMsg.messageIdLength = IfxCan_MessageIdLength_standard;
    rxMsg.dataLengthCode  = IfxCan_DataLengthCode_2;
    rxMsg.readFromRxFifo0 = FALSE;

    IfxCan_Can_readMessage(&g_canNode0, &rxMsg, rxData);

    if (rxMsg.messageId == CAN_RX_ID)
    {
        g_rxData0 = rxData[0];
        g_rxData1 = rxData[1];
        g_rxCount++;
        g_rxNew = 1U;

        g_canBrakeCmd  = (uint8)(rxData[0] & 0xFFU);
        g_canGearState = (uint8)((rxData[0] >> 8) & 0xFFU);

        g_canCmdValid = 1U;
        g_lastRxTick  = IfxStm_getLower(BSP_DEFAULT_TIMER);
    }
    else
    {
        g_rxFail = 1U;
    }
}

void checkCanTimeout(void)
{
    if (g_canCmdValid == 0U) return;

    uint32 now       = IfxStm_getLower(BSP_DEFAULT_TIMER);
    uint32 elapsedUs = (now - g_lastRxTick) / g_stmTicksPerUs;

    if (elapsedUs >= CAN_TIMEOUT_US)
    {
        g_canCmdValid  = 0U;
        g_canBrakeCmd  = BRAKE_CMD_RELEASE;
        g_canGearState = CAN_GEAR_P;
        g_gearMode     = GEAR_P;
    }
}

void send_act_status(void)
{
    IfxCan_Message txMsg;
    uint32 txData[2];
    IfxCan_Status status;
    sint32 timeout = 10000;

    g_txSuccess = 0U; g_txBusy = 0U; g_txFail = 0U;

    updateVehicleSpeed();
    updateBrakeStateCan();

    /*
     * 0x300 프레임 구조 (8바이트, little-endian)
     * txData[0] → Byte 0~3 : speed       (uint32, km/h x100)
     * txData[1] → Byte 4   : brake_state (uint8)
     *           → Byte 5   : accel_x     (sint8, g x100, ±100)
     *           → Byte 6   : accel_y     (sint8, g x100, ±100)
     *           → Byte 7   : accel_z     (sint8, g x100, ±100)
     */
    txData[0] = g_vehicleSpeed;
    txData[1] = ((uint32)g_brakeStateCan       & 0xFFU)
              | ((uint32)(uint8)g_accelX  <<  8U)
              | ((uint32)(uint8)g_accelY  << 16U)
              | ((uint32)(uint8)g_accelZ  << 24U);

    IfxCan_Can_initMessage(&txMsg);
    txMsg.messageId       = CAN_TX_ID;
    txMsg.bufferNumber    = 0;
    txMsg.frameMode       = IfxCan_FrameMode_standard;
    txMsg.messageIdLength = IfxCan_MessageIdLength_standard;
    txMsg.dataLengthCode  = IfxCan_DataLengthCode_8;   /* 5 → 8 변경 */
    txMsg.readFromRxFifo0 = FALSE;

    do {
        status = IfxCan_Can_sendMessage(&g_canNode0, &txMsg, txData);
        if (status == IfxCan_Status_ok) { g_txSuccess = 1U; g_txCount++; return; }
        if (status == IfxCan_Status_notSentBusy) g_txBusy = 1U;
        timeout--;
    } while (timeout > 0);

    g_txFail = 1U;
}
