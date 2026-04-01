/**********************************************************************************************************************
 * \file Cpu0_Main.c
 * TC375 Lite Kit - CAN Response Board
 *********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxStm.h"
#include "Bsp.h"

#include "IfxPort.h"
#include "IfxPort_PinMap.h"

#include "IfxCan.h"
#include "IfxCan_Can.h"

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

/* ---------------- LED ---------------- */
#define LED_TX_PORT   &MODULE_P10
#define LED_TX_PIN    2
#define LED_RX_PORT   &MODULE_P10
#define LED_RX_PIN    1

/* ---------------- CAN STB ---------------- */
#define CAN_STB_PORT  &MODULE_P20
#define CAN_STB_PIN   6

/* ---------------- CAN ID ---------------- */
#define MY_TX_ID      0x456
#define MY_RX_ID      0x655

/* ---------------- Global CAN handle ---------------- */
IfxCan_Can      g_mcmcan;
IfxCan_Can_Node g_canNode0;

/* ---------------- TX debug ---------------- */
volatile uint8  g_txSuccess = 0;
volatile uint8  g_txBusy    = 0;
volatile uint8  g_txFail    = 0;
volatile uint32 g_txId      = 0;
volatile uint32 g_txData0   = 0;
volatile uint32 g_txData1   = 0;
volatile uint32 g_txCount   = 0;

/* ---------------- RX debug ---------------- */
volatile uint8  g_rxNew     = 0;
volatile uint8  g_rxFail    = 0;
volatile uint32 g_rxId      = 0;
volatile uint32 g_rxData0   = 0;
volatile uint32 g_rxData1   = 0;
volatile uint32 g_rxCount   = 0;

static void initLeds(void);
static void initCanTxRx(void);
static void initStandardFilter(uint32 filterId, uint8 rxBufferNumber);
static void send_can_message(uint32 data0, uint32 data1);
static void receive_can_message(void);

void core0_main(void)
{
    IfxCpu_enableInterrupts();

    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    initLeds();
    initCanTxRx();

    while (1)
    {
        receive_can_message();
    }
}

static void initLeds(void)
{
    IfxPort_setPinHigh(LED_TX_PORT, LED_TX_PIN);
    IfxPort_setPinModeOutput(LED_TX_PORT, LED_TX_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinPadDriver(LED_TX_PORT, LED_TX_PIN,
                            IfxPort_PadDriver_cmosAutomotiveSpeed1);

    IfxPort_setPinHigh(LED_RX_PORT, LED_RX_PIN);
    IfxPort_setPinModeOutput(LED_RX_PORT, LED_RX_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinPadDriver(LED_RX_PORT, LED_RX_PIN,
                            IfxPort_PadDriver_cmosAutomotiveSpeed1);
}

static void initCanTxRx(void)
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

        canNodeConfig.nodeId = IfxCan_NodeId_0;
        canNodeConfig.baudRate.baudrate = 500000;
        canNodeConfig.frame.type = IfxCan_FrameType_transmitAndReceive;

        static const IfxCan_Can_Pins canPins = {
            &IfxCan_TXD00_P20_8_OUT,
            IfxPort_OutputMode_pushPull,
            &IfxCan_RXD00B_P20_7_IN,
            IfxPort_InputMode_pullUp,
            IfxPort_PadDriver_cmosAutomotiveSpeed1
        };
        canNodeConfig.pins = &canPins;

        canNodeConfig.txConfig.txMode = IfxCan_TxMode_dedicatedBuffers;
        canNodeConfig.txConfig.dedicatedTxBuffersNumber = 1;
        canNodeConfig.txConfig.txBufferDataFieldSize = IfxCan_DataFieldSize_8;
        canNodeConfig.txConfig.txEventFifoSize = 0;

        canNodeConfig.rxConfig.rxMode = IfxCan_RxMode_dedicatedBuffers;
        canNodeConfig.rxConfig.rxBufferDataFieldSize = IfxCan_DataFieldSize_8;
        canNodeConfig.rxConfig.rxFifo0DataFieldSize = IfxCan_DataFieldSize_8;
        canNodeConfig.rxConfig.rxFifo1DataFieldSize = IfxCan_DataFieldSize_8;

        IfxCan_Can_initNode(&g_canNode0, &canNodeConfig);
    }

    initStandardFilter(MY_RX_ID, 0);
}

static void initStandardFilter(uint32 filterId, uint8 rxBufferNumber)
{
    IfxCan_Filter filter;

    filter.number = 0;
    filter.elementConfiguration = IfxCan_FilterElementConfiguration_storeInRxBuffer;
    filter.type = IfxCan_FilterType_classic;
    filter.id1 = filterId;
    filter.id2 = 0x7FF;
    filter.rxBufferOffset = (IfxCan_RxBufferId)rxBufferNumber;

    IfxCan_Can_setStandardFilter(&g_canNode0, &filter);
}

static void send_can_message(uint32 data0, uint32 data1)
{
    IfxCan_Message txMsg;
    uint32 txData[2];
    IfxCan_Status status;
    sint32 timeout = 10000;

    g_txSuccess = 0;
    g_txBusy    = 0;
    g_txFail    = 0;

    txData[0] = data0;
    txData[1] = data1;

    g_txId    = MY_TX_ID;
    g_txData0 = txData[0];
    g_txData1 = txData[1];

    IfxCan_Can_initMessage(&txMsg);
    txMsg.messageId       = MY_TX_ID;
    txMsg.bufferNumber    = 0;
    txMsg.frameMode       = IfxCan_FrameMode_standard;
    txMsg.messageIdLength = IfxCan_MessageIdLength_standard;
    txMsg.dataLengthCode  = IfxCan_DataLengthCode_8;
    txMsg.readFromRxFifo0 = FALSE;

    do
    {
        status = IfxCan_Can_sendMessage(&g_canNode0, &txMsg, txData);

        if (status == IfxCan_Status_ok)
        {
            g_txSuccess = 1;
            g_txCount++;
            return;
        }

        if (status == IfxCan_Status_notSentBusy)
        {
            g_txBusy = 1;
        }

        timeout--;
    } while (timeout > 0);

    g_txFail = 1;
}

static void receive_can_message(void)
{
    IfxCan_Message rxMsg;
    uint32 rxData[2];

    g_rxFail = 0;
    g_rxNew  = 0;

    IfxCan_Can_initMessage(&rxMsg);
    rxMsg.bufferNumber    = 0;
    rxMsg.frameMode       = IfxCan_FrameMode_standard;
    rxMsg.messageIdLength = IfxCan_MessageIdLength_standard;
    rxMsg.dataLengthCode  = IfxCan_DataLengthCode_8;
    rxMsg.readFromRxFifo0 = FALSE;

    IfxCan_Can_readMessage(&g_canNode0, &rxMsg, rxData);

    if (rxMsg.messageId == MY_RX_ID)
    {
        g_rxId    = rxMsg.messageId;
        g_rxData0 = rxData[0];
        g_rxData1 = rxData[1];
        g_rxCount++;
        g_rxNew = 1;

        IfxPort_togglePin(LED_RX_PORT, LED_RX_PIN);

        send_can_message(0xAABBCCDD, 0x12345678);

        if (g_txSuccess == 1)
        {
            IfxPort_togglePin(LED_TX_PORT, LED_TX_PIN);
        }
    }
    else
    {
        g_rxFail = 1;
    }
}
