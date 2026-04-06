#include <Gtm/Tom/Pwm/IfxGtm_Tom_Pwm.h>

#include "motor_ctrl.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "Bsp.h"



Ifx_GTM *gtm = &MODULE_GTM;
#define TOM0_CH0_PRIO 10
IfxGtm_Tom_Pwm_Config tomConfig; //configuration structure
IfxGtm_Tom_Pwm_Driver tomHandle; // handle

/* ── 전역 정의 ── */
volatile uint32 g_m1Pulse = 0U, g_m2Pulse = 0U, g_avgPulse = 0U;
volatile sint32 g_m1Position = 0, g_m2Position = 0;
volatile sint8  g_m1Direction = 0, g_m2Direction = 0;
volatile uint16 g_servoPulseUs = SERVO_RELEASE_US;

/* ── 내부 전용 ── */
static volatile uint32 g_m1Change = 0U, g_m2Change = 0U;
static volatile uint8  g_m1AState = 0U, g_m1PrevAState = 0U, g_m1BState = 0U;
static volatile uint8  g_m2AState = 0U, g_m2PrevAState = 0U, g_m2BState = 0U;

static uint8 readEnc1A(void) { return IfxPort_getPinState(ENC1_A_PORT, ENC1_A_PIN) ? 1U : 0U; }
static uint8 readEnc1B(void) { return IfxPort_getPinState(ENC1_B_PORT, ENC1_B_PIN) ? 1U : 0U; }
static uint8 readEnc2A(void) { return IfxPort_getPinState(ENC2_A_PORT, ENC2_A_PIN) ? 1U : 0U; }
static uint8 readEnc2B(void) { return IfxPort_getPinState(ENC2_B_PORT, ENC2_B_PIN) ? 1U : 0U; }

static void motorsSetDirection(uint8 forward)
{
    if (forward != 0U)
    {
        /* 전진: 왼쪽(M1) 역방향 / 오른쪽(M2) 정방향 */
        IfxPort_setPinLow(M1_DIR_PORT,  M1_DIR_PIN);
        IfxPort_setPinHigh(M2_DIR_PORT, M2_DIR_PIN);
    }
    else
    {
        /* 후진: 왼쪽(M1) 정방향 / 오른쪽(M2) 역방향 */
        IfxPort_setPinHigh(M1_DIR_PORT, M1_DIR_PIN);
        IfxPort_setPinLow(M2_DIR_PORT,  M2_DIR_PIN);
    }
}

/* ── 공개 함수 ── */
void initMotorPins(void)
{
    IfxPort_setPinModeOutput(M1_PWM_PORT,   M1_PWM_PIN,   IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(M1_DIR_PORT,   M1_DIR_PIN,   IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(M2_PWM_PORT,   M2_PWM_PIN,   IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(M2_DIR_PORT,   M2_DIR_PIN,   IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    /* BRAKE 핀은 모터쉴드 점퍼를 GND 단락했으므로
       소프트웨어에서 LOW 고정 (브레이크 항상 해제) */
    IfxPort_setPinModeOutput(M1_BRAKE_PORT, M1_BRAKE_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(M2_BRAKE_PORT, M2_BRAKE_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    IfxPort_setPinLow(M1_PWM_PORT,   M1_PWM_PIN);
    IfxPort_setPinLow(M1_DIR_PORT,   M1_DIR_PIN);
    IfxPort_setPinLow(M2_PWM_PORT,   M2_PWM_PIN);
    IfxPort_setPinLow(M2_DIR_PORT,   M2_DIR_PIN);

    /* BRAKE 초기값 LOW → 브레이크 해제 상태로 시작 */
    IfxPort_setPinLow(M1_BRAKE_PORT, M1_BRAKE_PIN);
    IfxPort_setPinLow(M2_BRAKE_PORT, M2_BRAKE_PIN);
}

void initEncoderPins(void)
{
    IfxPort_setPinModeInput(ENC1_A_PORT, ENC1_A_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(ENC1_B_PORT, ENC1_B_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(ENC2_A_PORT, ENC2_A_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(ENC2_B_PORT, ENC2_B_PIN, IfxPort_InputMode_pullUp);
}

void initServoPin(void)
{
    IfxPort_setPinModeOutput(SERVO_PORT, SERVO_PIN,
                             IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinLow(SERVO_PORT, SERVO_PIN);
}

void motorsFullStop(void)
{
    /* P단 전용: GTM PWM 완전 정지(출력 LOW) + PWM핀 LOW + BRAKE핀 LOW */
    IfxGtm_Tom_Pwm_stop(&tomHandle, FALSE);
    IfxPort_setPinLow(M1_PWM_PORT,   M1_PWM_PIN);
    IfxPort_setPinLow(M2_PWM_PORT,   M2_PWM_PIN);
    IfxPort_setPinHigh(M1_BRAKE_PORT, M1_BRAKE_PIN);
    IfxPort_setPinHigh(M2_BRAKE_PORT, M2_BRAKE_PIN);
}

void motorsBrakeStop(void)
{
    ////////////////////////
    IfxGtm_Tom_Pwm_stop(&tomHandle, FALSE);
    ////////////////////////
    IfxPort_setPinLow(M1_PWM_PORT,   M1_PWM_PIN);
    IfxPort_setPinHigh(M1_BRAKE_PORT, M1_BRAKE_PIN);
    IfxPort_setPinLow(M2_PWM_PORT,   M2_PWM_PIN);
    IfxPort_setPinHigh(M2_BRAKE_PORT, M2_BRAKE_PIN);

}

void motorsReleaseBrake(void)
{
    ///////////////////////////////////////////////
    IfxGtm_Tom_Pwm_stop(&tomHandle, TRUE);
    ///////////////////////////////////////////////
    IfxPort_setPinLow(M1_BRAKE_PORT, M1_BRAKE_PIN);
    IfxPort_setPinLow(M2_BRAKE_PORT, M2_BRAKE_PIN);
}

void motorsRunDuty(uint8 forward, uint8 dutyM1, uint8 dutyM2, uint32 runTimeMs)
{
    uint32 elapsedMs = 0U, i;
    if (dutyM1 > 100U) dutyM1 = 100U;
    if (dutyM2 > 100U) dutyM2 = 100U;

    uint32 onTimeM1Us = (PWM_PERIOD_US * (uint32)dutyM1) / 100U;
    uint32 onTimeM2Us = (PWM_PERIOD_US * (uint32)dutyM2) / 100U;

    while (elapsedMs < runTimeMs)
    {
        for (i = 0U; i < PWM_PERIOD_US; i++)
        {
            updateEncoders();
            delayUs(1U);
        }
        elapsedMs++;
    }

    motorsSetDirection(forward);
    motorsReleaseBrake();
    ///////////////////////////////////
    IfxGtm_Tom_Pwm_start(&tomHandle, TRUE);
    ///////////////////////////////////
}

void updateEncoders(void)
{
    g_m1AState = readEnc1A();
    g_m1BState = readEnc1B();

    if (g_m1AState != g_m1PrevAState)
    {
        g_m1Change++;
        if ((g_m1PrevAState == 0U) && (g_m1AState == 1U))
        {
            g_m1Pulse++;
            if (g_m1BState == 0U) { g_m1Direction =  1; g_m1Position++; }
            else                  { g_m1Direction = -1; g_m1Position--; }
        }
        g_m1PrevAState = g_m1AState;
    }

    g_m2AState = readEnc2A();
    g_m2BState = readEnc2B();

    if (g_m2AState != g_m2PrevAState)
    {
        g_m2Change++;
        if ((g_m2PrevAState == 0U) && (g_m2AState == 1U))
        {
            g_m2Pulse++;
            if (g_m2BState == 0U) { g_m2Direction =  1; g_m2Position++; }
            else                  { g_m2Direction = -1; g_m2Position--; }
        }
        g_m2PrevAState = g_m2AState;
    }

    g_avgPulse = (g_m1Pulse + g_m2Pulse) / 2U;
}

void servoWritePulseUs(uint16 pulseUs)
{
    g_servoPulseUs = pulseUs;
    IfxPort_setPinHigh(SERVO_PORT, SERVO_PIN);
    waitTime(IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, pulseUs));
    IfxPort_setPinLow(SERVO_PORT, SERVO_PIN);
    waitTime(IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, (20000U - pulseUs)));
}

void servoHold(uint16 pulseUs, uint32 durationMs)
{
    uint32 elapsed = 0U;
    while (elapsed < durationMs)
    {
        servoWritePulseUs(pulseUs);
        elapsed += 20U;
    }
}

void delayUs(uint32 us)
{
    waitTime(IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, us));
}

void init_pwm(void)
{
    //////////////////////////////////////////////////
    // Enables the GTM
      IfxGtm_enable(gtm);

      float32 frequency = IfxGtm_Cmu_getModuleFrequency(gtm);

      // Set the global clock frequency to the max
      IfxGtm_Cmu_setGclkFrequency(gtm, frequency);
      // Set the CMU CLK0
      IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_0, frequency);
      // FXCLK: used by TOM and CLK0: used by ATOM
      IfxGtm_Cmu_enableClocks(gtm, IFXGTM_CMU_CLKEN_FXCLK | IFXGTM_CMU_CLKEN_CLK0);

      // initialise TOM


      IfxGtm_Tom_Pwm_initConfig(&tomConfig, gtm);

      tomConfig.tomChannel  = IfxGtm_Tom_Ch_3;
      tomConfig.period                   = 10000;
      tomConfig.dutyCycle                = 9500;
      tomConfig.pin.outputPin = &IfxGtm_TOM0_3_TOUT105_P10_3_OUT;
      tomConfig.synchronousUpdateEnabled = TRUE;

      IfxGtm_Tom_Pwm_init(&tomHandle, &tomConfig);
    //////////////////////////////////////////////////
}
