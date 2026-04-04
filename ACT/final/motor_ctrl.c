#include "motor_ctrl.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "Bsp.h"

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
        IfxPort_setPinHigh(M1_DIR_PORT, M1_DIR_PIN);
        IfxPort_setPinHigh(M2_DIR_PORT, M2_DIR_PIN);
    }
    else
    {
        IfxPort_setPinLow(M1_DIR_PORT, M1_DIR_PIN);
        IfxPort_setPinLow(M2_DIR_PORT, M2_DIR_PIN);
    }
}

/* ── 공개 함수 ── */
void initMotorPins(void)
{
    IfxPort_setPinModeOutput(M1_PWM_PORT,   M1_PWM_PIN,   IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(M1_BRAKE_PORT, M1_BRAKE_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(M1_DIR_PORT,   M1_DIR_PIN,   IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(M2_PWM_PORT,   M2_PWM_PIN,   IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(M2_BRAKE_PORT, M2_BRAKE_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(M2_DIR_PORT,   M2_DIR_PIN,   IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    IfxPort_setPinLow(M1_PWM_PORT,   M1_PWM_PIN);
    IfxPort_setPinHigh(M1_BRAKE_PORT, M1_BRAKE_PIN);
    IfxPort_setPinLow(M1_DIR_PORT,   M1_DIR_PIN);
    IfxPort_setPinLow(M2_PWM_PORT,   M2_PWM_PIN);
    IfxPort_setPinHigh(M2_BRAKE_PORT, M2_BRAKE_PIN);
    IfxPort_setPinLow(M2_DIR_PORT,   M2_DIR_PIN);
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

void motorsBrakeStop(void)
{
    IfxPort_setPinLow(M1_PWM_PORT,   M1_PWM_PIN);
    IfxPort_setPinHigh(M1_BRAKE_PORT, M1_BRAKE_PIN);
    IfxPort_setPinLow(M2_PWM_PORT,   M2_PWM_PIN);
    IfxPort_setPinHigh(M2_BRAKE_PORT, M2_BRAKE_PIN);
}

void motorsReleaseBrake(void)
{
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

    motorsSetDirection(forward);
    motorsReleaseBrake();

    while (elapsedMs < runTimeMs)
    {
        for (i = 0U; i < PWM_PERIOD_US; i++)
        {
            if (i < onTimeM1Us) IfxPort_setPinHigh(M1_PWM_PORT, M1_PWM_PIN);
            else                IfxPort_setPinLow(M1_PWM_PORT,  M1_PWM_PIN);
            if (i < onTimeM2Us) IfxPort_setPinHigh(M2_PWM_PORT, M2_PWM_PIN);
            else                IfxPort_setPinLow(M2_PWM_PORT,  M2_PWM_PIN);
            updateEncoders();
            delayUs(1U);
        }
        elapsedMs++;
    }

    IfxPort_setPinLow(M1_PWM_PORT, M1_PWM_PIN);
    IfxPort_setPinLow(M2_PWM_PORT, M2_PWM_PIN);
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
