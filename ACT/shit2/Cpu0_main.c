#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxStm.h"
#include "Bsp.h"

#include "can_act.h"
#include "motor_ctrl.h"
#include "drive_mode.h"
#include "mpu6050.h"

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

void core0_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* ── 하드웨어 초기화 ── */
    initMotorPins();
    initEncoderPins();
    initServoPin();
    servoHold(SERVO_RELEASE_US, 500U);   /* 초기 위치 고정 500ms (블로킹 허용) */

    initCanAct();
    initMPU6050();

    /* ★ g_stmTicksPerUs는 init_pwm() 이전에 반드시 설정 */
    g_stmTicksPerUs = (uint32)(IfxStm_getFrequency(BSP_DEFAULT_TIMER) / 1000000U);
    init_pwm();

    /* ── 서보 20ms 타이머 ── */
    uint32 servoLastTick = IfxStm_getLower(BSP_DEFAULT_TIMER);

    /* ── 메인 루프 ── */
    while (1)
    {
        updateEncoders();
        updateAccel();
        checkCanBusOff();
        receive_main_command();
        checkCanTimeout();

        /* 서보 펄스: 20ms마다 1회 출력 (HIGH 구간만 waitTime) */
        {
            uint32 nowTick   = IfxStm_getLower(BSP_DEFAULT_TIMER);
            uint32 elapsedUs = (nowTick - servoLastTick) / g_stmTicksPerUs;
            if (elapsedUs >= 20000U)
            {
                servoLastTick = nowTick;
                IfxPort_setPinHigh(SERVO_PORT, SERVO_PIN);
                waitTime(IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, g_servoPulseUs));
                IfxPort_setPinLow(SERVO_PORT, SERVO_PIN);
            }
        }

        if (g_canCmdValid == 0U)
        {
            enterWaitCommandStep();
            send_act_status();
            continue;
        }

        processBrakeCommand();

        if (g_canBrakeCmd == BRAKE_CMD_EMERGENCY)
        {
            applyEmergencyBrakeStep();
        }
        else
        {
            switch (g_gearMode)
            {
            case GEAR_P: enterPModeStep(); break;
            case GEAR_R: runRModeStep();   break;
            case GEAR_D: runDModeStep();   break;
            case GEAR_N: runNModeStep();   break;
            default:     enterPModeStep(); break;
            }
        }

        send_act_status();
    }
}
