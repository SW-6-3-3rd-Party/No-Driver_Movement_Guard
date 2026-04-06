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
    servoHold(SERVO_RELEASE_US, 500U);   /* 서보 초기 위치 고정 500ms */

    initCanAct();
    initMPU6050();

    /* ★ g_stmTicksPerUs는 init_pwm() 이전에 반드시 설정 */
    g_stmTicksPerUs = (uint32)(IfxStm_getFrequency(BSP_DEFAULT_TIMER) / 1000000U);

    init_pwm();

    /* 서보 20ms 주기 타이머 */
    uint32 servoLastTick = IfxStm_getLower(BSP_DEFAULT_TIMER);

    /* ── 메인 루프 ── */
    while (1)
    {
        updateEncoders();
        updateAccel();
        checkCanBusOff();          /* Bus-Off 감지 → 자동 복구 */
        receive_main_command();
        checkCanTimeout();

        /* 서보 펄스: 20ms마다 1회 출력 (블로킹 없이 주기 관리) */
        uint32 nowTick = IfxStm_getLower(BSP_DEFAULT_TIMER);
        uint32 servoElapsedUs = (nowTick - servoLastTick) / g_stmTicksPerUs;
        if (servoElapsedUs >= 20000U)
        {
            servoLastTick = nowTick;
            IfxPort_setPinHigh(SERVO_PORT, SERVO_PIN);
            waitTime(IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, g_servoPulseUs));
            IfxPort_setPinLow(SERVO_PORT, SERVO_PIN);
        }

        /* CAN 명령 미수신 → 대기 모드 (P단 + 모터 정지) */
        if (g_canCmdValid == 0U)
        {
            enterWaitCommandStep();
            send_act_status();
            continue;
        }

        processBrakeCommand();

        /* 비상 제동 최우선 */
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
