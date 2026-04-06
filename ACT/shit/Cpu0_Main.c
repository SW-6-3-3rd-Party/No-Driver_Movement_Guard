//#include "Ifx_Types.h"
//#include "IfxCpu.h"
//#include "IfxScuWdt.h"
//#include "IfxStm.h"
//#include "Bsp.h"
//
//#include "can_act.h"      /* CAN 송수신 */
//#include "motor_ctrl.h"   /* 모터 / 엔코더 / 서보 */
//#include "drive_mode.h"   /* 기어 상태머신 */
//#include "mpu6050.h"      /* mpu6050 가속도 센서*/
//
//IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;
//
//void core0_main(void)
//{
//    IfxCpu_enableInterrupts();
//    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
//    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
//
//    initMotorPins();
//    initEncoderPins();
//    initServoPin();
//    servoHold(SERVO_RELEASE_US, 500U);
//
//    initCanAct();
//    initMPU6050();
//
//    g_stmTicksPerUs = (uint32)(IfxStm_getFrequency(BSP_DEFAULT_TIMER) / 1000000U);
//
//    while (1)
//    {
//        updateEncoders();
//        updateAccel();
//        receive_main_command();
//        checkCanTimeout();
//
//        if (g_canCmdValid == 0U)
//        {
//            enterWaitCommandStep();
//            send_act_status();
//            continue;
//        }
//
//        processBrakeCommand();
//
//        if (g_canBrakeCmd == BRAKE_CMD_EMERGENCY)
//        {
//            applyEmergencyBrakeStep();
//        }
//        else
//        {
//            switch (g_gearMode)
//            {
//            case GEAR_P: enterPModeStep(); break;
//            case GEAR_R: runRModeStep();   break;
//            case GEAR_D: runDModeStep();   break;
//            case GEAR_N: runNModeStep();   break;
//            default:     enterPModeStep(); break;
//            }
//        }
//
//        send_act_status();
//    }
//
//}
//


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


    initMotorPins();
    initEncoderPins();
    initServoPin();
    servoHold(SERVO_RELEASE_US, 500U);

    /* CAN은 상태 송신만 볼 거면 유지, 아예 단독 테스트면 initCanAct도 잠시 빼도 됨 */
    initCanAct();
    initMPU6050();

    g_stmTicksPerUs = (uint32)(IfxStm_getFrequency(BSP_DEFAULT_TIMER) / 1000000U);

    init_pwm();


    while (1)
    {

        updateEncoders();
        updateAccel();
        receive_main_command();
        checkCanTimeout();

/*----------------------------------------*/
//        g_canCmdValid = 1U;
//        g_canGearState = CAN_GEAR_D;
//        g_canBrakeCmd = BRAKE_CMD_RELEASE;
//          servoHold(1500U, 2000U);
//          servoHold(2000U, 2000U);

/*----------------------------------------*/
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
