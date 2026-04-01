/**********************************************************************************************************************
 * \file tof_sensor.c
 * \brief VL53L0X ToF 센서 래퍼
 *
 * VL53L0X 단일 측정 API는 블로킹이므로 별도 FreeRTOS 태스크에서 주기적으로 읽고,
 * Task_Sensor는 최신 측정값만 소비합니다.
 *********************************************************************************************************************/
#include "App/tof_sensor.h"

#include "FreeRTOS.h"
#include "task.h"

#include "vl53l0x_api.h"

void VL53L0X_I2C_Init(void);

static VL53L0X_Dev_t g_tofDev;
static VL53L0X_DEV   g_tof = &g_tofDev;

static volatile uint16 g_tofDistanceMm = TOF_NOT_READY_MM;
static volatile boolean g_tofReady = FALSE;
static volatile int g_tofInitStatus = -1;
static volatile int g_tofReadStatus = -1;

volatile uint16 g_tof_raw_mm = TOF_NOT_READY_MM;
volatile int    g_tof_init_debug = -1;
volatile int    g_tof_read_debug = -1;

static int TofSensor_Setup(void)
{
    VL53L0X_Error status;
    uint32 refSpadCount;
    uint8 isApertureSpads;
    uint8 vhvSettings;
    uint8 phaseCal;

    g_tof->I2cDevAddr = 0x29;
    g_tof->comms_type = 1;
    g_tof->comms_speed_khz = 100;

    status = VL53L0X_DataInit(g_tof);
    if (status != VL53L0X_ERROR_NONE) return -2;

    status = VL53L0X_StaticInit(g_tof);
    if (status != VL53L0X_ERROR_NONE) return -3;

    status = VL53L0X_PerformRefSpadManagement(g_tof, &refSpadCount, &isApertureSpads);
    if (status != VL53L0X_ERROR_NONE) return -4;

    status = VL53L0X_PerformRefCalibration(g_tof, &vhvSettings, &phaseCal);
    if (status != VL53L0X_ERROR_NONE) return -5;

    status = VL53L0X_SetDeviceMode(g_tof, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE) return -6;

    return 0;
}

static int TofSensor_ReadBlocking(void)
{
    VL53L0X_Error status;
    VL53L0X_RangingMeasurementData_t data;

    status = VL53L0X_PerformSingleRangingMeasurement(g_tof, &data);
    if (status != VL53L0X_ERROR_NONE)
        return -1;

    if (data.RangeStatus == 0)
        return (int)data.RangeMilliMeter;

    return -2;
}

int TofSensor_Init(void)
{
    VL53L0X_I2C_Init();
    g_tofInitStatus = TofSensor_Setup();
    g_tofReady = (g_tofInitStatus == 0) ? TRUE : FALSE;
    g_tofDistanceMm = g_tofReady ? TOF_NOT_READY_MM : TOF_ERROR_MM;

    g_tof_init_debug = g_tofInitStatus;

    return g_tofInitStatus;
}

uint16 TofSensor_GetDistanceMm(void)
{
    return g_tofDistanceMm;
}

boolean TofSensor_IsReady(void)
{
    return g_tofReady;
}

void Task_ToF(void *param)
{
    (void)param;

    TickType_t xLastWake = xTaskGetTickCount();

    while (1)
    {
        if (g_tofReady == TRUE)
        {
            int distanceMm = TofSensor_ReadBlocking();
            g_tofReadStatus = distanceMm;

            if (distanceMm > 0)
                g_tofDistanceMm = (uint16)distanceMm;
            else
                g_tofDistanceMm = TOF_ERROR_MM;
        }
        else
        {
            g_tofDistanceMm = TOF_ERROR_MM;
        }

        g_tof_raw_mm = g_tofDistanceMm;
        g_tof_read_debug = g_tofReadStatus;

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(50));
    }
}
