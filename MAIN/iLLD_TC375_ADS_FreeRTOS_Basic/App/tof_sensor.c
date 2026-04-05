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
#include "vl53l0x_device.h"

void VL53L0X_I2C_Init(void);
void VL53L0X_I2C_Recover(void);
int VL53L0X_I2C_GetLastStatus(void);
int VL53L0X_I2C_GetLastBusStatus(void);

#define TOF_READ_PERIOD_MS              50u
#define TOF_RECOVERY_RETRY_PERIOD_MS   250u
#define TOF_MAX_I2C_ERROR_STREAK         3u
#define TOF_SOFT_RESET_ASSERT_POLLS      5u
#define TOF_SOFT_RESET_RELEASE_POLLS    10u
#define TOF_SOFT_RESET_POLL_DELAY_MS     2u

static VL53L0X_Dev_t g_tofDev;
static VL53L0X_DEV   g_tof = &g_tofDev;

static volatile uint16 g_tofDistanceMm = TOF_NOT_READY_MM;
static volatile boolean g_tofReady = FALSE;
static volatile int g_tofInitStatus = -1;
static volatile int g_tofReadStatus = -1;
static uint8 g_tofI2cErrorStreak = 0u;

volatile uint16 g_tof_raw_mm = TOF_NOT_READY_MM;
volatile int    g_tof_init_debug = -1;
volatile int    g_tof_read_debug = -1;
volatile int    g_tof_recover_debug = -1;
volatile int    g_tof_soft_reset_debug = -1;
volatile int    g_tof_i2c_status_debug = 0;
volatile int    g_tof_i2c_bus_status_debug = 0;
volatile uint8  g_tof_error_streak_debug = 0u;
volatile uint32 g_tof_recover_count = 0u;

static void TofSensor_UpdateI2cDebug(void)
{
    g_tof_i2c_status_debug = VL53L0X_I2C_GetLastStatus();
    g_tof_i2c_bus_status_debug = VL53L0X_I2C_GetLastBusStatus();
}

static void TofSensor_ConfigDevice(void)
{
    g_tof->I2cDevAddr = 0x29;
    g_tof->comms_type = 1;
    g_tof->comms_speed_khz = 100;
}

static int TofSensor_Setup(void)
{
    VL53L0X_Error status;
    uint32 refSpadCount;
    uint8 isApertureSpads;
    uint8 vhvSettings;
    uint8 phaseCal;

    TofSensor_ConfigDevice();

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

static int TofSensor_SoftReset(void)
{
    VL53L0X_Error status;
    uint8 modelId = 0xFFu;
    uint32 pollCount;

    status = VL53L0X_WrByte(g_tof, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00);
    if (status != VL53L0X_ERROR_NONE)
        return -1;

    for (pollCount = 0u; pollCount < TOF_SOFT_RESET_ASSERT_POLLS; ++pollCount)
    {
        status = VL53L0X_RdByte(g_tof, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &modelId);
        if ((status == VL53L0X_ERROR_NONE) && (modelId == 0x00u))
            break;

        vTaskDelay(pdMS_TO_TICKS(TOF_SOFT_RESET_POLL_DELAY_MS));
    }

    if ((status != VL53L0X_ERROR_NONE) || (modelId != 0x00u))
        return -2;

    status = VL53L0X_WrByte(g_tof, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01);
    if (status != VL53L0X_ERROR_NONE)
        return -3;

    for (pollCount = 0u; pollCount < TOF_SOFT_RESET_RELEASE_POLLS; ++pollCount)
    {
        status = VL53L0X_RdByte(g_tof, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &modelId);
        if ((status == VL53L0X_ERROR_NONE) && (modelId != 0x00u))
            return 0;

        vTaskDelay(pdMS_TO_TICKS(TOF_SOFT_RESET_POLL_DELAY_MS));
    }

    return -4;
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

static int TofSensor_Recover(void)
{
    int softResetStatus;
    int setupStatus;

    VL53L0X_I2C_Recover();
    TofSensor_ConfigDevice();

    softResetStatus = TofSensor_SoftReset();
    g_tof_soft_reset_debug = softResetStatus;

    if (softResetStatus != 0)
    {
        VL53L0X_I2C_Recover();
        TofSensor_ConfigDevice();
    }

    setupStatus = TofSensor_Setup();
    g_tof_recover_debug = setupStatus;
    TofSensor_UpdateI2cDebug();

    return setupStatus;
}

int TofSensor_Init(void)
{
    VL53L0X_I2C_Init();
    g_tofInitStatus = TofSensor_Setup();
    g_tofReady = (g_tofInitStatus == 0) ? TRUE : FALSE;
    g_tofDistanceMm = g_tofReady ? TOF_NOT_READY_MM : TOF_ERROR_MM;

    g_tof_init_debug = g_tofInitStatus;
    g_tof_recover_debug = g_tofInitStatus;
    g_tof_soft_reset_debug = 0;
    g_tofI2cErrorStreak = 0u;
    TofSensor_UpdateI2cDebug();

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
    TickType_t xLastRecoveryTick = xLastWake;

    while (1)
    {
        if (g_tofReady == TRUE)
        {
            int distanceMm = TofSensor_ReadBlocking();
            g_tofReadStatus = distanceMm;
            TofSensor_UpdateI2cDebug();

            if (distanceMm > 0)
            {
                g_tofDistanceMm = (uint16)distanceMm;
                g_tofI2cErrorStreak = 0u;
            }
            else
            {
                g_tofDistanceMm = TOF_ERROR_MM;

                if (distanceMm == -1)
                {
                    if (g_tofI2cErrorStreak < 255u)
                        g_tofI2cErrorStreak++;

                    if (g_tofI2cErrorStreak >= TOF_MAX_I2C_ERROR_STREAK)
                    {
                        xLastRecoveryTick = xTaskGetTickCount();
                        g_tof_recover_count++;
                        g_tofInitStatus = TofSensor_Recover();
                        g_tof_init_debug = g_tofInitStatus;
                        g_tofReady = (g_tofInitStatus == 0) ? TRUE : FALSE;
                        g_tofI2cErrorStreak = (g_tofReady == TRUE) ? 0u : TOF_MAX_I2C_ERROR_STREAK;
                    }
                }
                else
                {
                    g_tofI2cErrorStreak = 0u;
                }
            }
        }
        else
        {
            g_tofDistanceMm = TOF_ERROR_MM;
            TofSensor_UpdateI2cDebug();

            if ((xTaskGetTickCount() - xLastRecoveryTick) >= pdMS_TO_TICKS(TOF_RECOVERY_RETRY_PERIOD_MS))
            {
                xLastRecoveryTick = xTaskGetTickCount();
                g_tof_recover_count++;
                g_tofInitStatus = TofSensor_Recover();
                g_tof_init_debug = g_tofInitStatus;
                g_tofReady = (g_tofInitStatus == 0) ? TRUE : FALSE;
                g_tofI2cErrorStreak = (g_tofReady == TRUE) ? 0u : TOF_MAX_I2C_ERROR_STREAK;
            }
        }

        g_tof_raw_mm = g_tofDistanceMm;
        g_tof_read_debug = g_tofReadStatus;
        g_tof_error_streak_debug = g_tofI2cErrorStreak;

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(TOF_READ_PERIOD_MS));
    }
}
