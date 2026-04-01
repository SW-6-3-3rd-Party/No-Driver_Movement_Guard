#ifndef APP_TOF_SENSOR_H_
#define APP_TOF_SENSOR_H_

#include "Ifx_Types.h"

#define TOF_NOT_READY_MM   ((uint16)999U)
#define TOF_ERROR_MM       ((uint16)0U)

int TofSensor_Init(void);
uint16 TofSensor_GetDistanceMm(void);
boolean TofSensor_IsReady(void);
void Task_ToF(void *param);

#endif /* APP_TOF_SENSOR_H_ */
