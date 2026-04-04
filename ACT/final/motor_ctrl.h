#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "Ifx_Types.h"

/* ── 핀 정의 ── */
#define M1_PWM_PORT   &MODULE_P21
#define M1_PWM_PIN    7
#define M1_BRAKE_PORT &MODULE_P02
#define M1_BRAKE_PIN  7
#define M1_DIR_PORT   &MODULE_P10
#define M1_DIR_PIN    1

#define M2_PWM_PORT   &MODULE_P10
#define M2_PWM_PIN    3
#define M2_BRAKE_PORT &MODULE_P02
#define M2_BRAKE_PIN  6
#define M2_DIR_PORT   &MODULE_P10
#define M2_DIR_PIN    2

#define ENC1_A_PORT   &MODULE_P10
#define ENC1_A_PIN    4
#define ENC1_B_PORT   &MODULE_P02
#define ENC1_B_PIN    3

#define ENC2_A_PORT   &MODULE_P02
#define ENC2_A_PIN    5
#define ENC2_B_PORT   &MODULE_P02
#define ENC2_B_PIN    4

#define SERVO_PORT    &MODULE_P10
#define SERVO_PIN     5

/* ── PWM / 서보 파라미터 ── */
#define SERVO_RELEASE_US  1000U
#define SERVO_BRAKE_US    1500U
#define PWM_PERIOD_US     1000U
#define DRIVE_CHUNK_MS    20U
#define DRIVE_M1_DUTY     12U
#define DRIVE_M2_DUTY     15U

/* ── 엔코더 전역 ── */
extern volatile uint32 g_m1Pulse, g_m2Pulse, g_avgPulse;
extern volatile sint32 g_m1Position, g_m2Position;
extern volatile sint8  g_m1Direction, g_m2Direction;

/* ── 서보 상태 ── */
extern volatile uint16 g_servoPulseUs;

/* ── 공개 함수 ── */
void initMotorPins(void);
void initEncoderPins(void);
void initServoPin(void);

void motorsBrakeStop(void);
void motorsReleaseBrake(void);
void motorsRunDuty(uint8 forward, uint8 dutyM1, uint8 dutyM2, uint32 runTimeMs);

void updateEncoders(void);

void servoWritePulseUs(uint16 pulseUs);
void servoHold(uint16 pulseUs, uint32 durationMs);
void delayUs(uint32 us);

#endif /* MOTOR_CTRL_H */
