/*
 * RoboClaw.h
 *
 *  Created on: 1 mar 2026
 *      Author: aquiles
 *
 *  Ported from Arduino library to STM32 HAL
 */

#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

/******************************************************************************
* Definitions
******************************************************************************/
#define ROBOCLAW_VERSION 10

// Tamaño del buffer de recepción
#define ROBOCLAW_RX_BUFFER_SIZE 64

/******************************************************************************
* Error Codes
******************************************************************************/
typedef enum {
    RC_ERROR_NONE           = 0x000000,
    RC_ERROR_ESTOP          = 0x000001,  // E-Stop active
    RC_ERROR_TEMP           = 0x000002,  // Temperature Sensor 1 >=100C
    RC_ERROR_TEMP2          = 0x000004,  // Temperature Sensor 2 >=100C
    RC_ERROR_MBATHIGH       = 0x000008,  // Main Battery Over Voltage
    RC_ERROR_LBATHIGH       = 0x000010,  // Logic Battery High Voltage
    RC_ERROR_LBATLOW        = 0x000020,  // Logic Battery Low Voltage
    RC_ERROR_FAULTM1        = 0x000040,  // Motor 1 Driver Fault
    RC_ERROR_FAULTM2        = 0x000080,  // Motor 2 Driver Fault
    RC_ERROR_SPEED1         = 0x000100,  // Motor 1 Speed Error Limit
    RC_ERROR_SPEED2         = 0x000200,  // Motor 2 Speed Error Limit
    RC_ERROR_POS1           = 0x000400,  // Motor 1 Position Error Limit
    RC_ERROR_POS2           = 0x000800,  // Motor 2 Position Error Limit
    RC_WARN_OVERCURRENTM1   = 0x010000,  // Motor 1 Current Limited
    RC_WARN_OVERCURRENTM2   = 0x020000,  // Motor 2 Current Limited
    RC_WARN_MBATHIGH        = 0x040000,  // Main Battery Voltage High
    RC_WARN_MBATLOW         = 0x080000,  // Main Battery Low Voltage
    RC_WARN_TEMP            = 0x100000,  // Temperature Sensor 1 >=85C
    RC_WARN_TEMP2           = 0x200000,  // Temperature Sensor 2 >=85C
    RC_WARN_S4              = 0x400000,  // Motor 1 Home/Limit Signal
    RC_WARN_S5              = 0x800000   // Motor 2 Home/Limit Signal
} RoboClaw_Error_t;

/******************************************************************************
* Command Codes
******************************************************************************/
typedef enum {
    RC_M1FORWARD = 0,
    RC_M1BACKWARD = 1,
    RC_SETMINMB = 2,
    RC_SETMAXMB = 3,
    RC_M2FORWARD = 4,
    RC_M2BACKWARD = 5,
    RC_M17BIT = 6,
    RC_M27BIT = 7,
    RC_MIXEDFORWARD = 8,
    RC_MIXEDBACKWARD = 9,
    RC_MIXEDRIGHT = 10,
    RC_MIXEDLEFT = 11,
    RC_MIXEDFB = 12,
    RC_MIXEDLR = 13,
    RC_GETM1ENC = 16,
    RC_GETM2ENC = 17,
    RC_GETM1SPEED = 18,
    RC_GETM2SPEED = 19,
    RC_RESETENC = 20,
    RC_GETVERSION = 21,
    RC_SETM1ENCCOUNT = 22,
    RC_SETM2ENCCOUNT = 23,
    RC_GETMBATT = 24,
    RC_GETLBATT = 25,
    RC_SETMINLB = 26,
    RC_SETMAXLB = 27,
    RC_SETM1PID = 28,
    RC_SETM2PID = 29,
    RC_GETM1ISPEED = 30,
    RC_GETM2ISPEED = 31,
    RC_M1DUTY = 32,
    RC_M2DUTY = 33,
    RC_MIXEDDUTY = 34,
    RC_M1SPEED = 35,
    RC_M2SPEED = 36,
    RC_MIXEDSPEED = 37,
    RC_M1SPEEDACCEL = 38,
    RC_M2SPEEDACCEL = 39,
    RC_MIXEDSPEEDACCEL = 40,
    RC_M1SPEEDDIST = 41,
    RC_M2SPEEDDIST = 42,
    RC_MIXEDSPEEDDIST = 43,
    RC_M1SPEEDACCELDIST = 44,
    RC_M2SPEEDACCELDIST = 45,
    RC_MIXEDSPEEDACCELDIST = 46,
    RC_GETBUFFERS = 47,
    RC_GETPWMS = 48,
    RC_GETCURRENTS = 49,
    RC_MIXEDSPEED2ACCEL = 50,
    RC_MIXEDSPEED2ACCELDIST = 51,
    RC_M1DUTYACCEL = 52,
    RC_M2DUTYACCEL = 53,
    RC_MIXEDDUTYACCEL = 54,
    RC_READM1PID = 55,
    RC_READM2PID = 56,
    RC_SETMAINVOLTAGES = 57,
    RC_SETLOGICVOLTAGES = 58,
    RC_GETMINMAXMAINVOLTAGES = 59,
    RC_GETMINMAXLOGICVOLTAGES = 60,
    RC_SETM1POSPID = 61,
    RC_SETM2POSPID = 62,
    RC_READM1POSPID = 63,
    RC_READM2POSPID = 64,
    RC_M1SPEEDACCELDECCELPOS = 65,
    RC_M2SPEEDACCELDECCELPOS = 66,
    RC_MIXEDSPEEDACCELDECCELPOS = 67,
    RC_SETM1DEFAULTACCEL = 68,
    RC_SETM2DEFAULTACCEL = 69,
    RC_SETPINFUNCTIONS = 74,
    RC_GETPINFUNCTIONS = 75,
    RC_SETDEADBAND = 76,
    RC_GETDEADBAND = 77,
    RC_GETENCODERS = 78,
    RC_GETISPEEDS = 79,
    RC_RESTOREDEFAULTS = 80,
    RC_GETTEMP = 82,
    RC_GETTEMP2 = 83,
    RC_GETERROR = 90,
    RC_GETENCODERMODE = 91,
    RC_SETM1ENCODERMODE = 92,
    RC_SETM2ENCODERMODE = 93,
    RC_WRITENVM = 94,
    RC_READNVM = 95,
    RC_SETCONFIG = 98,
    RC_GETCONFIG = 99,
    RC_SETM1MAXCURRENT = 133,
    RC_SETM2MAXCURRENT = 134,
    RC_GETM1MAXCURRENT = 135,
    RC_GETM2MAXCURRENT = 136,
    RC_SETPWMMODE = 148,
    RC_GETPWMMODE = 149,
    RC_FLAGBOOTLOADER = 255
} RoboClaw_Command_t;

/******************************************************************************
* Estructura principal de RoboClaw
******************************************************************************/
typedef struct {
    UART_HandleTypeDef *huart;   // Handle del UART de STM32
    uint32_t timeout;            // Timeout en ms
    uint16_t crc;                // CRC actual
    uint8_t rxBuffer[ROBOCLAW_RX_BUFFER_SIZE];
} RoboClaw_HandleTypeDef;

/******************************************************************************
* Funciones de inicialización
******************************************************************************/
void RoboClaw_Init(RoboClaw_HandleTypeDef *rc, UART_HandleTypeDef *huart, uint32_t timeout);

/******************************************************************************
* Control básico de motores
******************************************************************************/
bool RoboClaw_ForwardM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_BackwardM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_ForwardM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_BackwardM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_ForwardBackwardM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_ForwardBackwardM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);

/******************************************************************************
* Control mixto (para robots diferenciales)
******************************************************************************/
bool RoboClaw_ForwardMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_BackwardMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_TurnRightMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_TurnLeftMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_ForwardBackwardMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);
bool RoboClaw_LeftRightMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed);

/******************************************************************************
* Encoders
******************************************************************************/
uint32_t RoboClaw_ReadEncM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid);
uint32_t RoboClaw_ReadEncM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid);
bool RoboClaw_SetEncM1(RoboClaw_HandleTypeDef *rc, uint8_t address, int32_t val);
bool RoboClaw_SetEncM2(RoboClaw_HandleTypeDef *rc, uint8_t address, int32_t val);
bool RoboClaw_ResetEncoders(RoboClaw_HandleTypeDef *rc, uint8_t address);
bool RoboClaw_ReadEncoders(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t *enc1, uint32_t *enc2);

/******************************************************************************
* Velocidad
******************************************************************************/
uint32_t RoboClaw_ReadSpeedM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid);
uint32_t RoboClaw_ReadSpeedM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid);
uint32_t RoboClaw_ReadISpeedM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid);
uint32_t RoboClaw_ReadISpeedM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid);
bool RoboClaw_ReadISpeeds(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t *ispeed1, uint32_t *ispeed2);

/******************************************************************************
* Control por Duty Cycle
******************************************************************************/
bool RoboClaw_DutyM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty);
bool RoboClaw_DutyM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty);
bool RoboClaw_DutyM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty1, uint16_t duty2);
bool RoboClaw_DutyAccelM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty, uint32_t accel);
bool RoboClaw_DutyAccelM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty, uint32_t accel);
bool RoboClaw_DutyAccelM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2);

/******************************************************************************
* Control por velocidad
******************************************************************************/
bool RoboClaw_SpeedM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed);
bool RoboClaw_SpeedM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed);
bool RoboClaw_SpeedM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed1, uint32_t speed2);
bool RoboClaw_SpeedAccelM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed);
bool RoboClaw_SpeedAccelM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed);
bool RoboClaw_SpeedAccelM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2);
bool RoboClaw_SpeedAccelM1M2_2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2);

/******************************************************************************
* Control por distancia
******************************************************************************/
bool RoboClaw_SpeedDistanceM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag);
bool RoboClaw_SpeedDistanceM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag);
bool RoboClaw_SpeedDistanceM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag);
bool RoboClaw_SpeedAccelDistanceM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag);
bool RoboClaw_SpeedAccelDistanceM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag);
bool RoboClaw_SpeedAccelDistanceM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag);
bool RoboClaw_SpeedAccelDistanceM1M2_2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag);

/******************************************************************************
* Control por posición
******************************************************************************/
bool RoboClaw_SpeedAccelDeccelPositionM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag);
bool RoboClaw_SpeedAccelDeccelPositionM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag);
bool RoboClaw_SpeedAccelDeccelPositionM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t deccel1, uint32_t position1, uint32_t accel2, uint32_t speed2, uint32_t deccel2, uint32_t position2, uint8_t flag);

/******************************************************************************
* PID de velocidad
******************************************************************************/
bool RoboClaw_SetM1VelocityPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
bool RoboClaw_SetM2VelocityPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
bool RoboClaw_ReadM1VelocityPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float *Kp, float *Ki, float *Kd, uint32_t *qpps);
bool RoboClaw_ReadM2VelocityPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float *Kp, float *Ki, float *Kd, uint32_t *qpps);

/******************************************************************************
* PID de posición
******************************************************************************/
bool RoboClaw_SetM1PositionPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float kp, float ki, float kd, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max);
bool RoboClaw_SetM2PositionPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float kp, float ki, float kd, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max);
bool RoboClaw_ReadM1PositionPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float *Kp, float *Ki, float *Kd, uint32_t *KiMax, uint32_t *DeadZone, uint32_t *Min, uint32_t *Max);
bool RoboClaw_ReadM2PositionPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float *Kp, float *Ki, float *Kd, uint32_t *KiMax, uint32_t *DeadZone, uint32_t *Min, uint32_t *Max);

/******************************************************************************
* Configuración de voltaje
******************************************************************************/
bool RoboClaw_SetMinVoltageMainBattery(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t voltage);
bool RoboClaw_SetMaxVoltageMainBattery(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t voltage);
bool RoboClaw_SetMinVoltageLogicBattery(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t voltage);
bool RoboClaw_SetMaxVoltageLogicBattery(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t voltage);
bool RoboClaw_SetMainVoltages(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t min, uint16_t max);
bool RoboClaw_SetLogicVoltages(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t min, uint16_t max);

/******************************************************************************
* Lectura de voltaje
******************************************************************************/
uint16_t RoboClaw_ReadMainBatteryVoltage(RoboClaw_HandleTypeDef *rc, uint8_t address, bool *valid);
uint16_t RoboClaw_ReadLogicBatteryVoltage(RoboClaw_HandleTypeDef *rc, uint8_t address, bool *valid);
bool RoboClaw_ReadMinMaxMainVoltages(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *min, uint16_t *max);
bool RoboClaw_ReadMinMaxLogicVoltages(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *min, uint16_t *max);

/******************************************************************************
* Lectura de corriente y PWM
******************************************************************************/
bool RoboClaw_ReadCurrents(RoboClaw_HandleTypeDef *rc, uint8_t address, int16_t *current1, int16_t *current2);
bool RoboClaw_ReadPWMs(RoboClaw_HandleTypeDef *rc, uint8_t address, int16_t *pwm1, int16_t *pwm2);

/******************************************************************************
* Temperatura
******************************************************************************/
bool RoboClaw_ReadTemp(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *temp);
bool RoboClaw_ReadTemp2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *temp);

/******************************************************************************
* Estado y errores
******************************************************************************/
uint32_t RoboClaw_ReadError(RoboClaw_HandleTypeDef *rc, uint8_t address, bool *valid);
bool RoboClaw_ReadBuffers(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *depth1, uint8_t *depth2);
bool RoboClaw_ReadVersion(RoboClaw_HandleTypeDef *rc, uint8_t address, char *version);

/******************************************************************************
* Configuración de encoders
******************************************************************************/
bool RoboClaw_ReadEncoderModes(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *M1mode, uint8_t *M2mode);
bool RoboClaw_SetM1EncoderMode(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t mode);
bool RoboClaw_SetM2EncoderMode(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t mode);

/******************************************************************************
* Configuración general
******************************************************************************/
bool RoboClaw_SetM1DefaultAccel(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel);
bool RoboClaw_SetM2DefaultAccel(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel);
bool RoboClaw_SetPinFunctions(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode);
bool RoboClaw_GetPinFunctions(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *S3mode, uint8_t *S4mode, uint8_t *S5mode);
bool RoboClaw_SetDeadBand(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t Min, uint8_t Max);
bool RoboClaw_GetDeadBand(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *Min, uint8_t *Max);

/******************************************************************************
* Límite de corriente
******************************************************************************/
bool RoboClaw_SetM1MaxCurrent(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t max);
bool RoboClaw_SetM2MaxCurrent(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t max);
bool RoboClaw_ReadM1MaxCurrent(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t *max);
bool RoboClaw_ReadM2MaxCurrent(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t *max);

/******************************************************************************
* PWM Mode
******************************************************************************/
bool RoboClaw_SetPWMMode(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t mode);
bool RoboClaw_GetPWMMode(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *mode);

/******************************************************************************
* NVM (Memoria no volátil)
******************************************************************************/
bool RoboClaw_WriteNVM(RoboClaw_HandleTypeDef *rc, uint8_t address);
bool RoboClaw_ReadNVM(RoboClaw_HandleTypeDef *rc, uint8_t address);
bool RoboClaw_RestoreDefaults(RoboClaw_HandleTypeDef *rc, uint8_t address);

/******************************************************************************
* Configuración
******************************************************************************/
bool RoboClaw_SetConfig(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t config);
bool RoboClaw_GetConfig(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *config);


#endif /* ROBOCLAW_H */
