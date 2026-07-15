/*
 * RoboClaw.c
 *
 *  Created on: 1 mar 2026
 *      Author: aquil
 *
 *  Ported from Arduino library to STM32 HAL
 */

#include "RoboClaw.h"
#include <string.h>

/******************************************************************************
* Definiciones privadas
******************************************************************************/
#define MAXRETRY 2

// Macros para convertir valores a bytes (Big Endian)
#define SetDWORDval(arg) (uint8_t)(((uint32_t)(arg))>>24),(uint8_t)(((uint32_t)(arg))>>16),(uint8_t)(((uint32_t)(arg))>>8),(uint8_t)(arg)
#define SetWORDval(arg)  (uint8_t)(((uint16_t)(arg))>>8),(uint8_t)(arg)

/******************************************************************************
* Funciones privadas - Prototipos
******************************************************************************/
static void crc_clear(RoboClaw_HandleTypeDef *rc);
static void crc_update(RoboClaw_HandleTypeDef *rc, uint8_t data);
static uint16_t crc_get(RoboClaw_HandleTypeDef *rc);
static bool write_byte(RoboClaw_HandleTypeDef *rc, uint8_t byte);
static int16_t read_byte(RoboClaw_HandleTypeDef *rc);
static void flush(RoboClaw_HandleTypeDef *rc);
static bool write_n(RoboClaw_HandleTypeDef *rc, uint8_t cnt, ...);
static bool read_n(RoboClaw_HandleTypeDef *rc, uint8_t cnt, uint8_t address, uint8_t cmd, ...);
static uint8_t Read1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t cmd, bool *valid);
static uint16_t Read2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t cmd, bool *valid);
static uint32_t Read4(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t cmd, bool *valid);
static uint32_t Read4_1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t cmd, uint8_t *status, bool *valid);

/******************************************************************************
* Funciones de CRC
******************************************************************************/
static void crc_clear(RoboClaw_HandleTypeDef *rc)
{
    rc->crc = 0;
}

static void crc_update(RoboClaw_HandleTypeDef *rc, uint8_t data)
{
    rc->crc = rc->crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; i++)
    {
        if (rc->crc & 0x8000)
            rc->crc = (rc->crc << 1) ^ 0x1021;
        else
            rc->crc <<= 1;
    }
}

static uint16_t crc_get(RoboClaw_HandleTypeDef *rc)
{
    return rc->crc;
}

/******************************************************************************
* Funciones de comunicación UART
******************************************************************************/
static bool write_byte(RoboClaw_HandleTypeDef *rc, uint8_t byte)
{
    return HAL_UART_Transmit(rc->huart, &byte, 1, rc->timeout) == HAL_OK;
}

static int16_t read_byte(RoboClaw_HandleTypeDef *rc)
{
    uint8_t data;
    if (HAL_UART_Receive(rc->huart, &data, 1, rc->timeout) == HAL_OK)
    {
        return data;
    }
    return -1;  // Timeout o error
}

static void flush(RoboClaw_HandleTypeDef *rc)
{
    uint8_t dummy;
    // Leer y descartar cualquier dato pendiente en el buffer
    while (HAL_UART_Receive(rc->huart, &dummy, 1, 1) == HAL_OK);
}

/******************************************************************************
* Funciones de escritura
******************************************************************************/
static bool write_n(RoboClaw_HandleTypeDef *rc, uint8_t cnt, ...)
{
    uint8_t trys = MAXRETRY;

    do {
        crc_clear(rc);

        va_list marker;
        va_start(marker, cnt);

        for (uint8_t index = 0; index < cnt; index++)
        {
            uint8_t data = (uint8_t)va_arg(marker, int);
            crc_update(rc, data);
            write_byte(rc, data);
        }
        va_end(marker);

        // Enviar CRC
        uint16_t crc = crc_get(rc);
        write_byte(rc, crc >> 8);
        write_byte(rc, crc & 0xFF);

        // Esperar confirmación (0xFF)
        if (read_byte(rc) == 0xFF)
            return true;

    } while (trys--);

    return false;
}

/******************************************************************************
* Funciones de lectura
******************************************************************************/
static bool read_n(RoboClaw_HandleTypeDef *rc, uint8_t cnt, uint8_t address, uint8_t cmd, ...)
{
    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;

    do {
        flush(rc);

        data = 0;
        crc_clear(rc);
        write_byte(rc, address);
        crc_update(rc, address);
        write_byte(rc, cmd);
        crc_update(rc, cmd);

        va_list marker;
        va_start(marker, cmd);

        for (uint8_t index = 0; index < cnt; index++)
        {
            uint32_t *ptr = va_arg(marker, uint32_t *);

            if (data != -1)
            {
                data = read_byte(rc);
                crc_update(rc, data);
                value = (uint32_t)data << 24;
            }
            else break;

            if (data != -1)
            {
                data = read_byte(rc);
                crc_update(rc, data);
                value |= (uint32_t)data << 16;
            }
            else break;

            if (data != -1)
            {
                data = read_byte(rc);
                crc_update(rc, data);
                value |= (uint32_t)data << 8;
            }
            else break;

            if (data != -1)
            {
                data = read_byte(rc);
                crc_update(rc, data);
                value |= (uint32_t)data;
            }
            else break;

            *ptr = value;
        }
        va_end(marker);

        if (data != -1)
        {
            uint16_t ccrc;
            data = read_byte(rc);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read_byte(rc);
                if (data != -1)
                {
                    ccrc |= data;
                    return crc_get(rc) == ccrc;
                }
            }
        }
    } while (trys--);

    return false;
}

static uint8_t Read1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t cmd, bool *valid)
{
    if (valid)
        *valid = false;

    uint8_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;

    do {
        flush(rc);

        crc_clear(rc);
        write_byte(rc, address);
        crc_update(rc, address);
        write_byte(rc, cmd);
        crc_update(rc, cmd);

        data = read_byte(rc);
        crc_update(rc, data);
        value = data;

        if (data != -1)
        {
            uint16_t ccrc;
            data = read_byte(rc);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read_byte(rc);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get(rc) == ccrc)
                    {
                        if (valid)
                            *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return 0;
}

static uint16_t Read2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t cmd, bool *valid)
{
    if (valid)
        *valid = false;

    uint16_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;

    do {
        flush(rc);

        crc_clear(rc);
        write_byte(rc, address);
        crc_update(rc, address);
        write_byte(rc, cmd);
        crc_update(rc, cmd);

        data = read_byte(rc);
        crc_update(rc, data);
        value = (uint16_t)data << 8;

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            value |= (uint16_t)data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read_byte(rc);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read_byte(rc);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get(rc) == ccrc)
                    {
                        if (valid)
                            *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return 0;
}

static uint32_t Read4(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t cmd, bool *valid)
{
    if (valid)
        *valid = false;

    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;

    do {
        flush(rc);

        crc_clear(rc);
        write_byte(rc, address);
        crc_update(rc, address);
        write_byte(rc, cmd);
        crc_update(rc, cmd);

        data = read_byte(rc);
        crc_update(rc, data);
        value = (uint32_t)data << 24;

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            value |= (uint32_t)data << 16;
        }

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            value |= (uint32_t)data << 8;
        }

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            value |= (uint32_t)data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read_byte(rc);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read_byte(rc);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get(rc) == ccrc)
                    {
                        if (valid)
                            *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return 0;
}

static uint32_t Read4_1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t cmd, uint8_t *status, bool *valid)
{
    if (valid)
        *valid = false;

    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;

    do {
        flush(rc);

        crc_clear(rc);
        write_byte(rc, address);
        crc_update(rc, address);
        write_byte(rc, cmd);
        crc_update(rc, cmd);

        data = read_byte(rc);
        crc_update(rc, data);
        value = (uint32_t)data << 24;

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            value |= (uint32_t)data << 16;
        }

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            value |= (uint32_t)data << 8;
        }

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            value |= (uint32_t)data;
        }

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            if (status)
                *status = data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read_byte(rc);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read_byte(rc);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get(rc) == ccrc)
                    {
                        if (valid)
                            *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return 0;
}

/******************************************************************************
* Función de inicialización
******************************************************************************/
void RoboClaw_Init(RoboClaw_HandleTypeDef *rc, UART_HandleTypeDef *huart, uint32_t timeout)
{
    rc->huart = huart;
    rc->timeout = timeout;
    rc->crc = 0;
    memset(rc->rxBuffer, 0, ROBOCLAW_RX_BUFFER_SIZE);
}

/******************************************************************************
* Control básico de motores
******************************************************************************/
bool RoboClaw_ForwardM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_M1FORWARD, speed);
}

bool RoboClaw_BackwardM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_M1BACKWARD, speed);
}

bool RoboClaw_ForwardM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_M2FORWARD, speed);
}

bool RoboClaw_BackwardM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_M2BACKWARD, speed);
}

bool RoboClaw_ForwardBackwardM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_M17BIT, speed);
}

bool RoboClaw_ForwardBackwardM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_M27BIT, speed);
}

/******************************************************************************
* Control mixto
******************************************************************************/
bool RoboClaw_ForwardMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_MIXEDFORWARD, speed);
}

bool RoboClaw_BackwardMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_MIXEDBACKWARD, speed);
}

bool RoboClaw_TurnRightMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_MIXEDRIGHT, speed);
}

bool RoboClaw_TurnLeftMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_MIXEDLEFT, speed);
}

bool RoboClaw_ForwardBackwardMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_MIXEDFB, speed);
}

bool RoboClaw_LeftRightMixed(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t speed)
{
    return write_n(rc, 3, address, RC_MIXEDLR, speed);
}

/******************************************************************************
* Encoders
******************************************************************************/
uint32_t RoboClaw_ReadEncM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(rc, address, RC_GETM1ENC, status, valid);
}

uint32_t RoboClaw_ReadEncM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(rc, address, RC_GETM2ENC, status, valid);
}

bool RoboClaw_SetEncM1(RoboClaw_HandleTypeDef *rc, uint8_t address, int32_t val)
{
    return write_n(rc, 6, address, RC_SETM1ENCCOUNT, SetDWORDval(val));
}

bool RoboClaw_SetEncM2(RoboClaw_HandleTypeDef *rc, uint8_t address, int32_t val)
{
    return write_n(rc, 6, address, RC_SETM2ENCCOUNT, SetDWORDval(val));
}

bool RoboClaw_ResetEncoders(RoboClaw_HandleTypeDef *rc, uint8_t address)
{
    return write_n(rc, 2, address, RC_RESETENC);
}

bool RoboClaw_ReadEncoders(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t *enc1, uint32_t *enc2)
{
    return read_n(rc, 2, address, RC_GETENCODERS, enc1, enc2);
}

/******************************************************************************
* Velocidad
******************************************************************************/
uint32_t RoboClaw_ReadSpeedM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(rc, address, RC_GETM1SPEED, status, valid);
}

uint32_t RoboClaw_ReadSpeedM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(rc, address, RC_GETM2SPEED, status, valid);
}

uint32_t RoboClaw_ReadISpeedM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(rc, address, RC_GETM1ISPEED, status, valid);
}

uint32_t RoboClaw_ReadISpeedM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *status, bool *valid)
{
    return Read4_1(rc, address, RC_GETM2ISPEED, status, valid);
}

bool RoboClaw_ReadISpeeds(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t *ispeed1, uint32_t *ispeed2)
{
    return read_n(rc, 2, address, RC_GETISPEEDS, ispeed1, ispeed2);
}

/******************************************************************************
* Control por Duty Cycle
******************************************************************************/
bool RoboClaw_DutyM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty)
{
    return write_n(rc, 4, address, RC_M1DUTY, SetWORDval(duty));
}

bool RoboClaw_DutyM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty)
{
    return write_n(rc, 4, address, RC_M2DUTY, SetWORDval(duty));
}

bool RoboClaw_DutyM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty1, uint16_t duty2)
{
    return write_n(rc, 6, address, RC_MIXEDDUTY, SetWORDval(duty1), SetWORDval(duty2));
}

bool RoboClaw_DutyAccelM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty, uint32_t accel)
{
    return write_n(rc, 8, address, RC_M1DUTYACCEL, SetWORDval(duty), SetDWORDval(accel));
}

bool RoboClaw_DutyAccelM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty, uint32_t accel)
{
    return write_n(rc, 8, address, RC_M2DUTYACCEL, SetWORDval(duty), SetDWORDval(accel));
}

bool RoboClaw_DutyAccelM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2)
{
    return write_n(rc, 14, address, RC_MIXEDDUTYACCEL, SetWORDval(duty1), SetDWORDval(accel1), SetWORDval(duty2), SetDWORDval(accel2));
}

/******************************************************************************
* Control por velocidad
******************************************************************************/
bool RoboClaw_SpeedM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed)
{
    return write_n(rc, 6, address, RC_M1SPEED, SetDWORDval(speed));
}

bool RoboClaw_SpeedM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed)
{
    return write_n(rc, 6, address, RC_M2SPEED, SetDWORDval(speed));
}

bool RoboClaw_SpeedM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed1, uint32_t speed2)
{
    return write_n(rc, 10, address, RC_MIXEDSPEED, SetDWORDval(speed1), SetDWORDval(speed2));
}

bool RoboClaw_SpeedAccelM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed)
{
    return write_n(rc, 10, address, RC_M1SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}

bool RoboClaw_SpeedAccelM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed)
{
    return write_n(rc, 10, address, RC_M2SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}

bool RoboClaw_SpeedAccelM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2)
{
    return write_n(rc, 14, address, RC_MIXEDSPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed1), SetDWORDval(speed2));
}

bool RoboClaw_SpeedAccelM1M2_2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2)
{
    return write_n(rc, 18, address, RC_MIXEDSPEED2ACCEL, SetDWORDval(accel1), SetDWORDval(speed1), SetDWORDval(accel2), SetDWORDval(speed2));
}

/******************************************************************************
* Control por distancia
******************************************************************************/
bool RoboClaw_SpeedDistanceM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
    return write_n(rc, 11, address, RC_M1SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool RoboClaw_SpeedDistanceM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
    return write_n(rc, 11, address, RC_M2SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool RoboClaw_SpeedDistanceM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
    return write_n(rc, 19, address, RC_MIXEDSPEEDDIST, SetDWORDval(speed1), SetDWORDval(distance1), SetDWORDval(speed2), SetDWORDval(distance2), flag);
}

bool RoboClaw_SpeedAccelDistanceM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
    return write_n(rc, 15, address, RC_M1SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool RoboClaw_SpeedAccelDistanceM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
    return write_n(rc, 15, address, RC_M2SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool RoboClaw_SpeedAccelDistanceM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
    return write_n(rc, 23, address, RC_MIXEDSPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed1), SetDWORDval(distance1), SetDWORDval(speed2), SetDWORDval(distance2), flag);
}

bool RoboClaw_SpeedAccelDistanceM1M2_2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
    return write_n(rc, 27, address, RC_MIXEDSPEED2ACCELDIST, SetDWORDval(accel1), SetDWORDval(speed1), SetDWORDval(distance1), SetDWORDval(accel2), SetDWORDval(speed2), SetDWORDval(distance2), flag);
}

/******************************************************************************
* Control por posición
******************************************************************************/
bool RoboClaw_SpeedAccelDeccelPositionM1(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag)
{
    return write_n(rc, 19, address, RC_M1SPEEDACCELDECCELPOS, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(deccel), SetDWORDval(position), flag);
}

bool RoboClaw_SpeedAccelDeccelPositionM2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag)
{
    return write_n(rc, 19, address, RC_M2SPEEDACCELDECCELPOS, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(deccel), SetDWORDval(position), flag);
}

bool RoboClaw_SpeedAccelDeccelPositionM1M2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t deccel1, uint32_t position1, uint32_t accel2, uint32_t speed2, uint32_t deccel2, uint32_t position2, uint8_t flag)
{
    return write_n(rc, 35, address, RC_MIXEDSPEEDACCELDECCELPOS, SetDWORDval(accel1), SetDWORDval(speed1), SetDWORDval(deccel1), SetDWORDval(position1), SetDWORDval(accel2), SetDWORDval(speed2), SetDWORDval(deccel2), SetDWORDval(position2), flag);
}

/******************************************************************************
* PID de velocidad
******************************************************************************/
bool RoboClaw_SetM1VelocityPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps)
{
    uint32_t kp = (uint32_t)(Kp * 65536);
    uint32_t ki = (uint32_t)(Ki * 65536);
    uint32_t kd = (uint32_t)(Kd * 65536);
    return write_n(rc, 18, address, RC_SETM1PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

bool RoboClaw_SetM2VelocityPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps)
{
    uint32_t kp = (uint32_t)(Kp * 65536);
    uint32_t ki = (uint32_t)(Ki * 65536);
    uint32_t kd = (uint32_t)(Kd * 65536);
    return write_n(rc, 18, address, RC_SETM2PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

bool RoboClaw_ReadM1VelocityPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float *Kp, float *Ki, float *Kd, uint32_t *qpps)
{
    uint32_t kp, ki, kd;
    bool valid = read_n(rc, 4, address, RC_READM1PID, &kp, &ki, &kd, qpps);
    *Kp = ((float)kp) / 65536.0f;
    *Ki = ((float)ki) / 65536.0f;
    *Kd = ((float)kd) / 65536.0f;
    return valid;
}

bool RoboClaw_ReadM2VelocityPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float *Kp, float *Ki, float *Kd, uint32_t *qpps)
{
    uint32_t kp, ki, kd;
    bool valid = read_n(rc, 4, address, RC_READM2PID, &kp, &ki, &kd, qpps);
    *Kp = ((float)kp) / 65536.0f;
    *Ki = ((float)ki) / 65536.0f;
    *Kd = ((float)kd) / 65536.0f;
    return valid;
}

/******************************************************************************
* PID de posición
******************************************************************************/
bool RoboClaw_SetM1PositionPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float kp, float ki, float kd, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max)
{
    uint32_t Kp = (uint32_t)(kp * 1024);
    uint32_t Ki = (uint32_t)(ki * 1024);
    uint32_t Kd = (uint32_t)(kd * 1024);
    return write_n(rc, 30, address, RC_SETM1POSPID, SetDWORDval(Kd), SetDWORDval(Kp), SetDWORDval(Ki), SetDWORDval(kiMax), SetDWORDval(deadzone), SetDWORDval(min), SetDWORDval(max));
}

bool RoboClaw_SetM2PositionPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float kp, float ki, float kd, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max)
{
    uint32_t Kp = (uint32_t)(kp * 1024);
    uint32_t Ki = (uint32_t)(ki * 1024);
    uint32_t Kd = (uint32_t)(kd * 1024);
    return write_n(rc, 30, address, RC_SETM2POSPID, SetDWORDval(Kd), SetDWORDval(Kp), SetDWORDval(Ki), SetDWORDval(kiMax), SetDWORDval(deadzone), SetDWORDval(min), SetDWORDval(max));
}

bool RoboClaw_ReadM1PositionPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float *Kp, float *Ki, float *Kd, uint32_t *KiMax, uint32_t *DeadZone, uint32_t *Min, uint32_t *Max)
{
    uint32_t kp, ki, kd;
    bool valid = read_n(rc, 7, address, RC_READM1POSPID, &kp, &ki, &kd, KiMax, DeadZone, Min, Max);
    *Kp = ((float)kp) / 1024.0f;
    *Ki = ((float)ki) / 1024.0f;
    *Kd = ((float)kd) / 1024.0f;
    return valid;
}

bool RoboClaw_ReadM2PositionPID(RoboClaw_HandleTypeDef *rc, uint8_t address, float *Kp, float *Ki, float *Kd, uint32_t *KiMax, uint32_t *DeadZone, uint32_t *Min, uint32_t *Max)
{
    uint32_t kp, ki, kd;
    bool valid = read_n(rc, 7, address, RC_READM2POSPID, &kp, &ki, &kd, KiMax, DeadZone, Min, Max);
    *Kp = ((float)kp) / 1024.0f;
    *Ki = ((float)ki) / 1024.0f;
    *Kd = ((float)kd) / 1024.0f;
    return valid;
}

/******************************************************************************
* Configuración de voltaje
******************************************************************************/
bool RoboClaw_SetMinVoltageMainBattery(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t voltage)
{
    return write_n(rc, 3, address, RC_SETMINMB, voltage);
}

bool RoboClaw_SetMaxVoltageMainBattery(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t voltage)
{
    return write_n(rc, 3, address, RC_SETMAXMB, voltage);
}

bool RoboClaw_SetMinVoltageLogicBattery(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t voltage)
{
    return write_n(rc, 3, address, RC_SETMINLB, voltage);
}

bool RoboClaw_SetMaxVoltageLogicBattery(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t voltage)
{
    return write_n(rc, 3, address, RC_SETMAXLB, voltage);
}

bool RoboClaw_SetMainVoltages(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t min, uint16_t max)
{
    return write_n(rc, 6, address, RC_SETMAINVOLTAGES, SetWORDval(min), SetWORDval(max));
}

bool RoboClaw_SetLogicVoltages(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t min, uint16_t max)
{
    return write_n(rc, 6, address, RC_SETLOGICVOLTAGES, SetWORDval(min), SetWORDval(max));
}

/******************************************************************************
* Lectura de voltaje
******************************************************************************/
uint16_t RoboClaw_ReadMainBatteryVoltage(RoboClaw_HandleTypeDef *rc, uint8_t address, bool *valid)
{
    return Read2(rc, address, RC_GETMBATT, valid);
}

uint16_t RoboClaw_ReadLogicBatteryVoltage(RoboClaw_HandleTypeDef *rc, uint8_t address, bool *valid)
{
    return Read2(rc, address, RC_GETLBATT, valid);
}

bool RoboClaw_ReadMinMaxMainVoltages(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *min, uint16_t *max)
{
    bool valid;
    uint32_t value = Read4(rc, address, RC_GETMINMAXMAINVOLTAGES, &valid);
    if (valid)
    {
        *min = value >> 16;
        *max = value & 0xFFFF;
    }
    return valid;
}

bool RoboClaw_ReadMinMaxLogicVoltages(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *min, uint16_t *max)
{
    bool valid;
    uint32_t value = Read4(rc, address, RC_GETMINMAXLOGICVOLTAGES, &valid);
    if (valid)
    {
        *min = value >> 16;
        *max = value & 0xFFFF;
    }
    return valid;
}

/******************************************************************************
* Lectura de corriente y PWM
******************************************************************************/
bool RoboClaw_ReadCurrents(RoboClaw_HandleTypeDef *rc, uint8_t address, int16_t *current1, int16_t *current2)
{
    bool valid;
    uint32_t value = Read4(rc, address, RC_GETCURRENTS, &valid);
    if (valid)
    {
        *current1 = value >> 16;
        *current2 = value & 0xFFFF;
    }
    return valid;
}

bool RoboClaw_ReadPWMs(RoboClaw_HandleTypeDef *rc, uint8_t address, int16_t *pwm1, int16_t *pwm2)
{
    bool valid;
    uint32_t value = Read4(rc, address, RC_GETPWMS, &valid);
    if (valid)
    {
        *pwm1 = value >> 16;
        *pwm2 = value & 0xFFFF;
    }
    return valid;
}

/******************************************************************************
* Temperatura
******************************************************************************/
bool RoboClaw_ReadTemp(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *temp)
{
    bool valid;
    *temp = Read2(rc, address, RC_GETTEMP, &valid);
    return valid;
}

bool RoboClaw_ReadTemp2(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *temp)
{
    bool valid;
    *temp = Read2(rc, address, RC_GETTEMP2, &valid);
    return valid;
}

/******************************************************************************
* Estado y errores
******************************************************************************/
uint32_t RoboClaw_ReadError(RoboClaw_HandleTypeDef *rc, uint8_t address, bool *valid)
{
    return Read4(rc, address, RC_GETERROR, valid);
}

bool RoboClaw_ReadBuffers(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *depth1, uint8_t *depth2)
{
    bool valid;
    uint16_t value = Read2(rc, address, RC_GETBUFFERS, &valid);
    if (valid)
    {
        *depth1 = value >> 8;
        *depth2 = value & 0xFF;
    }
    return valid;
}

bool RoboClaw_ReadVersion(RoboClaw_HandleTypeDef *rc, uint8_t address, char *version)
{
    uint8_t trys = MAXRETRY;
    int16_t data;

    do {
        flush(rc);
        data = 0;

        crc_clear(rc);
        write_byte(rc, address);
        crc_update(rc, address);
        write_byte(rc, RC_GETVERSION);
        crc_update(rc, RC_GETVERSION);

        for (uint8_t i = 0; i < 48; i++)
        {
            if (data != -1)
            {
                data = read_byte(rc);
                version[i] = data;
                crc_update(rc, version[i]);
                if (version[i] == 0)
                {
                    uint16_t ccrc;
                    data = read_byte(rc);
                    if (data != -1)
                    {
                        ccrc = data << 8;
                        data = read_byte(rc);
                        if (data != -1)
                        {
                            ccrc |= data;
                            return crc_get(rc) == ccrc;
                        }
                    }
                    break;
                }
            }
            else
            {
                break;
            }
        }
    } while (trys--);

    return false;
}

/******************************************************************************
* Configuración de encoders
******************************************************************************/
bool RoboClaw_ReadEncoderModes(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *M1mode, uint8_t *M2mode)
{
    bool valid;
    uint16_t value = Read2(rc, address, RC_GETENCODERMODE, &valid);
    if (valid)
    {
        *M1mode = value >> 8;
        *M2mode = value & 0xFF;
    }
    return valid;
}

bool RoboClaw_SetM1EncoderMode(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t mode)
{
    return write_n(rc, 3, address, RC_SETM1ENCODERMODE, mode);
}

bool RoboClaw_SetM2EncoderMode(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t mode)
{
    return write_n(rc, 3, address, RC_SETM2ENCODERMODE, mode);
}

/******************************************************************************
* Configuración general
******************************************************************************/
bool RoboClaw_SetM1DefaultAccel(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel)
{
    return write_n(rc, 6, address, RC_SETM1DEFAULTACCEL, SetDWORDval(accel));
}

bool RoboClaw_SetM2DefaultAccel(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t accel)
{
    return write_n(rc, 6, address, RC_SETM2DEFAULTACCEL, SetDWORDval(accel));
}

bool RoboClaw_SetPinFunctions(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode)
{
    return write_n(rc, 5, address, RC_SETPINFUNCTIONS, S3mode, S4mode, S5mode);
}

bool RoboClaw_GetPinFunctions(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *S3mode, uint8_t *S4mode, uint8_t *S5mode)
{
    uint8_t trys = MAXRETRY;
    int16_t data;
    uint8_t val1, val2, val3;

    do {
        flush(rc);

        crc_clear(rc);
        write_byte(rc, address);
        crc_update(rc, address);
        write_byte(rc, RC_GETPINFUNCTIONS);
        crc_update(rc, RC_GETPINFUNCTIONS);

        data = read_byte(rc);
        crc_update(rc, data);
        val1 = data;

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            val2 = data;
        }

        if (data != -1)
        {
            data = read_byte(rc);
            crc_update(rc, data);
            val3 = data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read_byte(rc);
            if (data != -1)
            {
                ccrc = data << 8;
                data = read_byte(rc);
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get(rc) == ccrc)
                    {
                        *S3mode = val1;
                        *S4mode = val2;
                        *S5mode = val3;
                        return true;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

bool RoboClaw_SetDeadBand(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t Min, uint8_t Max)
{
    return write_n(rc, 4, address, RC_SETDEADBAND, Min, Max);
}

bool RoboClaw_GetDeadBand(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *Min, uint8_t *Max)
{
    bool valid;
    uint16_t value = Read2(rc, address, RC_GETDEADBAND, &valid);
    if (valid)
    {
        *Min = value >> 8;
        *Max = value & 0xFF;
    }
    return valid;
}

/******************************************************************************
* Límite de corriente
******************************************************************************/
bool RoboClaw_SetM1MaxCurrent(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t max)
{
    return write_n(rc, 10, address, RC_SETM1MAXCURRENT, SetDWORDval(max), SetDWORDval(0));
}

bool RoboClaw_SetM2MaxCurrent(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t max)
{
    return write_n(rc, 10, address, RC_SETM2MAXCURRENT, SetDWORDval(max), SetDWORDval(0));
}

bool RoboClaw_ReadM1MaxCurrent(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t *max)
{
    uint32_t tmax, dummy;
    bool valid = read_n(rc, 2, address, RC_GETM1MAXCURRENT, &tmax, &dummy);
    if (valid)
        *max = tmax;
    return valid;
}

bool RoboClaw_ReadM2MaxCurrent(RoboClaw_HandleTypeDef *rc, uint8_t address, uint32_t *max)
{
    uint32_t tmax, dummy;
    bool valid = read_n(rc, 2, address, RC_GETM2MAXCURRENT, &tmax, &dummy);
    if (valid)
        *max = tmax;
    return valid;
}

/******************************************************************************
* PWM Mode
******************************************************************************/
bool RoboClaw_SetPWMMode(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t mode)
{
    return write_n(rc, 3, address, RC_SETPWMMODE, mode);
}

bool RoboClaw_GetPWMMode(RoboClaw_HandleTypeDef *rc, uint8_t address, uint8_t *mode)
{
    bool valid;
    uint8_t value = Read1(rc, address, RC_GETPWMMODE, &valid);
    if (valid)
    {
        *mode = value;
    }
    return valid;
}

/******************************************************************************
* NVM (Memoria no volátil)
******************************************************************************/
bool RoboClaw_WriteNVM(RoboClaw_HandleTypeDef *rc, uint8_t address)
{
    return write_n(rc, 6, address, RC_WRITENVM, SetDWORDval(0xE22EAB7A));
}

bool RoboClaw_ReadNVM(RoboClaw_HandleTypeDef *rc, uint8_t address)
{
    return write_n(rc, 2, address, RC_READNVM);
}

bool RoboClaw_RestoreDefaults(RoboClaw_HandleTypeDef *rc, uint8_t address)
{
    return write_n(rc, 2, address, RC_RESTOREDEFAULTS);
}

/******************************************************************************
* Configuración
******************************************************************************/
bool RoboClaw_SetConfig(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t config)
{
    return write_n(rc, 4, address, RC_SETCONFIG, SetWORDval(config));
}

bool RoboClaw_GetConfig(RoboClaw_HandleTypeDef *rc, uint8_t address, uint16_t *config)
{
    bool valid;
    uint16_t value = Read2(rc, address, RC_GETCONFIG, &valid);
    if (valid)
    {
        *config = value;
    }
    return valid;
}
