/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t RXData;
uint8_t distance = 0;
uint16_t angle_x = 0;
uint16_t angle_y = 0;
uint16_t angle_z = 0;

volatile uint8_t uartRxBuffer[256];
volatile uint16_t uartRxHead = 0;
volatile uint16_t uartRxTail = 0;

volatile uint32_t uartRxErrors = 0;
volatile uint32_t uartRxOverflow = 0;

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

uint32_t previousTime = 0;
uint8_t angleInitialized = 0;

#define RAD_TO_DEG 57.2957795f
#define FILTER_ALPHA 0.98f

// Sampling frequency
#define IMU_INTERVAL_MS 100U

// Complementary filter time constant
#define FILTER_TAU 0.5f

// Gyroscope offsets
static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;

// Sampling state
static uint32_t previousIMUTime = 0;
static uint8_t imuInitialized = 0;

// Latest sensor data
static float Ax, Ay, Az;
static float Gx, Gy, Gz;

#define GRAVITY 9.80665f
#define DEG_TO_RAD 0.01745329252f

// Experimental acceleration noise threshold
#define ACCEL_DEADBAND 0.10f

static float velocityX = 0.0f;
static float velocityY = 0.0f;

static float positionX = 0.0f;
static float positionY = 0.0f;

// Estimated total horizontal distance traveled
static float distanceMeters = 0.0f;

static uint32_t previousDistanceTime = 0;
static uint8_t distanceInitialized = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void UART_StartReceive(void);
static void UART_ProcessReceivedData(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_StartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, &RXData, 1);
}


static void UART_ProcessReceivedData(void)
{
    uint8_t data;

    while (uartRxHead != uartRxTail)
    {
        data = uartRxBuffer[uartRxTail];

        uartRxTail = (uartRxTail + 1) % 256;

        // Forward received data to PC through USART2
        HAL_UART_Transmit(&huart2, &data, 1, 100);

        // Toggle LED when a newline is received
        if (data == 'S')
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        } else if (data == 'F'){
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        } else if (data == 'L'){
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        } else if (data == 'R'){
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        } else if (data == 'B'){
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        }

    }
}

static HAL_StatusTypeDef MPU6050_ReadAll(void)
{
    uint8_t data[14];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        ACCEL_XOUT_H_REG,
        I2C_MEMADD_SIZE_8BIT,
        data,
        sizeof(data),
        100
    );

    if (status != HAL_OK)
    {
        return status;
    }

    int16_t axRaw = (int16_t)(
        ((uint16_t)data[0] << 8) | data[1]
    );

    int16_t ayRaw = (int16_t)(
        ((uint16_t)data[2] << 8) | data[3]
    );

    int16_t azRaw = (int16_t)(
        ((uint16_t)data[4] << 8) | data[5]
    );

    // Bytes 6-7 contain temperature.

    int16_t gxRaw = (int16_t)(
        ((uint16_t)data[8] << 8) | data[9]
    );

    int16_t gyRaw = (int16_t)(
        ((uint16_t)data[10] << 8) | data[11]
    );

    int16_t gzRaw = (int16_t)(
        ((uint16_t)data[12] << 8) | data[13]
    );

    // Accelerometer: +/- 2g
    Ax = (float)axRaw / 16384.0f;
    Ay = (float)ayRaw / 16384.0f;
    Az = (float)azRaw / 16384.0f;

    // Gyroscope: +/- 250 degrees/second
    Gx = (float)gxRaw / 131.0f;
    Gy = (float)gyRaw / 131.0f;
    Gz = (float)gzRaw / 131.0f;

    return HAL_OK;
}

static HAL_StatusTypeDef MPU6050_CalibrateGyro(void)
{
    const uint16_t samples = 500;

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    // Allow initial readings to settle.
    for (uint16_t i = 0; i < 50; i++)
    {
        if (MPU6050_ReadAll() != HAL_OK)
            return HAL_ERROR;

        HAL_Delay(5);
    }

    for (uint16_t i = 0; i < samples; i++)
    {
        if (MPU6050_ReadAll() != HAL_OK)
        {
            return HAL_ERROR;
        }

        sumX += Gx;
        sumY += Gy;
        sumZ += Gz;

        HAL_Delay(5);
    }

    gyroBiasX = sumX / samples;
    gyroBiasY = sumY / samples;
    gyroBiasZ = sumZ / samples;

    return HAL_OK;
}

static float NormalizeAngle(float angle)
{
    while (angle >= 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static void PrintDataMPU(void)
{
    char message[128];

//    // Read accelerometer and gyroscope
//    MPU6050_Read_Accel();
//    MPU6050_Read_Gyro();
//
//    // Calculate elapsed time in seconds
//    uint32_t currentTime = HAL_GetTick();
//
//    float dt = (currentTime - previousTime) / 1000.0f;
//
//    previousTime = currentTime;
//
//    // Calculate angles from accelerometer (degrees)
//    float accelRoll = atan2f(Ay, Az) * RAD_TO_DEG;
//
//    float accelPitch = atan2f(
//        -Ax,
//        sqrtf(Ay * Ay + Az * Az)
//    ) * RAD_TO_DEG;
//
//    if (!angleInitialized)
//    {
//        // Initialize angles from accelerometer
//        roll = accelRoll;
//        pitch = accelPitch;
//        yaw = 0.0f;
//
//        angleInitialized = 1;
//    }
//    else
//    {
//        // Complementary filter for roll
//        roll = FILTER_ALPHA * (roll + Gx * dt)
//             + (1.0f - FILTER_ALPHA) * accelRoll;
//
//        // Complementary filter for pitch
//        pitch = FILTER_ALPHA * (pitch + Gy * dt)
//              + (1.0f - FILTER_ALPHA) * accelPitch;
//
//        // Integrate gyroscope Z for relative yaw
//        yaw += Gz * dt;
//    }

    // Print angles through UART2
    snprintf(message, sizeof(message),
             "ANGLE: Roll=%.2f Pitch=%.2f Yaw=%.2f\r\n",
             roll, pitch, yaw);

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)message,
        strlen(message),
        1000
    );
}

static void ReadDataMPU(void)
{
    uint32_t currentTime = HAL_GetTick();

    if (MPU6050_ReadAll() != HAL_OK)
    {
        // Do not update orientation using stale data.
        return;
    }

    // Accelerometer angles
    float accelRoll = atan2f(Ay, Az) * RAD_TO_DEG;

    float accelPitch = atan2f(
        -Ax,
        sqrtf(Ay * Ay + Az * Az)
    ) * RAD_TO_DEG;

    // Remove gyroscope zero-rate offsets
    float gx = Gx - gyroBiasX;
    float gy = Gy - gyroBiasY;
    float gz = Gz - gyroBiasZ;

    // First valid sample
    if (!imuInitialized)
    {
        roll = accelRoll;
        pitch = accelPitch;
        yaw = 0.0f;

        previousIMUTime = currentTime;
        imuInitialized = 1;

        return;
    }

    float dt =
        (currentTime - previousIMUTime) / 1000.0f;

    previousIMUTime = currentTime;

    // Reject zero or unusually long sampling intervals.
    // A long interval should not be blindly integrated.
    if (dt <= 0.0f || dt > 0.1f)
    {
        return;
    }

    // Time-based complementary filter
    float alpha = FILTER_TAU / (FILTER_TAU + dt);

    // Gyroscope prediction
    float gyroRoll = NormalizeAngle(
        roll + gx * dt
    );

    float gyroPitch = pitch + gy * dt;

    // Accelerometer magnitude in g
    float accelMagnitude = sqrtf(
        Ax * Ax +
        Ay * Ay +
        Az * Az
    );

    // Accelerometer tilt is most reliable when
    // acceleration is approximately gravity.

    if (accelMagnitude > 0.85f && accelMagnitude < 1.15f)
    {
        // Correct roll using shortest angular difference.
        float rollError = NormalizeAngle(
            accelRoll - gyroRoll
        );

        roll = NormalizeAngle(
            gyroRoll + (1.0f - alpha) * rollError
        );

        // Correct pitch
        pitch = alpha * gyroPitch
              + (1.0f - alpha) * accelPitch;
    }
    else
    {
        // During strong linear acceleration,
        // temporarily rely on gyroscope prediction.
        roll = gyroRoll;
        pitch = gyroPitch;
    }

    // Relative yaw: gyro only
    yaw = NormalizeAngle(
        yaw + gz * dt
    );
}

//static void ReadDataMPU(void)
//{
//    MPU6050_Read_Accel();
//    MPU6050_Read_Gyro();
//
//    uint32_t currentTime = HAL_GetTick();
//
//    float dt = (currentTime - previousTime) / 1000.0f;
//
//    previousTime = currentTime;
//
//    float accelRoll = atan2f(Ay, Az) * RAD_TO_DEG;
//
//    float accelPitch = atan2f(
//        -Ax,
//        sqrtf(Ay * Ay + Az * Az)
//    ) * RAD_TO_DEG;
//
//    if (!angleInitialized)
//    {
//        roll = accelRoll;
//        pitch = accelPitch;
//        yaw = 0.0f;
//
//        angleInitialized = 1;
//    }
//    else
//    {
//        roll = FILTER_ALPHA * (roll + Gx * dt)
//             + (1.0f - FILTER_ALPHA) * accelRoll;
//
//        pitch = FILTER_ALPHA * (pitch + Gy * dt)
//              + (1.0f - FILTER_ALPHA) * accelPitch;
//
//        yaw += Gz * dt;
//
//        // Normalize yaw to [-180, 180)
//        while (yaw >= 180.0f)
//            yaw -= 360.0f;
//
//        while (yaw < -180.0f)
//            yaw += 360.0f;
//    }
//}

static void SendSensorData(void)
{
    char txBuffer[48];

    // Example distance: 125.75 cm
    // Convert estimated distance from meters
    // to centimeters, scaled by 100.
    float distanceCM = distanceMeters * 100.0f;

    // UART protocol supports up to 99999.99 cm
    if (distanceCM < 0.0f)
        distanceCM = 0.0f;

    if (distanceCM > 99999.99f)
        distanceCM = 99999.99f;

    uint32_t distance100 = (uint32_t)lroundf(distanceCM * 100.0f);

    // Convert MPU6050 angles to tenths of degrees
    int32_t roll10  = (int32_t)lroundf(roll * 10.0f);
    int32_t pitch10 = (int32_t)lroundf(pitch * 10.0f);
    int32_t yaw10   = (int32_t)lroundf(yaw * 10.0f);

    // Clamp to representable range
    if (roll10 > 9999) roll10 = 9999;
    if (roll10 < -9999) roll10 = -9999;

    if (pitch10 > 9999) pitch10 = 9999;
    if (pitch10 < -9999) pitch10 = -9999;

    if (yaw10 > 9999) yaw10 = 9999;
    if (yaw10 < -9999) yaw10 = -9999;

    int len = snprintf(
        txBuffer,
        sizeof(txBuffer),

        "%05lu.%02lu,"
        "%c%03ld.%01ld,"
        "%c%03ld.%01ld,"
        "%c%03ld.%01ld\n",

        (unsigned long)(distance100 / 100),
        (unsigned long)(distance100 % 100),

        roll10 < 0 ? '-' : '+',
        (long)(labs((long)roll10) / 10),
        (long)(labs((long)roll10) % 10),

        pitch10 < 0 ? '-' : '+',
        (long)(labs((long)pitch10) / 10),
        (long)(labs((long)pitch10) % 10),

        yaw10 < 0 ? '-' : '+',
        (long)(labs((long)yaw10) / 10),
        (long)(labs((long)yaw10) % 10)
    );

    if (len > 0 && len < sizeof(txBuffer))
    {
        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)txBuffer,
            (uint16_t)len,
            100
        );
//        HAL_UART_Transmit(
//			&huart2,
//			(uint8_t *)txBuffer,
//			(uint16_t)len,
//			100
//		);
    }
}

static void CalculateDistance(void)
{
    uint32_t now = HAL_GetTick();

    if (!distanceInitialized)
    {
        previousDistanceTime = now;
        distanceInitialized = 1;
        return;
    }

    float dt = (now - previousDistanceTime) / 1000.0f;
    previousDistanceTime = now;

    if (dt <= 0.0f || dt > 0.05f)
    {
        return;
    }

    // Convert angles to radians
    float r = roll * DEG_TO_RAD;
    float p = pitch * DEG_TO_RAD;
    float y = yaw * DEG_TO_RAD;

    float cr = cosf(r);
    float sr = sinf(r);

    float cp = cosf(p);
    float sp = sinf(p);

    float cy = cosf(y);
    float sy = sinf(y);

    // =====================================
    // 1. Remove gravity (body coordinates)
    // =====================================

    float linearAx = Ax + sp;

    float linearAy = Ay - sr * cp;

    float linearAz = Az - cr * cp;

    // Convert acceleration to m/s^2
    linearAx *= GRAVITY;
    linearAy *= GRAVITY;
    linearAz *= GRAVITY;

    // =====================================
    // 2. Transform to horizontal world frame
    // =====================================

    float worldAx =
        (cy * cp) * linearAx
      + (cy * sp * sr - sy * cr) * linearAy
      + (cy * sp * cr + sy * sr) * linearAz;

    float worldAy =
        (sy * cp) * linearAx
      + (sy * sp * sr + cy * cr) * linearAy
      + (sy * sp * cr - cy * sr) * linearAz;

    // =====================================
    // 3. Apply a basic noise threshold
    // =====================================

    if (fabsf(worldAx) < ACCEL_DEADBAND)
        worldAx = 0.0f;

    if (fabsf(worldAy) < ACCEL_DEADBAND)
        worldAy = 0.0f;

    // =====================================
    // 4. Integrate acceleration
    // =====================================

    float dx = velocityX * dt
             + 0.5f * worldAx * dt * dt;

    float dy = velocityY * dt
             + 0.5f * worldAy * dt * dt;

    velocityX += worldAx * dt;
    velocityY += worldAy * dt;

    // =====================================
    // 5. Update position and distance
    // =====================================

    positionX += dx;
    positionY += dy;

    distanceMeters += sqrtf(dx * dx + dy * dy);
}

static void ResetVelocity(void)
{
    velocityX = 0.0f;
    velocityY = 0.0f;
}

//static void SendSensorData(void)
//{
//    char txBuffer[40];
//
//    // Example values in scaled integer format
//    ReadDataMPU();
//    uint32_t distance100 = 12575; // 125.75
//    uint16_t angleX10 = (uint16_t *) roll*10;   // 90.5 -> 905
//    uint16_t angleY10 = (uint16_t *) pitch*10;  // 180.0-> 1800
//    uint16_t angleZ10 = (uint16_t *) yaw*10;	// 45.2 -> 452
//
//    int len = snprintf(
//        txBuffer,
//        sizeof(txBuffer),
//        "%05lu.%02lu,%03u.%01u,%03u.%01u,%03u.%01u\n",
//
//        (unsigned long)(distance100 / 100),
//        (unsigned long)(distance100 % 100),
//
//        (unsigned int)(angleX10 / 10),
//        (unsigned int)(angleX10 % 10),
//
//        (unsigned int)(angleY10 / 10),
//        (unsigned int)(angleY10 % 10),
//
//        (unsigned int)(angleZ10 / 10),
//        (unsigned int)(angleZ10 % 10)
//    );
//
//    if (len > 0 && len < sizeof(txBuffer))
//    {
//        HAL_UART_Transmit(
//            &huart1,
//            (uint8_t *)txBuffer,
//            (uint16_t)len,
//            100
//        );
//    }
//}

void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	if (HAL_I2C_Mem_Read(
	        &hi2c1,
	        MPU6050_ADDR,
			0x3B,
	        I2C_MEMADD_SIZE_8BIT,
	        Rec_Data,
	        6,
	        1000) != HAL_OK)
	{
	    return;
	}

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Ax = (float)Accel_X_RAW/16384.0;
	Ay = (float)Accel_Y_RAW/16384.0;
	Az = (float)Accel_Z_RAW/16384.0;
}

void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	if (HAL_I2C_Mem_Read(
			&hi2c1,
			MPU6050_ADDR,
			0x43,
			I2C_MEMADD_SIZE_8BIT,
			Rec_Data,
			6,
			1000) != HAL_OK)
	{
		return;
	}

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Gx = (float)Gyro_X_RAW/131.0;
	Gy = (float)Gyro_Y_RAW/131.0;
	Gz = (float)Gyro_Z_RAW/131.0;
}

static void MPU6050_TestConnection(void)
{
    char msg[100];
    uint8_t who_am_i = 0;

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        WHO_AM_I_REG,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i,
        1,
        1000
    );

    if (status == HAL_OK)
    {
        snprintf(msg, sizeof(msg),
                 "MPU6050 WHO_AM_I = 0x%02X\r\n",
                 who_am_i);
    }
    else
    {
        snprintf(msg, sizeof(msg),
                 "MPU6050 I2C ERROR: %d (HAL error: 0x%08lX)\r\n",
                 (int)status,
                 (unsigned long)HAL_I2C_GetError(&hi2c1));
    }

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)msg,
        strlen(msg),
        1000
    );
}

//static void I2C_Scan(void)
//{
//    char msg[100];
//    uint8_t found = 0;
//
//    HAL_UART_Transmit(
//        &huart2,
//        (uint8_t *)"\r\nScanning I2C bus...\r\n",
//        strlen("\r\nScanning I2C bus...\r\n"),
//        1000
//    );
//
//    for (uint8_t addr = 1; addr < 127; addr++)
//    {
//        if (HAL_I2C_IsDeviceReady(
//                &hi2c1,
//                addr << 1,
//                2,
//                100) == HAL_OK)
//        {
//            snprintf(msg, sizeof(msg),
//                     "I2C device found at 0x%02X\r\n",
//                     addr);
//
//            HAL_UART_Transmit(
//                &huart2,
//                (uint8_t *)msg,
//                strlen(msg),
//                1000
//            );
//
//            found = 1;
//        }
//    }
//
//    if (!found)
//    {
//        char errorMsg[] = "No I2C devices found!\r\n";
//
//        HAL_UART_Transmit(
//            &huart2,
//            (uint8_t *)errorMsg,
//            strlen(errorMsg),
//            1000
//        );
//    }
//}
//static void I2C_Debug(void)
//{
//    char msg[128];
//
//    uint32_t state = HAL_I2C_GetState(&hi2c1);
//    uint32_t error = HAL_I2C_GetError(&hi2c1);
//
//    uint32_t busy =
//        (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET);
//
//    GPIO_PinState scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
//    GPIO_PinState sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
//
//    snprintf(msg, sizeof(msg),
//             "I2C State: %lu | Error: 0x%08lX | BUSY: %lu | SCL: %d | SDA: %d\r\n",
//             (unsigned long)state,
//             (unsigned long)error,
//             (unsigned long)busy,
//             (int)scl,
//             (int)sda);
//
//    HAL_UART_Transmit(
//        &huart2,
//        (uint8_t *)msg,
//        strlen(msg),
//        1000
//    );
//}
//static void MPU6050_CheckAddress(void)
//{
//    char msg[100];
//
//    HAL_StatusTypeDef status;
//
//    status = HAL_I2C_IsDeviceReady(
//        &hi2c1,
//        0x68 << 1,
//        2,
//        100
//    );
//
//    snprintf(msg, sizeof(msg),
//             "Address 0x68: Status=%d Error=0x%08lX\r\n",
//             status,
//             (unsigned long)HAL_I2C_GetError(&hi2c1));
//
//    HAL_UART_Transmit(&huart2,
//                      (uint8_t *)msg,
//                      strlen(msg),
//                      1000);
//
//    I2C_Debug();
//
//    // Only test 0x69 if the bus is still idle.
//    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET)
//    {
//        status = HAL_I2C_IsDeviceReady(
//            &hi2c1,
//            0x69 << 1,
//            2,
//            100
//        );
//
//        snprintf(msg, sizeof(msg),
//                 "Address 0x69: Status=%d Error=0x%08lX\r\n",
//                 status,
//                 (unsigned long)HAL_I2C_GetError(&hi2c1));
//
//        HAL_UART_Transmit(&huart2,
//                          (uint8_t *)msg,
//                          strlen(msg),
//                          1000);
//
//        I2C_Debug();
//    }
//}

//static void I2C1_ResetPeripheral(void)
//{
//    HAL_I2C_DeInit(&hi2c1);
//
//    __HAL_RCC_I2C1_FORCE_RESET();
//
//    HAL_Delay(10);
//
//    __HAL_RCC_I2C1_RELEASE_RESET();
//
//    HAL_Delay(10);
//
//    MX_I2C1_Init();
//
//    HAL_Delay(100);
//}

static void MPU6050_CheckWHOAMI(void)
{
    uint8_t id = 0;
    char msg[64];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        0x75,
        I2C_MEMADD_SIZE_8BIT,
        &id,
        1,
        100
    );

    if (status == HAL_OK)
    {
        snprintf(msg, sizeof(msg),
                 "WHO_AM_I = 0x%02X\r\n", id);
    }
    else
    {
        snprintf(msg, sizeof(msg),
                 "WHO_AM_I READ ERROR: %d\r\n", status);
    }

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)msg,
        strlen(msg),
        100
    );
}

static HAL_StatusTypeDef MPU6050_Init(void)
{
    uint8_t data;
    HAL_StatusTypeDef status;

    // Wake up MPU6050
    data = 0x00;

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x6B,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );

    if (status != HAL_OK)
        return status;

    HAL_Delay(100);

    // Configure sample rate
    data = 0x07;

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x19,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );

    if (status != HAL_OK)
        return status;

    // Accelerometer ±2g
    data = 0x00;

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x1C,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );

    if (status != HAL_OK)
        return status;

    // Gyroscope ±250 degrees/s
    data = 0x00;

    status = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        0x1B,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );

    return status;
}

void SetRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    uint32_t r = (red * 999U) / 255U;
    uint32_t g = (green * 999U) / 255U;
    uint32_t b = (blue * 999U) / 255U;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, r);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, g);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, b);
}

static uint8_t KY032_IsObstacle(void)
{
    return HAL_GPIO_ReadPin(
        GPIOB,
        GPIO_PIN_5
    ) == GPIO_PIN_RESET;
}

static void KY032_Debug(void)
{
    static uint32_t lastPrint = 0;

    if (HAL_GetTick() - lastPrint < 200)
        return;

    lastPrint = HAL_GetTick();

    GPIO_PinState sensor =
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

    GPIO_PinState enable =
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);

    char msg[100];

    int len = snprintf(
        msg,
        sizeof(msg),
        "KY032 EN=%d OUT=%d Obstacle=%d\r\n",
        (int)enable,
        (int)sensor,
        KY032_IsObstacle()
    );

    if (len > 0 && len < sizeof(msg))
    {
        HAL_UART_Transmit(
            &huart2,
            (uint8_t*)msg,
            (uint16_t)len,
            100
        );
    }



}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_Delay(100);

  MPU6050_CheckWHOAMI();
  MPU6050_TestConnection();


  HAL_StatusTypeDef mpuStatus = MPU6050_Init();

  if (mpuStatus == HAL_OK)
  {
      char msg[] = "MPU6050 initialized successfully!\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)msg,
          strlen(msg),
          100
      );
  }
  else
  {
      char msg[] = "MPU6050 initialization FAILED!\r\n";

      HAL_UART_Transmit(
          &huart2,
          (uint8_t *)msg,
          strlen(msg),
          100
      );
  }

  if (mpuStatus != HAL_OK)
  {
      Error_Handler();
  }

  // Keep MPU6050 stationary during calibration.
  HAL_StatusTypeDef calibrationStatus =
      MPU6050_CalibrateGyro();

  if (calibrationStatus != HAL_OK)
  {
      Error_Handler();
  }

  UART_StartReceive();

  // Timer initialization
  uint32_t lastIMU = HAL_GetTick();
  uint32_t lastSend = HAL_GetTick();
//  uint32_t lastSend = HAL_GetTick();
//  I2C_Scan();
//  I2C_Debug();
//  MPU6050_TestConnection();
//  MPU6050_Init();

  char startMsg[] = "\r\nSTM32 UART Ready!\r\n";
  char waitingMsg[] = "Waiting for ESP8266 data...\r\n";

  HAL_UART_Transmit(
	  &huart2,
	  (uint8_t *)startMsg,
	  strlen(startMsg),
	  HAL_MAX_DELAY
  );
  HAL_UART_Transmit(
	  &huart2,
	  (uint8_t *)waitingMsg,
	  strlen(waitingMsg),
	  HAL_MAX_DELAY
  );

  // Start receiving UART data from ESP8266
  UART_StartReceive();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	UART_ProcessReceivedData();
//
//	uint32_t now = HAL_GetTick();
//
//	// =====================================
//	// MPU6050 UPDATE: 100 Hz
//	// =====================================
//
//	if ((uint32_t)(now - lastIMU) >= IMU_INTERVAL_MS)
//	{
//		lastIMU = now;
//
//		ReadDataMPU();
//		PrintDataMPU();
//		SendSensorData();
//	}

	UART_ProcessReceivedData();

	uint32_t now = HAL_GetTick();

	// IMU update at 100 Hz
	if (now - lastIMU >= 10)
	{
		lastIMU = now;

		ReadDataMPU();

		// Call only after a successful fresh IMU reading
		CalculateDistance();
//
//		PrintDataMPU();
	}

	// UART transmission at 2 Hz
	if (now - lastSend >= 500)
	{
		lastSend = now;

		SendSensorData();
	}

	if(KY032_IsObstacle()){

		SetRGB(
			(uint8_t)50,
			(uint8_t)50,
			(uint8_t)50
		);

		HAL_Delay(10);
	}else{
		SetRGB(
			(uint8_t)0,
			(uint8_t)0,
			(uint8_t)0
		);

		HAL_Delay(10);
	}

	KY032_Debug();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint16_t nextHead = (uartRxHead + 1) % 256;

        if (nextHead != uartRxTail)
        {
            // Store received byte in buffer
            uartRxBuffer[uartRxHead] = RXData;

            uartRxHead = nextHead;
        }
        else
        {
            // Buffer full
            uartRxOverflow++;
        }

        // Restart UART reception for next byte
        HAL_UART_Receive_IT(&huart1, &RXData, 1);
    }
}

/**
  * @brief UART error callback
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uartRxErrors++;

        // Restart reception after UART errors
        HAL_UART_Receive_IT(&huart1, &RXData, 1);
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
