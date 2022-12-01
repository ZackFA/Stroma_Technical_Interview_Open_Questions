/*
 * easy_test.c
 *
 *  Created on: Nov 27, 2022
 *      Author: macintosh
 */

/*****************************************************
 *             				                               *
 * 	    This code initialises some                   *
 * 	    memory base addresses                        *
 * 	    as well as reads & manipulates data	         *
 * 	    received & applies Kalman filter             *
 * 	    for the MPU6050 sensor.                      *
 * 		                                               *
 *                                                   *
 *****************************************************/


#include <math.h>
#include "mpu6050.h"

/*** MACRO DEFINITIONS FOR THE SLAVE (MPU6050) MEMORY ADDRESS BASES ***/

#define RAD2DEG 57.295779513082320876798154814105
#define WHOAMI 0x75
#define PWR_MGMT 0x6B
#define SMPLRT_DIV 0x19
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define MPU6050_ADDR 0xD0
const uint16_t I2C_TIMEOUT = 100;
const double Acc_Z_corrector = 14418.0;
uint32_t timer;

Filter_t FilterX = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f};

Filter_t FilterY = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f,
};
/*************************************************/



uint8_t MPU_Init(I2C_HandleTypeDef *I2Cx)
{

	/** GENERAL INITIALISATION FOR THE MPU6050 SENSOR **/

    uint8_t check;
    uint8_t Data;

    /*** Reading Data & Assigning it to check from WHO AM I ***/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHOAMI, 1, &check, 1, I2C_TIMEOUT);


    /*** Writing Memory Location on Slave Device for Different Purposes ***/
    if (check == 104) // checks if the value received from slave is correct
    {
    	/* Assigning Memory address for Power Management Configuration */
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT, 1, &Data, 1, I2C_TIMEOUT);

        /* Assigning Memory address for Sample Rate Divider Configuration */
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, I2C_TIMEOUT);

        /* Assigning Memory address for Acceleration Configuration */
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, I2C_TIMEOUT);

        /* Assigning Memory address for Gyroscope Configuration */
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, I2C_TIMEOUT);
        return 0;
    }
    return 1;
}



/** READING ACCELERATION DATA **/
void Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6]; // Reading 6 bits

    /*** Requesting Data from Slave using a specific memory addr ***/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);


    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); // Assigning Read Data to X Coord. & Shifting It to corresponding Arr. Element
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]); // Assigning Read Data to Y Coord. & Shifting It to corresponding Arr. Element
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]); // Assigning Read Data to Z Coord. & Shifting It to corresponding Arr. Element

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0; // Applying Unit Conversion
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0; // Applying Unit Conversion
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector; // Applying Unit Conversion
}


/** READING GYRO DATA **/
void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6]; // Reading 6 bits

    /*** Requesting Data from Slave using a specific memory addr ***/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); // Assigning Read Data to X Coord. & Shifting It to corresponding Arr. Element
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]); // Assigning Read Data to X Coord. & Shifting It to corresponding Arr. Element
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]); // Assigning Read Data to X Coord. & Shifting It to corresponding Arr. Element

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0; // Applying Unit Conversion
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0; // Applying Unit Conversion
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0; // Applying Unit Conversion
}


/** READING TEMPERATURE DATA **/
void Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2]; // Reading 2 bits
    int16_t temp;

    /*** Requesting Data from Slave using a specific memory addr ***/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2, I2C_TIMEOUT);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); // Assigning Read Data for Temperature & Shifting It to corresponding Arr. Element
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53); // Applying Unit Conversion
}


/** READING ALL DATA STARTING FROM ACCELERATION TO TEMPERATURE **/
void Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14]; // Reading 14 bits
    int16_t temp;

    /*** Requesting Data from Slave using a specific memory addr ***/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 14, I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); // Assigning Read Data to X Coord. & Shifting It to corresponding Arr. Element
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]); // Assigning Read Data to Y Coord. & Shifting It to corresponding Arr. Element
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]); // Assigning Read Data to Z Coord. & Shifting It to corresponding Arr. Element
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]); // Assigning Read Data for Temperature & Shifting It to corresponding Arr. Element
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]); // Assigning Read Data to X Coord. & Shifting It to corresponding Arr. Element
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]); // Assigning Read Data to X Coord. & Shifting It to corresponding Arr. Element
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]); // Assigning Read Data to X Coord. & Shifting It to corresponding Arr. Element

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0; // Applying Unit Conversion
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0; // Applying Unit Conversion
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector; // Applying Unit Conversion
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53); // Applying Unit Conversion
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0; // Applying Unit Conversion
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0; // Applying Unit Conversion
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0; // Applying Unit Conversion


    /*** KALMAN FILTER SPECIFICS ***/

    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD2DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD2DEG;
    if ((pitch < -90 && DataStruct->FilterAngleY > 90) || (pitch > 90 && DataStruct->FilterAngleY < -90))
    {
        FilterY.angle = pitch;
        DataStruct->FilterAngleY = pitch;
    }
    else
    {
        DataStruct->FilterAngleY = Filter_getAngle(&FilterY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->FilterAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->FilterAngleX = Filter_getAngle(&FilterX, roll, DataStruct->Gx, dt);
}


/*** KALMAN FILTER ANGLE CALCULATION API ***/
double Filter_getAngle(Filter_t *Filter, double newAngle, double newRate, double dt)
{
    double rate = newRate - Filter->bias;
    Filter->angle += dt * rate;

    Filter->P[0][0] += dt * (dt * Filter->P[1][1] - Filter->P[0][1] - Filter->P[1][0] + Filter->Q_ANGLE);
    Filter->P[0][1] -= dt * Filter->P[1][1];
    Filter->P[1][0] -= dt * Filter->P[1][1];
    Filter->P[1][1] += Filter->Q_BIAS * dt;

    double S = Filter->P[0][0] + Filter->R_MEASURE;
    double K[2];
    K[0] = Filter->P[0][0] / S;
    K[1] = Filter->P[1][0] / S;

    double y = newAngle - Filter->angle;
    Filter->angle += K[0] * y;
    Filter->bias += K[1] * y;

    double P00_temp = Filter->P[0][0];
    double P01_temp = Filter->P[0][1];

    Filter->P[0][0] -= K[0] * P00_temp;
    Filter->P[0][1] -= K[0] * P01_temp;
    Filter->P[1][0] -= K[1] * P00_temp;
    Filter->P[1][1] -= K[1] * P01_temp;

    return Filter->angle;
};
Footer
© 2022 GitHub, Inc.
Footer navigation
Terms
Privacy
Security
Status
Docs
Contact GitHub
Pricing
API
Training
Blog
About
Stroma-Technical-Interview/easy_test.c at main · ZackFA/Stroma-Technical-Interview
