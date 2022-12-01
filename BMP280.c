#include "BMP280.h"



void BMP280_INIT(BMP280_ConfigTypeDef_t *pConfig)
{

    pConfig->Mode = Mode_Normal;
    pConfig->Filter = Filter_OFF;
    pConfig->Oversampling_Pres = STANDARD;
    pConfig->Oversampling_Temp = STANDARD;
    pConfig->Standby = StandbyTime_250;

}

void I2C_READ(BMP280_HandleDef *pHandle, uint8_t addr, uint8_t *pVal)
{

    uint16_t tx_buff;
    uint8_t rx_buff[2];
    tx_buff = (pHandle->addr << 1);

    if(HAL_I2C_Mem_Read(pHandle->i2c, tx_buff, addr, 1,rx_buff,2, 5000) == HAL_OK)
    {
        *pVal = (uint16_t)((rx_buff[1] << 8 | rx_buff[0]));
    }


}

void I2C_WRITE(BMP280_HandleDef *pHandle, uint8_t addr, uint8_t Val)
{
    uint16_t tx_buff;
    tx_buff = (pHandle->addr << 1);
    HAL_I2C_Mem_Read(pHandle->i2c, tx_buff, addr, 1,rx_buff,2, 5000)
}

void Get_Calib_Data(BMP280_HandleDef *pHandle)
{

    if(I2C_READ(*pHandle, 0x88, &pHandle->dig_T1)
                && I2C_READ(pHandle, 0x8a, (uint16_t *)&pHandle->dig_T2)
                && I2C_READ(pHandle, 0x8c, (uint16_t *)&pHandle->dig_T2)
                && I2C_READ(pHandle, 0x8e, &pHandle->dig_P1)
                && I2C_READ(pHandle, 0x90, (uint16_t *)&pHandle->dig_P2)
                && I2C_READ(pHandle, 0x92, (uint16_t *)&pHandle->dig_P3)
                && I2C_READ(pHandle, 0x94, (uint16_t *)&pHandle->dig_P4)
                && I2C_READ(pHandle, 0x96, (uint16_t *)&pHandle->dig_P5)
                && I2C_READ(pHandle, 0x98, (uint16_t *)&pHandle->dig_P6)
                && I2C_READ(pHandle, 0x9a, (uint16_t *)&pHandle->dig_P7)
                && I2C_READ(pHandle, 0x9c, (uint16_t *)&pHandle->dig_P8)
                && I2C_READ(pHandle, 0x9a, (uint16_t *)&pHandle->dig_P9))

                
}

/*** CALCULATION OF THE PRESSURE, RETURN VALUES IN CELSIUS  ***/
int32_t Calc_Temp(BMP280_HandleDef *pHandle, int32_t tempTemp, int32_t *fin_temp)
{

    int32_t var1, var2;

    var1 = ((((tempTemp >> 3) - ((int32_t) pHandle->dig_T1 << 1))) 
             * (int32_t) pHandle->dig_T2) >> 11;

    var2 = (((((tempTemp >> 4) - (int32_t) pHandle->dig_T1) 
             * ((tempTemp >> 4)) - (int32_t) pHandle->dig_T1)) >> 12
             * (int32_t) pHandle->dig_T3) >> 14;

    *fin_temp = var1 + var2;
    return (*fin_temp * 5 + 128) >> 8;


}


/*** CALCULATION OF THE PRESSURE, RETURN VALUES IN PASCAL  ***/
uint32_t Calc_Press(BMP280_HandleDef *pHandle, int32_t tempPress, int32_t fin_Press)
{

    int64_t var1, var2, p;

    var1 = (int64_t) fin_Press - 128000;
    var2 = var1 * var1 * (int64_t) pHandle->dig_P6;
    var2= var2 + ((var1 * (int64_t) pHandle->dig_P5) << 17);
    var2 = var2 + (((int64_t)pHandle->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t) pHandle->dig_P3) >> 8) + 
            ((var1 * (int64_t) pHandle->dig_P2) << 12);
    var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) pHandle->dig_P1) >> 33;

    if(var1 == 0)
    {
        return 0;
    }

    p = 1048576 - tempPress;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t) pHandle->dig_P9 *(p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t) pHandle-> dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t) pHandle->dig_P7 << 4);
    
    return p;

}

void Read_Data_Raw(BMP280_HandleDef *pHandle, int32_t *pTemp, int32_t *pPress)
{

    int32_t tempPress;
    int32_t tempTemp;
    uint8_t data[8];

    tempPress = data[0] << 12 | data[1] << 4 | data[2] >> 4;
    tempTemp = data[3] << 12 | data[4] << 4 | data[5] >> 4;

    int32_t fin_temp;
    
    *pTemp = Calc_temp(pHandle, tempTemp, &fin_temp);
    *pPress = Calc_Press(pHandle, tempPress, fin_temp);



}
void Read_Data_Float(BMP280_HandleDef *pHandle, float *pTemp, float *pPress)
{
    int32_t fixTemp;
    uint32_t fixPress;

    if(Read_Data_Raw(pHandle, &fixTemp, &fixPress) ?: NULL)
    {
        *pTemp = (float) fixTemp / 100;
        *pPress = (float) fixPress / 256;
    }
}
