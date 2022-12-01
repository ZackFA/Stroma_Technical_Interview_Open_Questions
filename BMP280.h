#ifndef BMP280_H_
#define BMP280_H_

#include <stdint.h>
#include <stdio.h>


/*** REGISTERS BASE ADDRESS DEFINITIONS***/
#define BMP280_REG_TEMP_XLSB    0xFC  // from bit 7 till bit 0
#define BMP280_REG_TEMP_LSB     0xFB 
#define BMP280_REG_TEMP_MSB     0xFA
#define BMP280_REG_PRESS_XLSB   0xF9  // from bit 7 till bit 0
#define BMP280_REG_PRESS_LSB    0xF8
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP28_REG_CONFIG        0xF5  // t_sb bit 7 to 5, filter bit 4 to 2, bit 1 reserved, spi3w_en bit 0
#define BMP280_REG_CTRL_MEAS    0xF4  // osrs_t bit 7 to 5, osrs_p bit 4 to 2, mode bit 0 to 1
#define BMP280_REG_STATUS       0xF3  // bit 7 to 4 reserved, measuring bit 3, bit 2 to 1 is reserved , im_update bit 0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_ID           0xD0
#define BMP280_REG_CALIB        0x88



#define BMP280_CHIP_ID          0x58
#define BMP280_CHIP_ID          0x60

#define BMP280_I2C_SDO_LOW      0x76    // IF SDO PIN IS PULLED LOW
#define BMP280_I2C_SD0_HIGH     0x77    // IF SDO PIN IS PULLED HIGH

#define BMP280_RESET_VALUE      0xB6  // Value written to the reset register

typedef struct{

    BMP280_FuncMode Mode;
    BMP280_Filter Filter;
    BMP280_OversampleRate Oversampling_Pres;
    BMP280_OversampleRate Oversampling_Temp;
    BMP280_MeasRate Standby;

} BMP280_ConfigTypeDef_t;



/*** SENSOR FUNCTIONIN MODE  ***/
typedef enum {

    Mode_Sleep = 0,
    Mode_Forced = 1,
    Mode_Normal = 2

} BMP280_FuncMode;

/*** IIR FILTER SETTINGS ***/
typedef enum {

    Filter_OFF = 0,
    Filter_COE2 = 1,
    Filter_COE4 = 2,
    Filter_CO8 = 3,
    Filter_CO16 = 4,

} BMP280_Filter;


/*** OVERSAMPLING SETTING FOR PRESSURE   ***/
typedef enum {


    ULTRA_LOWP = 1,
    LOWP = 2,
    STANDARD = 3,
    HIGHRES = 4,
    ULTRA_HIGHRES = 5


} BMP280_OversampleRate;

/*** STANDBY TIME RATE FOR MEASUREMENT RATES  ***/
typedef enum {

    StandbyTime_05 = 0,
    StandbyTime_62 = 1,
    StandbyTime_125 = 2,
    StandbyTime_250 = 3,
    StandbyTime_500 = 4,
    StandbyTime_1000 = 5,
    StandbyTime_2000 = 6,
    StandbyTime_4000 = 7


} BMP280_MeasRate;


typedef struct {


    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;


} BMP280_HandleDef;

/*** API INITIALISATIONS FOR THE SOURCE FILE***/

void BMP280_INIT(BMP280_ConfigTypeDef_t *pConfig);
void Read_Data_Raw(BMP280_HandleDef *pHandle, int32_t *pTemp, uint32_t *pPres);
void BRead_Data_Float(BMP280_HandleDef *pHandle, float *pTemp, float *pPres);







#endif