/*
 * medium_test.c
 *
 *  Created on: Nov 27, 2022
 *      Author: macintosh
 */

/*****************************************************
 *             			                     * 
 * 	This code initialises some                   *
 * 	memory base addresses, API's                 *
 * 	and peripheral configurations                *
 * 	for an objective that will be        	     *
 * 	using UART communication                     *
 * 	for some certain purpose.                    *
 * 			                             *
 *                                                   *
 *****************************************************/


#include "main.h"


/*** MACRO DEFINITIONS FOR MEMORY ADDRESS BASES ***/
#define BOOT_FLAG_ADDRESS           0x08004000U
#define APP_ADDRESS                 0x08008000U
#define TIMEOUT_VALUE               SystemCoreClock/4

#define ACK     0x06U
#define NACK    0x16U

static UART_HandleTypeDef huart;
static uint8_t RX_Buffer[32]; // RECEIVED WORD LENGTH
typedef enum
{
    ERASE = 0x43,
    WRITE = 0x31,
    CHECK = 0x51,
    JUMP  = 0xA1,
} COMMANDS;
/**************************************************/


/*** API INITILIASATION ***/
static void Jump2App(void);
static void Boot_Init(void);
static void Transmit_ACK(UART_HandleTypeDef *huart);
static void Transmit_NACK(UART_HandleTypeDef *huart);
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len);
static void Erase(void);
static void Write(void);
static void Check(void);
/**************************/


int main(void)
{
    Clk_Update(); // Clock configurations
    Boot_Init(); // Initialise the device (Peripherals etc.)

    Transmit_ACK(&huart);
    if(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT) // UART Receive
    {
        Transmit_NACK(&huart);
        Jump2App();
    }
    if(Check_Checksum(RX_Buffer, 2) != 1 || RX_Buffer[0] != ACK) // Check if the received data include corruption or errors
    {
        Transmit_NACK(&huart);
        Jump2App();
    }

    
    /*** FOR LOOP TO DETECT THE DATA RECEIVED AND USE A FUNCTION ACCORDING TO THE DATA ***/
	for(;;)
	{
        while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT); // UART Receive

        if(Check_Checksum(RX_Buffer, 2) != 1) // Check if the received data include corruption or errors
        {
            Transmit_NACK(&huart);
        }
        else
        {
            switch(RX_Buffer[0]) // Detecting which function is received from the buffer
            {
                case ERASE:
                    Transmit_ACK(&huart);
                    Erase();
                    break;
                case WRITE:
                    Transmit_ACK(&huart);
                    Write();
                    break;
                case CHECK:
                    Transmit_ACK(&huart);
                    Check();
                    break;
                case JUMP:
                    Transmit_ACK(&huart);
                    Jump2App();
                    break;
                default:
                    Transmit_NACK(&huart);
                    break;
            }
        }
	}

    for(;;); // Random for loop :D
	return 0;
}

/*** JUMPING TO THE USER APPLICATION ***/ 
static void Jump2App(void)
{
    if (((*(__IO uint32_t*)APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
        __disable_irq();
        uint32_t jump_address = *(__IO uint32_t *)(APP_ADDRESS + 4);
        __set_MSP(*(__IO uint32_t *)APP_ADDRESS);
        void (*pmain_app)(void) = (void (*)(void))(jump_address);
        pmain_app();
    }

}

/*** GENERAL INITILISATION FOR PERIPHERAL AND PERIPHERAL CONFIGURATIONS ***/
static void Boot_Init(void)
{
    GPIO_InitTypeDef gpio_uart;

    
    /*** CONFIGURING GPIO REGISTERS ***/
    gpio_uart.Pin = GPIO_PIN_2 | GPIO_PIN_3;  // USE PIN 2 & 3
    gpio_uart.Mode = GPIO_MODE_AF_PP; // GPIO MODE : ALTERNATING FUNCTION PP
    gpio_uart.Pull = GPIO_PULL_NONE; // NO PULL UP OR PULL DOWN
    gpio_uart.Speed = GPIO_SPEED_LOW; // LOW SPEED
    gpio_uart.Alternate = GPIO_AF7_USART2; 

    HAL_RCC_GPIOA_CLK_ENABLE(); //INITIALISE CLOCK FOR GPIOA
    HAL_GPIO_Init(GPIOA, &gpio_uart); //INITIALISE GPIOA FOR UART

    /*** CONFIGURING UART PROPERTIES ***/
    huart.Init.BaudRate = 115200; // Transmission rate
    huart.Init.Mode = HAL_UART_MODE_TX_RX; // UART Mode
    huart.Init.OverSampling = HAL_UART_OVERSAMPLING_16; // OVERSAMPLING RATE
    huart.Init.Parity = HAL_UART_PARITY_NONE; // PARITY BIT 
    huart.Init.StopBits = HAL_UART_STOP_1; 
    huart.Init.WordLength = HAL_UART_WORD8;
    huart.Instance = USART2;

    HAL_RCC_USART2_CLK_ENABLE(); // CLOCK ENABLE FOR USART2
    HAL_UART_Init(&huart);
}

/*** TRANSMITS ACK BITS ***/
static void Transmit_ACK(UART_HandleTypeDef *handle)
{
    uint8_t msg[2] = {ACK, ACK}; // Reading 2 bits for ACK

    HAL_UART_Tx(handle, msg, 2); // transmits data through UART
}
;
/*** TRANSMITS NACK BITS ***/
static void Transmit_NACK(UART_HandleTypeDef *handle)
{
    uint8_t msg[2] = {NACK, NACK}; // Reading 2 bits for NACK

    HAL_UART_Tx(handle, msg, 2); // transmits data through UART
}


static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len)
{
    uint8_t initial = 0xFF; 
    uint8_t result = 0x7F; 

    result = initial ^ *pBuffer++; 
    len--;
    while(len--)
    {
        result ^= *pBuffer++;
    }

    result ^= 0xFF;

    if(result == 0x00)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*** API FOR ERASING ***/
static void Erase(void)
{
    Flash_EraseInitTypeDef flashEraseConfig;
    uint32_t sectorError;

    while(HAL_UART_Rx(&huart, RX_Buffer, 3, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    if(Check_Checksum(RX_Buffer, 3) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }

    if(RX_Buffer[0] == 0xFF)
    {
        Transmit_NACK(&huart);
    }
    else
    {

        flashEraseConfig.TypeErase = HAL_FLASH_TYPEERASE_SECTOR;
        flashEraseConfig.NbSectors = RX_Buffer[0];
        flashEraseConfig.Sector = RX_Buffer[1];

        HAL_Flash_Unlock();
        HAL_Flash_Erase(&flashEraseConfig, &sectorError);
        HAL_Flash_Lock();

        Transmit_ACK(&huart);
    }
}

/*** API FOR WRITING TO BUFFER ***/
static void Write(void)
{
    uint8_t numBytes;
    uint32_t startingAddress = 0;
    uint8_t i;

    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT); // UART read

    if(Check_Checksum(RX_Buffer, 5) != 1) // Check if the received data include corruption or errors
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }

    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8)
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);

    while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    numBytes = RX_Buffer[0];

    while(HAL_UART_Rx(&huart, RX_Buffer, numBytes+1, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }

    i = 0;
    HAL_Flash_Unlock();
    while(numBytes--)
    {
        HAL_Flash_Program(FLASH_TYPEPROGRAM_BYTE, startingAddress, RX_Buffer[i]);
        startingAddress++;
        i++;
    }
    HAL_Flash_Lock();
    Transmit_ACK(&huart);
}

/*** GENERAL ADDRESS CHECK ***/
static void Check(void)
{
    uint32_t startingAddress = 0;
    uint32_t endingAddress = 0;
    uint32_t address;
    uint32_t *data;
    uint32_t crcResult;

    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    /*** CHECKS IF NO CORRUPT DATA EXISTS ***/
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    
    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8)
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24); // Initialise starting address

    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    /*** CHECKS IF NO CORRUPT DATA EXISTS ***/
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }

    endingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8)
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24); // Initialise end address

    HAL_RCC_CRC_CLK_ENABLE(); // Clock enable
    data = (uint32_t *)((__IO uint32_t*) startingAddress);
    
    /*** Write data into address ***/
    for(address = startingAddress; address < endingAddress; address += 4)
    {
        data = (uint32_t *)((__IO uint32_t*) address);
        crcResult = HAL_CRC_Accumulate(data, 1);
    }

    HAL_RCC_CRC_CLK_DISABLE(); // Clock disable
    if(crcResult == 0x00)
    {
        Transmit_ACK(&huart);
    }
    else
    {
        Transmit_NACK(&huart);
    }

    Jump2App();
}
