#include "sx1276-Hal.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

void SX1276InitIo( void ) { }

void SX1276SetReset( uint8_t state )
{
	if(state != RADIO_RESET_ON)
		GPIOA->BSRR = GPIO_PIN_1;
	else
	{
		GPIOA->BSRR = (uint32_t)GPIO_PIN_1 << 16;
		HAL_UART_Transmit(&huart1, (uint8_t*)"SX1276 Reset\r\n", 14, 10);
	}
}

/*uint8_t spiInOut(uint8_t data)
{
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		GPIOA->BSRR = (data & 0x80)? GPIO_PIN_7: (uint32_t)GPIO_PIN_7 << 16;//Data bit
		GPIOA->BSRR = GPIO_PIN_5;//SCK = 1
		data = data << 1 | !!(GPIOA->IDR & GPIO_PIN_6);
		GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16;//SCK = 0
	}
	return data;
}*/

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
//	uint8_t i;
	addr |= 0x80;
	
	GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16; //SS = 0
/*	spiInOut(addr);
	for(i = 0; i < size; i++)
		spiInOut(buffer[i]);*/
	HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
	HAL_SPI_Transmit(&hspi1, buffer, size, 10);
	GPIOA->BSRR = GPIO_PIN_4; //SS = 1
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
//	uint8_t i;
	addr &= 0x7f;
	
	GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16; //SS = 0;
/*	spiInOut(addr);
	for(i = 0; i < size; i++)
		buffer[i] = spiInOut(0);*/
	HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
	HAL_SPI_Receive(&hspi1, buffer, size, 10);
	GPIOA->BSRR = GPIO_PIN_4; //SS = 1
}

void SX1276Write( uint8_t addr, uint8_t data )
{
	SX1276WriteBuffer(addr, &data, 1);
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
	SX1276ReadBuffer(addr, data, 1);
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
	SX1276WriteBuffer(0, buffer, size);
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
	SX1276ReadBuffer(0, buffer, size);
}

uint8_t SX1276ReadDio0( void )
{
	return !!(GPIOA->IDR & GPIO_PIN_0);
}

uint8_t SX1276ReadDio1( void )
{
	return !!(GPIOB->IDR & GPIO_PIN_1);
}

uint8_t SX1276ReadDio2( void )
{
	return !!(GPIOB->IDR & GPIO_PIN_10);
}

uint8_t SX1276ReadDio3( void )
{
	return !!(GPIOA->IDR & GPIO_PIN_8);
}

uint8_t SX1276ReadDio4( void )
{
	return !!(GPIOA->IDR & GPIO_PIN_11);
}

uint8_t SX1276ReadDio5( void )
{
	return !!(GPIOA->IDR & GPIO_PIN_12);
}

void SX1276WriteRxTx( uint8_t txEnable ) { }
