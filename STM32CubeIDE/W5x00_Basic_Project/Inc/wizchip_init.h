#ifndef __WIZCHIP_INIT_H__
#define __WIZCHIP_INIT_H__

//#define WIZCHIP_REGISTER_DUMP

#ifdef USE_STDPERIPH_DRIVER
	#warning USE_STDPERIPH_DRIVER

#include "stm32f10x_conf.h"
#include "serialCommand.h"
#include "mcu_init.h"
#define DMA

#elif defined USE_HAL_DRIVER
	#warning USE_HAL_DRIVER
#include "main.h"

/* RESET */
#define WIZnet_RESET_PIN		GPIO_PIN_8
#define WIZnet_RESET_PORT		GPIOD

/* SPI1 */
extern SPI_HandleTypeDef hspi1;
#define WIZnet_SPI  			hspi1
#define WIZnet_CS_PIN			GPIO_PIN_6
#define WIZnet_CS_PORT			GPIOB

#define WIZnet_SPI_SIZE          1
#define WIZnet_SPI_TIMEOUT       10
#else
	#error Error!! STD_DRIVER not defined

#endif

#include "wizchip_conf.h"

#if (_WIZCHIP_IO_MODE_==_WIZCHIP_IO_MODE_BUS_INDIR_)
	#ifdef DMA
   	   #define BUS_DMA
	#endif
#elif(_WIZCHIP_IO_MODE_== _WIZCHIP_IO_MODE_SPI_VDM_)||(_WIZCHIP_IO_MODE_== _WIZCHIP_IO_MODE_SPI_FDM_)
	#ifdef DMA
		#define SPI_DMA
	#endif
#endif

void WIZnet_Initialze(void);
void WIZnet_Reset(void);

uint8_t WIZnet_SpiReadByte(void);
void WIZnet_SpiWriteByte(uint8_t byte);

uint8_t WIZnet_SpiDummyReadBurst(uint8_t* pBuf, uint16_t len);
void WIZnet_SpiDummyWriteBurst(uint8_t* pBuf, uint16_t len);

#if defined SPI_DMA
uint8_t WIZnet_SpiReadBurst(uint8_t* pBuf, uint16_t len);
void WIZnet_SpiWriteBurst(uint8_t* pBuf, uint16_t len);
#endif

void WIZnet_BusWriteByte(uint32_t addr, iodata_t data);
iodata_t WIZnet_BusReadByte(uint32_t addr);

#if defined BUS_DMA
void WIZnet_BusWriteBurst(uint32_t addr, uint8_t* pBuf ,uint32_t len,uint8_t addr_inc);
void WIZnet_BusReadBurst(uint32_t addr,uint8_t* pBuf, uint32_t len,uint8_t addr_inc);
#endif

void WIZnet_Reset(void);
void WIZnet_CsEnable(void);
void WIZnet_CsDisable(void);
void WIZnet_ResetAssert(void);
void WIZnet_ResetDeassert(void);

#ifdef WIZCHIP_REGISTER_DUMP
void WIZnet_Register_read(void);
void WIZnet_SocketRegister_read(uint8_t sn);
#endif

#endif
