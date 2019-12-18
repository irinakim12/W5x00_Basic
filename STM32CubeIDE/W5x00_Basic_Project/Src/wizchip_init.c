#include "wizchip_init.h"

#ifdef USE_STDPERIPH_DRIVER
DMA_InitTypeDef		DMA_RX_InitStructure, DMA_TX_InitStructure;
#elif defined USE_HAL_DRIVER
#endif
void WIZnet_Initialze(void)
{
	WIZnet_Reset();

#if _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SPI_
/* SPI method callback registration */
	#if defined SPI_DMA
	reg_wizchip_spi_cbfunc(WIZnet_SpiReadByte, WIZnet_SpiWriteByte, WIZnet_SpiReadBurst, WIZnet_SpiWriteBurst);
	#else
	reg_wizchip_spi_cbfunc(WIZnet_SpiReadByte, WIZnet_SpiWriteByte);
	#endif
	/* CS function register */
	reg_wizchip_cs_cbfunc(WIZnet_CsEnable, WIZnet_CsDisable);
#else
/* Indirect bus method callback registration */
	#if defined BUS_DMA
	reg_wizchip_bus_cbfunc(WIZnet_BusReadByte, WIZnet_BusWriteByte, WIZnet_BusReadBurst, WIZnet_BusWriteBurst);
	#else
	reg_wizchip_bus_cbfunc(WIZnet_BusReadByte, WIZnet_BusWriteByte, 0, 0);
	#endif
#endif

	uint8_t temp;
	unsigned char WIZnet__AdrSet[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};
	do
	{
		if (ctlwizchip(CW_GET_PHYLINK, (void *)&temp) == -1)
		{
			printf("Unknown PHY link status.\r\n");
		}
	} while (temp == PHY_LINK_OFF);
	printf("PHY OK.\r\n");

	temp = IK_DEST_UNREACH;

	if (ctlwizchip(CW_INIT_WIZCHIP, (void *)WIZnet__AdrSet) == -1)
	{
		printf("WIZnet_ initialized fail.\r\n");
	}

	if (ctlwizchip(CW_SET_INTRMASK, &temp) == -1)
	{
		printf("WIZnet_ interrupt\r\n");
	}
	//printf("interrupt mask: %02x\r\n",getIMR());
}

uint8_t WIZnet_SpiReadByte(void)
{
#ifdef USE_STDPERIPH_DRIVER

	while (SPI_I2S_GetFlagStatus(hspi1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(hspi1, 0xff);
	while (SPI_I2S_GetFlagStatus(hspi1, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(hspi1);

#elif defined USE_HAL_DRIVER

	uint8_t rx = 0, tx = 0xFF;
	HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, WIZnet_SPI_SIZE, WIZnet_SPI_TIMEOUT);
	return rx;
#elif defined USE_HAL_DRIVER

	// Sppi read
#endif
}

void WIZnet_SpiWriteByte(uint8_t byte)
{
#ifdef USE_STDPERIPH_DRIVER

	while (SPI_I2S_GetFlagStatus(hspi1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(hspi1, byte);
	while (SPI_I2S_GetFlagStatus(hspi1, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(hspi1);

#elif defined USE_HAL_DRIVER

	uint8_t rx;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &rx, WIZnet_SPI_SIZE, WIZnet_SPI_TIMEOUT);
#endif

}

uint8_t WIZnet_SpiReadBurst(uint8_t* pBuf, uint16_t len)
{
#ifdef USE_STDPERIPH_DRIVER

	unsigned char tempbuf =0xff;
	DMA_TX_InitStructure.DMA_BufferSize = len;
	DMA_TX_InitStructure.DMA_MemoryBaseAddr = &tempbuf;
	//DMA_TX_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(hdma_spi1_tx, &DMA_TX_InitStructure);

	DMA_RX_InitStructure.DMA_BufferSize = len;
	DMA_RX_InitStructure.DMA_MemoryBaseAddr = pBuf;
	DMA_Init(hdma_spi1_rx, &DMA_RX_InitStructure);
	/* Enable SPI Rx/Tx DMA Request*/
	DMA_Cmd(hdma_spi1_rx, ENABLE);
	DMA_Cmd(hdma_spi1_tx, ENABLE);
	/* Waiting for the end of Data Transfer */
	while(DMA_GetFlagStatus(DMA_TX_FLAG) == RESET);
	while(DMA_GetFlagStatus(DMA_RX_FLAG) == RESET);

	DMA_ClearFlag(DMA_TX_FLAG | DMA_RX_FLAG);

	DMA_Cmd(hdma_spi1_tx, DISABLE);
	DMA_Cmd(hdma_spi1_rx, DISABLE);

#elif defined USE_HAL_DRIVER

#endif

}

void WIZnet_SpiWriteBurst(uint8_t* pBuf, uint16_t len)
{
#ifdef USE_STDPERIPH_DRIVER

	unsigned char tempbuf;
	DMA_TX_InitStructure.DMA_BufferSize = len;
	DMA_TX_InitStructure.DMA_MemoryBaseAddr = pBuf;
	DMA_Init(hdma_spi1_tx, &DMA_TX_InitStructure);

	DMA_RX_InitStructure.DMA_BufferSize = 1;
	DMA_RX_InitStructure.DMA_MemoryBaseAddr = &tempbuf;
	DMA_RX_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(hdma_spi1_rx, &DMA_RX_InitStructure);

	DMA_Cmd(hdma_spi1_rx, ENABLE);
	DMA_Cmd(hdma_spi1_tx, ENABLE);

	/* Enable SPI Rx/Tx DMA Request*/

	/* Waiting for the end of Data Transfer */
	while(DMA_GetFlagStatus(DMA_TX_FLAG) == RESET);
	while(DMA_GetFlagStatus(DMA_RX_FLAG) == RESET);

	DMA_ClearFlag(DMA_TX_FLAG | DMA_RX_FLAG);

	DMA_Cmd(hdma_spi1_tx, DISABLE);
	DMA_Cmd(hdma_spi1_rx, DISABLE);

#elif defined USE_HAL_DRIVER

#endif

}

void WIZnet_BusWriteByte(uint32_t addr, iodata_t data)
{
	(*(volatile uint8_t*)(addr)) = data;
}

iodata_t WIZnet_BusReadByte(uint32_t addr)
{
	return (*((volatile uint8_t*)(addr)));
}

void WIZnet_BusWriteBurst(uint32_t addr, uint8_t* pBuf ,uint32_t len,uint8_t addr_inc)
{
#ifdef USE_STDPERIPH_DRIVER

	if(addr_inc){
	 	DMA_TX_InitStructure.DMA_MemoryInc  = DMA_MemoryInc_Enable;

	}
	else 	DMA_TX_InitStructure.DMA_MemoryInc  = DMA_MemoryInc_Disable;


	DMA_TX_InitStructure.DMA_BufferSize = len;
	DMA_TX_InitStructure.DMA_MemoryBaseAddr = addr;
	DMA_TX_InitStructure.DMA_PeripheralBaseAddr = pBuf;

	DMA_Init(hdma_spi1_tx, &DMA_TX_InitStructure);

	DMA_Cmd(hdma_spi1_tx, ENABLE);

	/* Enable SPI Rx/Tx DMA Request*/


	/* Waiting for the end of Data Transfer */
	while(DMA_GetFlagStatus(DMA_TX_FLAG) == RESET);


	DMA_ClearFlag(DMA_TX_FLAG);

	DMA_Cmd(DMA_CHANNEL_TX, DISABLE);

#elif defined USE_HAL_DRIVER

#endif

}

void WIZnet_BusReadBurst(uint32_t addr,uint8_t* pBuf, uint32_t len,uint8_t addr_inc)
{
#ifdef USE_STDPERIPH_DRIVER

	DMA_RX_InitStructure.DMA_BufferSize = len;
	DMA_RX_InitStructure.DMA_MemoryBaseAddr =pBuf;
	DMA_RX_InitStructure.DMA_PeripheralBaseAddr =addr;

	DMA_Init(hdma_spi1_rx, &DMA_RX_InitStructure);

	DMA_Cmd(hdma_spi1_rx, ENABLE);
	/* Waiting for the end of Data Transfer */
	while(DMA_GetFlagStatus(DMA_RX_FLAG) == RESET);


	DMA_ClearFlag(DMA_RX_FLAG);


	DMA_Cmd(hdma_spi1_rx, DISABLE);

#elif defined USE_HAL_DRIVER

#endif

}

inline void WIZnet_CsEnable(void)
{
#ifdef USE_STDPERIPH_DRIVER

	GPIO_ResetBits(SP1_NSS_GPIO_Port, SP1_NSS_Pin);

#elif defined USE_HAL_DRIVER

	HAL_GPIO_WritePin(SP1_NSS_GPIO_Port, SP1_NSS_Pin, GPIO_PIN_RESET);
#endif

}

inline void WIZnet_CsDisable(void)
{
#ifdef USE_STDPERIPH_DRIVER

	GPIO_SetBits(SP1_NSS_GPIO_Port, SP1_NSS_Pin);

#elif defined USE_HAL_DRIVER

	HAL_GPIO_WritePin(SP1_NSS_GPIO_Port, SP1_NSS_Pin, GPIO_PIN_SET);
#endif

}

inline void WIZnet_ResetAssert(void)
{
#ifdef USE_STDPERIPH_DRIVER

	GPIO_ResetBits(WIZnet__RESET_PORT, WIZnet__RESET_PIN);

#elif defined USE_HAL_DRIVER

	HAL_GPIO_WritePin(WIZnet_RESET_PORT, WIZnet_RESET_PIN, GPIO_PIN_RESET);
#endif

}

inline void WIZnet_ResetDeassert(void)
{
#ifdef USE_STDPERIPH_DRIVER

	GPIO_SetBits(WIZnet_RESET_PORT, WIZnet_RESET_PIN);

#elif defined USE_HAL_DRIVER

	HAL_GPIO_WritePin(WIZnet_RESET_PORT, WIZnet_RESET_PIN, GPIO_PIN_SET);
#endif

}

void WIZnet_Reset(void)
{
#ifdef USE_STDPERIPH_DRIVER
	WIZnet_ResetAssert();
	delay(10);
	WIZnet_ResetDeassert();
	delay(10);
#elif defined USE_HAL_DRIVER
	WIZnet_ResetAssert();
	HAL_Delay(10);
	WIZnet_ResetDeassert();
	HAL_Delay(10);
#endif
}

#ifdef WIZCHIP_REGISTER_DUMP

void WIZnet_Register_read(void)
{
	int i;
	printf("                    ----register read----\r\n");
	printf("Address | ");
	for (i = 0; i < 16; i++)
		printf("%02x ", i);
	printf("\r\n---------------------------------------------------------");
	for (i = 0; i < 0x0090; i++)
	{
		if (i % 16 == 0)
			printf("\r\n  %04x  | ", i);
		printf("%02x ", WIZCHIP_READ(_WIZCHIP_IO_BASE_ + (i << 8) + (WIZCHIP_CREG_BLOCK << 3)));
	}
	printf("\r\n");
}

void WIZnet_SocketRegister_read(uint8_t sn)
{
	int i;
	printf("                    ----Sn register read----\r\n");
	printf("Address | ");
	for (i = 0; i < 16; i++)
		printf("%02x ", i);
	printf("\r\n---------------------------------------------------------");
	for (i = 0x400 + (sn * (0x100)); i < 0x400 + (sn * (0x100) + 0x35); i++)
	{
		if (i % 16 == 0)
			printf("\r\n0x%04x  | ", i);
		printf("%02x ", WIZCHIP_READ(i));
	}
	printf("\r\n");
}

#endif
