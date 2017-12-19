#include "serial.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"

#include "lcdm2000.h"

volatile u8 TxBuffer[4] = {0x01, 0x2, 0x3, 0x40};
volatile u8 RxBuffer[64] = {0};
DMA_InitTypeDef DMA_InitStructure;

void USART1_Init(void);
static void NVIC_Configuration(void);
void SendDataUSART1(uint8_t *USART1_TxBuffer, uint32_t Length);
void USART1_SendByte(u8 byte);
void Usart1_WaitRx(void);


void taskSerial( void *pvParameters ){

	USART1_Init();
	NVIC_Configuration();
	
	
	Usart1_WaitRx();

	for(;;){
		;
	}
}


void USART1_Init(void){
	USART_InitTypeDef USART_InitStructure;
	
  USART_DeInit(USART1);
	
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);
	
//	USART_ITConfig(USART1, /*USART_IT_TXE, */USART_IT_RXNE, ENABLE);
//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);


	/* DMA1 Channel4 (triggered by USART1 Tx event) Config */
  DMA_DeInit(DMA1_Channel4);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)TxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = sizeof(TxBuffer);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);

  /* DMA1 Channel4 (triggered by USART1 Rx event) Config */
  DMA_DeInit(DMA1_Channel5);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = sizeof(RxBuffer)/*10*/;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	
  /* Enable USART1 DMA TX\RX request */
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	
  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);	

  /* Wait until TXE=1 */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	
//	USART1_SendByte(ACK);
	
}

static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure the priority group */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  /* Enable the DMAChannel1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // Enable the DMAChannel1 Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
}



void USART1_SendByte(u8 byte)
{
	USART_SendData(USART1, byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	;
}
void SendDataUSART1(uint8_t *USART1_TxBuffer, uint32_t Length)
{

  /* Enable DMA1 Channel5 transmit complete interrupt */
//  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
  
//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);             ???????
  /* Enable DMA1 Channel4 */
//	DMA_Cmd(DMA1_Channel4, ENABLE);  

  /* Set high pin DE (PC.06) */
//  GPIOC->BSRR = GPIO_Pin_6;
  
}

void Usart1_WaitRx(void){
	
	USART_ITConfig(USART1, /*USART_IT_TXE, */USART_IT_RXNE, ENABLE);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_HT, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	
}


void USART1_IRQHandler (void){
	//USART_ClearITPendingBit
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)){
		USART_SendData(USART1, ACK);
	}

}








void DMA1_Channel4_IRQHandler(void)
{
  if(DMA1->ISR & (u32)0x00002000)
  {
    /* Enable USART1 Transmit complete interrupt */
//    USART_ITConfig(USART1, USART_IT_TC, ENABLE); 
    
    /* Disable DMA1 Channel4 transmit complete interrupt */
//    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
  
    /* Clear DMA TC pending bit */
    DMA_ClearITPendingBit(DMA1_IT_TC4);
  }
}

void DMA1_Channel5_IRQHandler(void)
{
	/*
  if(DMA1->ISR & (u32)0x00004000)
  {
//    USART_ITConfig(USART1, USART_IT_TC, ENABLE); 
    
//    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
  
    DMA_ClearITPendingBit(DMA1_IT_TC5);
  }
	*/
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_HT5))
	{
		USART_SendData(USART1, ACK);
		DMA_ClearITPendingBit(DMA1_IT_HT5);
	}
}
