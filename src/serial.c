#include "serial.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "cmsis_os.h"
#include "stm32f10x_usart.h"

#include "lcdm2000.h"
#define MAX_REQ_LEN    12

volatile u8 TxBuffer[16] = {0};
volatile u8 RxBuffer[32] = {0};
u8 RxReq[MAX_REQ_LEN] = {0};
DMA_InitTypeDef DMA_InitStructure;

void USART1_Init(void);
static void NVIC_Configuration(void);
//void SendDataUSART1(uint8_t *USART1_TxBuffer, uint32_t Length);

void USART1_SendByte(u8 byte);
void USART1_Send(u8* pData, u8 len);
void USART1_SendDMA(u8*, u8);


void Usart1_WaitRx(void);
void Usart1_WaitACK(void);
u8 CRC_calculate(u8* pdata, u8 len);


#define SERIAL_RX_WAIT					0
#define SERIAL_READ_OK			1
#define SERIAL_RX_START			2
#define SERIAL_RX_PENDING		3
#define SERIAL_RX_RDY				4
#define SERIAL_ACK_WAIT			0x05
#define SERIAL_TX_START			0x12
#define SERIAL_TX_RDY				0x14
#define SERIAL_TX_ACK				0x10
#define SERIAL_RX_ACK_OK		0x11
#define SERIAL_CRC_ERR			0xF0
#define SERIAL_ERROR				0xFF

void SerialRead(void *arg);                   
u8 serialStatus = SERIAL_RX_WAIT;
 
osTimerDef (SerialRxTimer, SerialRead);                      // define timers
osTimerId RxTmr_id;
osStatus  status;
u8  crc_OK; 
void taskSerial( void *pvParameters ){

	DMA_InitTypeDef DMA_Tx_Init;
	u8 i;
	LCDM_CMD cmd;
	
	USART1_Init();
	NVIC_Configuration();
	
	RxTmr_id = osTimerCreate (osTimer(SerialRxTimer), osTimerOnce, &crc_OK);
	if(RxTmr_id != NULL)
	{
		crc_OK = 0;
	}
	Usart1_WaitRx();

	for(;;){
		switch(serialStatus)
		{
			case SERIAL_RX_START:
				status = osTimerStart(RxTmr_id, 100);
				
				if(status == osOK){
					USART_SendData(USART1, ACK);
					serialStatus = SERIAL_RX_PENDING;
				}
				break;

			case SERIAL_RX_PENDING:
				break;
			case SERIAL_RX_RDY:
				osTimerStop(RxTmr_id);
				for(i=0; i<MAX_REQ_LEN; i++){
					if(RxBuffer[i] == EOT)
						continue;
					else{
						RxReq[i-1] = RxBuffer[i];
					}
					if(RxBuffer[i] == ETX){
						serialStatus = SERIAL_READ_OK;
						break;
					}
						
					
				}
				break;
			case SERIAL_READ_OK:
				cmd = RxReq[2];

				switch(cmd){
					case PURGE:
//						serialStatus = SERIAL_TX_START;
					RespPurge[7] = CRC_calculate(RespPurge, sizeof(RespPurge)-1);
			
						USART1_SendDMA(RespPurge, sizeof(RespPurge));
//						serialStatus = SERIAL_RX_WAIT;
						break;
					
					case UPPER_DISPENSE:
						break;
					
					case STATUS:
						break;
					
					case ROM_VERSION:
						break;
					
					case LOWER_DISPENSE:  
						break;
					
					case UPPER_LOWER_DISPENCE:
						break;
					
					case UPPER_TEST_DISPENCE:
						break;
					
					case LOWER_TEST_DISPENCE:
						break;										
					default:
						//error
					
				}
				Usart1_WaitACK();
				//parsing request
				break;
				
			case SERIAL_TX_RDY:
						/* Disable DMA1 Channel4 transmit complete interrupt */
						DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
//						Usart1_WaitACK();
//						
						serialStatus = SERIAL_ACK_WAIT;

				break;

			case SERIAL_RX_ACK_OK:
				Usart1_WaitRx();
				serialStatus = SERIAL_RX_WAIT;
			
				break;
			case SERIAL_ERROR:
			default:
				//EROR code
		}
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

/*
	// DMA1 Channel4 (triggered by USART1 Tx event) Config 
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

  // DMA1 Channel4 (triggered by USART1 Rx event) Config 
  DMA_DeInit(DMA1_Channel5);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = MAX_REQ_LEN;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	
 
	// Enable USART1 DMA TX\RX request 
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
 */
	
  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);	

  /* Wait until TXE=1 */
//  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	
//	USART1_SendByte(ACK);
	
}

static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure the priority group */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  /* Enable the DMA1Channel4 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // Enable the DMA1Channel5 Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
/*	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	*/
	
}



void USART1_SendByte(u8 byte)
{
	USART_SendData(USART1, byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	;
}

void USART1_Send(u8* pData, u8 len)
{
	u8 i;

	
	for(i=0; i<len; i++)
	{
		USART_SendData(USART1, *pData);
		pData++;
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		
	}

}

void USART1_SendDMA(u8 *pdata, u8 len)
{

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)pdata;
	DMA_InitStructure.DMA_BufferSize = len;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	
	/* Enable DMA1 Channel5 transmit complete interrupt */
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	
	DMA_Cmd(DMA1_Channel4, ENABLE);
	
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);	
	
	serialStatus = SERIAL_TX_START;
}

void Usart1_WaitRx(void){
	
//	USART_ITConfig(USART1, /*USART_IT_TXE, */USART_IT_RXNE, ENABLE);
	
	// DMA1 Channel4 (triggered by USART1 Rx event) Config 
  DMA_DeInit(DMA1_Channel5);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = MAX_REQ_LEN;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_HT, ENABLE);
}
void Usart1_WaitACK(void){
	
//	USART_ITConfig(USART1, /*USART_IT_TXE, */USART_IT_RXNE, ENABLE);
	
	// DMA1 Channel4 (triggered by USART1 Rx event) Config 
  DMA_DeInit(DMA1_Channel5);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
}

void USART1_IRQHandler (void){
	//USART_ClearITPendingBit
//	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)){
//		USART_SendData(USART1, ACK);
//	}

}


void DMA1_Channel4_IRQHandler(void)
{
//  if(DMA1->ISR & (u32)0x00002000)
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC4))
  {
  
    /* Clear DMA TC pending bit */
    DMA_ClearITPendingBit(DMA1_IT_TC4|DMA1_IT_GL4);
		serialStatus = SERIAL_TX_RDY;
  }
}

void DMA1_Channel5_IRQHandler(void)
{
	
  if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC5))
  {
//    USART_ITConfig(USART1, USART_IT_TC, ENABLE); 
    
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, DISABLE);
    DMA_ClearITPendingBit(DMA1_IT_TC5|DMA1_IT_HT5|DMA1_IT_GL5);
		serialStatus = SERIAL_RX_ACK_OK;
  }
	
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_HT5))
	{
		serialStatus = SERIAL_RX_START;

		DMA_ClearITPendingBit(DMA1_IT_HT5);
	}
}

void SerialRead(void *arg)
{
	u8 i, crc=0;
	for(i=0; i<sizeof(RxBuffer); i++){
		if(EOT == RxBuffer[i])
			crc = EOT;
		else
			crc ^= RxBuffer[i];
		if( ETX == RxBuffer[i])
			break;
	}
	if(crc == RxBuffer[++i]){
			*(u8*)arg = 1;
		serialStatus = SERIAL_RX_RDY;
		return;
	}
	serialStatus = SERIAL_CRC_ERR;
}
/*
void ResponseCompose(LCDM_RESP responce, u8 argc, u8* argv){
	
}

*/

u8 CRC_calculate(u8* pdata, u8 len){
	u8 i=0, crc=0;
	while(i<len){
		crc ^= *pdata++;
	}
	
	return crc_OK;
}