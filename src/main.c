//main.c
#include "main.h"
#include "serial.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"

#define HCLK_FREQ_72MHz


GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;
    
/* Private function prototypes -----------------------------------------------*/
static void RCC_Configuration(void);
static void NVIC_Configuration(void);


void GPIOInit(void);
int Init_Thread (void);

osThreadId tidThreadUsart;                                          // thread id
osThreadDef (taskSerial, osPriorityNormal, 1, 0);                   // thread object

int Init_Thread (void) {

  tidThreadUsart = osThreadCreate (osThread(taskSerial), NULL);
  if(!tidThreadUsart) return(-1);
	

  
  return(0);
}

// RTX User Timers
//static osTimerId TmrBlink;                                     // timer id

void Init_Timers (void) 
{
//  osStatus  status;                                        // function return status
/*
  TmrBlink = osTimerCreate (osTimer(TmrBlink), osTimerPeriodic, NULL);
  if (TmrBlink != NULL)  {     // One-shoot timer created
    // start timer with delay 500ms
		status = osTimerStart (TmrBlink, 1000);            
    if (status != osOK)  {
    }
	}
	*/
}


U16 CurrTemp = 0xFFFF;
int main(void)
{
	RCC_Configuration();
	NVIC_Configuration();
	GPIOInit();
	

//	printf("TmrBlink create problem");
	
	if (!osKernelRunning ())  {                    // if kernel is not running, initialize the kernel
    if (osKernelInitialize () != osOK)  {        // check osStatus for other possible valid values
      // exit with an error message
    }
  }
	Init_Thread();
//	Init_Timers();


	if (!osKernelRunning ())  {                    // is the kernel running ?
    if (osKernelStart () != osOK)  {             // start the kernel
                                                 // kernel could not be started
    }
  }	
		while(1);
}

void GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

   /* Configure USART1 Tx (PA.09) as alternate function push-pull */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Output SYSCLK clock on MCO pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
   /* Configure GPIOC.06 (DE pin) as output push-pull */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Output the system clock signal on MCO pin (PA.08)*/ 
   RCC_MCOConfig(RCC_MCO_SYSCLK);
}
void Wait(U32 timeout)
{
	U8 i=0;
	while(timeout--)
	{
		for(i=0; i<3; i++);
	
	}
}


static void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
 
#if defined HCLK_FREQ_72MHz  /* 72MHz */

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 	
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div1);
    

#ifdef STM32F10X_CL
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {}

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
#else
    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
#endif


#else /* 24MHz */
    
   /* Flash 0 wait state */
   FLASH_SetLatency(FLASH_Latency_0);

   /* HCLK = SYSCLK */
   RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
   /* PCLK2 = HCLK */
   RCC_PCLK2Config(RCC_HCLK_Div1); 

   /* PCLK1 = HCLK/2 */
   RCC_PCLK1Config(RCC_HCLK_Div1);
   
   
#ifdef STM32F10X_CL
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {}

    /* PLL configuration: PLLCLK = (PLL2 / 10) * 6 = 24 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div10);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_6);
#else
    /* PLLCLK = 8MHz * 3 = 24 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_3);
#endif
 
#endif

   /* Enable PLL */ 
   RCC_PLLCmd(ENABLE);

   /* Wait till PLL is ready */
   while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
   {
   }

   /* Select PLL as system clock source */
   RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

   /* Wait till PLL is used as system clock source */
   while(RCC_GetSYSCLKSource() != 0x08)
   {
   }

   /* Enable USART1 and GPIOx clocks */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOC |\
                      RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
   
   /* Enable DMA1 clock */
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock configuration.
       User can add here some code to deal with this error */    

    /* Go to infinite loop */
    while (1)
    {
    }
  }
}

static void NVIC_Configuration(void)
{
	/*
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // Configure the priority group 
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  // Enable the DMAChannel1 Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // Enable the DMAChannel1 Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	*/
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	
	printf("Wrong parameters value: file %s on line %d\r\n", file, line)
  
	/* Infinite loop */
  while (1)
  {
  }
}
#endif
