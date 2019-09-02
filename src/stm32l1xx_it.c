/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Template/stm32l1xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    May-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_rtc.h"
#include "discover_functions.h"
#include "discover_board.h"
#include "stm32l_discovery_lcd.h"
#include "tsl.h"
#include "stm32l1xx_gpio.h"

//#define LED_GREEN_OFF    {GPIOB->BSRRL = (1<<7);}
//#define LED_GREEN_ON     {GPIOB->BSRRH = (1<<7);}

//#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected */

extern volatile bool KeyPressed;
extern uint8_t state_machine;
extern bool self_test;
extern bool UserButton;
extern volatile bool Idd_WakeUP;
extern uint8_t t_bar[2];
extern bool LedON;
extern bool swipe_ST;
extern uint16_t swipe_delay;
bool KeyON;
uint32_t button_deb = 0;
uint32_t LED_delay = 0;
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1);
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1);
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1);
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1);
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1);
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
 /* Go to infinite loop when Hard Fault exception occurs */
  while (1);
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
  while (1);
}

/**
  * @brief  This function handles SysTick interrupts.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
//    disableGlobalInterrupts();
//    TimingDelay_Decrement();
//    enableGlobalInterrupts();

    if(LedON)
    {
    	LedON = FALSE;
    	LED_delay = 6600;
    	GPIO_HIGH(LD_GPIO_PORT,LD_GREEN_GPIO_PIN);
    }
    else if(LED_delay>0)
    {
    	LED_delay--;
    }
    else GPIO_LOW(LD_GPIO_PORT,LD_GREEN_GPIO_PIN);


    /* key debounce*/

    if(KeyON == TRUE)
    {
    	 if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
    	 {
    		 if(button_deb == 0)
    		 {
    			 KeyON = FALSE;
    			 /* Go to next state of state machine*/
    			 state_machine++;
    			 if (state_machine == MAX_STATE) state_machine=STATE_1;
    		 }
    		 else button_deb--;
    	 }
    	 else
    	 {
    		 KeyON = FALSE;
    		 button_deb = 0;
    	 }
    }
    if(swipe_ST)
    {
    	if(swipe_delay == 0)
    	{
    		swipe_ST= FALSE;
    	}
    	else
    	{
    		swipe_delay--;
    	}
    }
    TSL_tim_ProcessIT();
}

/**
  * @brief  This function handles external interrupts generated by UserButton.
  * @param  None
  * @retval None
  */
void UserButtonHandler (void)
{ 
   
  /* set KeyPressed Flag */
  KeyPressed = TRUE;
  KeyON = TRUE;
  button_deb = 44;   //2200*0.02;


}

void EXTI0_IRQHandler(void)
{
  /* Disable general interrupts */
  disableGlobalInterrupts();
  
  /* UserButton usage activated*/ 

  UserButtonHandler();

  EXTI_ClearITPendingBit(EXTI_Line0);
  enableGlobalInterrupts();
}


void RTC_WKUP_IRQHandler (void)
{
  RTC_ClearITPendingBit(RTC_IT_WUT);
  EXTI_ClearITPendingBit(EXTI_Line20);
}

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_md.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
