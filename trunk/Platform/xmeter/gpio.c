/**
 *
 * \file gpio.c
 * \brief micro GPIO functions.
 *
 *  Micro: General purpose functions. LED control, IRQ allocation.
 *	
 */

#include "stm32f10x.h"
 
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>
#include <signal.h>

#include "bus.h"
#include "debug.h"

#include "gpio.h"
#include "portable.h"

#ifdef HAVE_POWERSAVE
#include "powersave.h"
#endif

 
typedef void (*irq_handler_t)(void);

void exti_default_isr(void){}

volatile irq_handler_t exti_isr[16] =
{
    exti_default_isr,exti_default_isr,exti_default_isr,exti_default_isr,
    exti_default_isr,exti_default_isr,exti_default_isr,exti_default_isr,
    exti_default_isr,exti_default_isr,exti_default_isr,exti_default_isr
};

void __attribute__ ((interrupt)) __cs3_isr_exti0(void)
{
    exti_isr[0]();
}
void __attribute__ ((interrupt)) __cs3_isr_exti1(void)
{
    exti_isr[1]();
}
void __attribute__ ((interrupt)) __cs3_isr_exti2(void)
{
    exti_isr[2]();
}
void __attribute__ ((interrupt)) __cs3_isr_exti3(void)
{
    exti_isr[3]();
}
void __attribute__ ((interrupt)) __cs3_isr_exti4(void)
{
    exti_isr[4]();
}
void __attribute__ ((interrupt)) __cs3_isr_exti9_5(void)
{
    uint8_t i;
    uint32_t bitmask = (0x0001<<5);
    for(i=5;i<=9;i++)
    {
	if(RESET != EXTI_GetFlagStatus(bitmask))
	{
	    exti_isr[i]();
	}
	bitmask <<= 1;
    }
}
void __attribute__ ((interrupt)) __cs3_isr_exti15_10(void)
{
    uint8_t i;
    uint32_t bitmask = (0x0001<<10);
    for(i=10;i<=15;i++)
    {
	if(RESET != EXTI_GetFlagStatus(bitmask))
	{
	    exti_isr[i]();
	}
	bitmask <<= 1;
    }
}
/*
 * Init GPIO port
 * */
void gpio_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  /* Configure PC13 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* Configure PB0 and PB1 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* Configure PA1 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI1 Line to and PB1  pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

  /* Configure EXTI1 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI1 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel =  EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0E;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**
 *  Allocate interrupt in port 1.
 *
 *	\param pin  port pin to allocate
 *	\param isr  interrupt service routine
 *  \param edge 0 = rising edge, anything else falling edge
 *
 *  \return   pdTRUE  allocation succeeded
 *  \return   pdFALSE interrupt reserved or not available on platform
 */
portCHAR gpio1_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge)
{
    exti_isr[1] = (irq_handler_t)isr;
    return pdTRUE;
}
#if 0
/**
 *  Allocate interrupt in port 2.
 *
 *	\param pin  port pin to allocate
 *	\param isr  interrupt service routine
 *  \param edge 0 = rising edge, anything else falling edge
 *
 *  \return   pdTRUE  allocation succeeded
 *  \return   pdFALSE interrupt reserved or not available on platform
 */
portCHAR gpio2_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge)
{
    exti_isr[1] = (irq_handler_t)isr;
    return pdTRUE;
}
#endif /* 0 */

void gpio_irq_enable(uint8_t pin)
{
/* Configure EXTI1 line */
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void gpio_irq_disable(uint8_t pin)
{
/* Configure EXTI1 line */
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
}



void led_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  /* Configure PB.00 and PB.01 pin as output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void led1_off()
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_RESET);
}
void led1_on()
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_SET);
}

void led2_off()
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_RESET);
}
void led2_on()
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_SET);
}
