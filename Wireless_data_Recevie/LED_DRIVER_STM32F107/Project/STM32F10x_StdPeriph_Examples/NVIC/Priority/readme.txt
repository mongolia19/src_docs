/**
  @page NVIC_Priority NVIC_Priority
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file    NVIC/Priority/readme.txt 
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Description of the NVIC Priority Example.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
   @endverbatim

@par Example Description 

This example demontrates the use of the Nested Vectored Interrupt Controller (NVIC): 

- Configuration of 2 EXTI Lines (Wakeup button EXTI Line & Key button EXTI Line)
  to generate an interrupt on each falling edge and use the SysTick interrupt.
- These interrupts are configured with the following parameters:
    - Wakeup button EXTI Line:  
        - PreemptionPriority = PreemptionPriorityValue
        - SubPriority = 0
    - Key button EXTI Line:    
        - PreemptionPriority = 0
        - SubPriority = 1           
    - SysTick Handler:  
        - PreemptionPriority = !PreemptionPriorityValue
        - SubPriority = 0             
First, the PreemptionPriorityValue is equal to 0, the Wakeup button EXTI Line 
has higher preemption priority than the SysTick handler. 

In the key button EXTI Line interrupt routine the Wakeup button EXTI Line and 
SysTick preemption priorities are inverted. 
In the Wakeup button EXTI Line interrupt routine the pending bit of the SysTick 
interrupt is set this will cause SysTick ISR to preempt the Wakeup button EXTI 
Line ISR only if it has higher preemption priority.

The system behaves as following:
 
1) The first time Key button EXTI Line interrupt occurs the SysTick preemption 
become higher than Wakeup button EXTI Line one. So when the Wakeup button EXTI 
Line interrupt occurs, the SysTick ISR is executed and the PreemptionOccured 
variable become TRUE and the four leds (LED1, LED2, LED3, LED4) start toggling.

2) When the next Key button EXTI Line interrupt occurs the SysTick preemption
become lower than Wakeup button EXTI Line one. So when the Wakeup button EXTI Line
interrupt occurs, the PreemptionOccured variable became FALSE and the four leds
(LED1, LED2, LED3, LED4) stop toggling.

Then this behavior is repeated from point 1) in an infinite loop.


@par Directory contents 

  - NVIC/Priority/stm32f10x_conf.h     Library Configuration file
  - NVIC/Priority/stm32f10x_it.c       Interrupt handlers
  - NVIC/Priority/stm32f10x_it.h       Interrupt handlers header file
  - NVIC/Priority/main.c               Main program


@par Hardware and Software environment 

  - This example runs on STM32F10x Connectivity line, High-Density, Medium-Density 
    and Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210C-EVAL (STM32F10x 
    Connectivity line), STM3210E-EVAL (STM32F10x High-Density) and STM3210B-EVAL
    (STM32F10x Medium-Density) evaluation boards and can be easily tailored to
    any other supported device and development board.
    To select the STMicroelectronics evaluation board used to run the example, 
    uncomment the corresponding line in stm32_eval.h file

 - STM3210C-EVAL Set-up  
    - Use LED1, LED2, LED3 and LED4 connected respectively to PD.07, PD.13, PF.03
      and PD.04 pins
    - Use the Key push-button connected to pin PB.09 (EXTI Line9).
    - Use the Wakeup push-button connected to pin PA.00 (EXTI Line0). 
      
  - STM3210E-EVAL Set-up 
    - Use LED1, LED2, LED3 and LED4 connected respectively to PF.06, PF0.7, PF.08 
      and PF.09 pins
    - Use the Key push-button connected to pin PG.08 (EXTI Line8).
    - Use the Wakeup push-button connected to pin PA.00 (EXTI Line0).

  - STM3210B-EVAL Set-up  
    - Use LED1, LED2, LED3 and LED4 connected respectively to PC.06, PC.07, PC.08
      and PC.09 pins
    - Use the Key push-button connected to pin PB.09 (EXTI Line9).
    - Use the Wakeup push-button connected to pin PA.00 (EXTI Line0).         
    
@par How to use it ? 

In order to make the program work, you must do the following :
- Create a project and setup all project configuration
- Add the required Library files :
  - stm32f10x_exti.c 
  - stm32f10x_gpio.c 
  - stm32f10x_rcc.c 
  - misc.c 
  - stm32f10x_usart.c
  - system_stm32f10x.c (under Libraries\CMSIS\Core\CM3)
  - stm32_eval.c (under Utilities\STM32_EVAL)
        
- Edit stm32f10x.h file to select the device you are working on.
- Edit stm32_eval.h file to select the evaluation board you will use.
  
@b Tip: You can tailor the provided project template to run this example, for 
        more details please refer to "stm32f10x_stdperiph_lib_um.chm" user 
        manual; select "Peripheral Examples" then follow the instructions 
        provided in "How to proceed" section.   
- Link all compiled files and load your image into target memory
- Run the example

@note
 - Low-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 32 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
    
 * <h3><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h3>
 */
