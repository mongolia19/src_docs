/**
  ******************************************************************************
  * @file    USART/HalfDuplex/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "platform_config.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_can.h"
#include "stm32f10x_tim.h"
#include "misc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t capture_number1=0,capture_number = 0;
volatile uint16_t aa=0,up,down,i,j,tim3_count1,tim3_count2;
extern uint8_t databuffer[8];
extern uint8_t sendbufferall[111];
extern uint8_t sendbuffer[8];
extern uint8_t sendbuffer0[31];
extern uint8_t sendbuffer1[31];
extern uint8_t sendbuffer2[31];
extern uint8_t sendbuffer3[31];
extern uint8_t sendbuffer4[31];
extern uint8_t sendbuffer5[31];
extern uint8_t sendbuffer6[31];
extern uint8_t sendbuffer7[31];
extern uint8_t ref,numofbyte,cnt,t;

#define TRUE 1
#define FALSE 0


extern uint8_t rec_f;
extern uint8_t RxCounter1;
extern uint8_t RxBuffer1[13];
extern uint8_t TxBuffer1[6];


extern int send_wireless_flag;
extern int distance_vaild_flag;
extern float x_bias;//the offset of laser beam from the center of the screen x direction
extern float y_bias;
extern float x0,y0;
extern int x_dir;
extern int y_dir;
extern uint8_t LocationSendFlag;
extern uint8_t shouldturnflag;
extern int hitpos;
/* Private function prototypes -----------------------------------------------*/
extern void Delay(__IO uint32_t nCount);
extern void UART_OUT(uint16_t *Data,uint16_t Len);
/* Private functions ---------------------------------------------------------*/
 CanRxMsg RxMessage;
 CanTxMsg TxMessage; 
 TIM_ICInitTypeDef  TIM_ICInitStructure1;
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
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}
/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)	   //�ж϶��Ĵ����Ƿ�ǿ�
  
 { 
   
  // unsigned int i;
    
     USART_SendData(USART1,USART_ReceiveData(USART1));
 }
  //���-������������Ҫ�ȶ� SR,�ٶ� DR �Ĵ����������������жϵ�����[ţ��˵Ҫ����]  
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET) 
  {   
    USART_ClearFlag(USART1,USART_FLAG_ORE); //�� SR ��ʵ���������־ 
    USART_ReceiveData(USART1); //�� DR  
  }
}


void USART2_IRQHandler(void)
{
if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)	
  {  Delay(0xffff);	
    /* Read one byte from the receive data register */
    aa= USART_ReceiveData(USART2);   //�����Ĵ��������ݻ��浽���ջ�������
  }
}


void CAN1_RX0_IRQHandler(void)

{      
     CanRxMsg RxMessage;	 
	//u8 dw=0,wd=0;
	//TIM_Configuration();
     Delay(100);
     RxMessage.StdId=0x7FF;
     RxMessage.ExtId=0x1FFFFFFF;
        CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
     
    
         if(RxMessage.ExtId == 0x00111217)
         {
           y_dir=0;
             if(RxMessage.Data[2]==0x01&&RxMessage.Data[6]==0x14)
             {
               
               y_dir=(y_dir|(1<<7));
               x_bias =(RxMessage.Data[0]>>6);
               if(RxMessage.Data[1]==0x01)////x bais
               {
                  y_dir=(y_dir|(1<<6));;//
               }
               else 
               {
               ;
               }
                if(RxMessage.Data[5]==0x01) //y vertical bias direction
                {
                 y_dir=(y_dir|(1<<5));//position valid vertical too high
                }
                else
                {
                 
                  
                }
               y_bias =(RxMessage.Data[4]>>5);
               
               y_dir=y_dir|((int)x_bias<<3);
               y_dir=y_dir|((int)y_bias<<0);
               
                distance_vaild_flag=TRUE;         
             }
             else if(RxMessage.Data[2]==0x00&&RxMessage.Data[6]==0x14)
               {
                 
                 if(RxMessage.Data[3]==0x01)////lost  on the left should turn right
                 {
                     y_dir=(y_dir|(1<<6));
                 }
                 if(RxMessage.Data[7]==0x01)////lost  on the right should turn right
                 {
                        y_dir=(y_dir|(1<<5));
                 }
                 distance_vaild_flag=FALSE;     
               }
               
             
           
         // y_bias=RxMessage.Data[4];
          send_wireless_flag=TRUE;
           CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
         }
}

void USART3_IRQHandler (void)      //�ô��ڲɳ�ʼ��������
{
   if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET)	   //�ж϶��Ĵ����Ƿ�ǿ�
  
 { 
   
   unsigned int i;
   
    RxBuffer1[RxCounter1++] = USART_ReceiveData(USART3);

     if(RxCounter1==13) 
      { 
        rec_f=1;	
        RxCounter1=0;
	//UART3_ITConfig(UART3_IT_RXNE, DISABLE);
        for(i=0; i<13; i++) 
        {
          if(RxBuffer1[i]==0x55)
          {
           TxBuffer1[0]= RxBuffer1[i];
           TxBuffer1[1]= RxBuffer1[i+1]; 
           TxBuffer1[2]= RxBuffer1[i+2];
           TxBuffer1[3]= RxBuffer1[i+3];
           TxBuffer1[4]= RxBuffer1[i+4];
           TxBuffer1[5]= RxBuffer1[i+5];
           break;
          }
        }
     
      }
 }
  
}

void TIM2_IRQHandler(void)
{   
//  if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) 
//  {
//     TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//     if(LocationSendFlag==1)
//     {
//       TxMessage.ExtId=0x000000c0;
//        TxMessage.RTR=CAN_RTR_DATA;
//        TxMessage.IDE=CAN_ID_EXT;
//        TxMessage.DLC=8;
//        TxMessage.Data[0]=0x00;
//        TxMessage.Data[1]=0x00;
//        TxMessage.Data[2]=0x00;
//        TxMessage.Data[3]=0x00;
//        TxMessage.Data[4]=0x00;
//        TxMessage.Data[5]=0x00;
//        TxMessage.Data[6]=0x00;
//        TxMessage.Data[7]=0x00;
//        CAN_Transmit(CAN1, &TxMessage);
//        LocationSendFlag=0;
//          if(hitpos==2|hitpos==3)//inside two hitted
//          {
//            TxMessage.ExtId=0x00111213;
//            TxMessage.RTR=CAN_RTR_DATA;
//            TxMessage.IDE=CAN_ID_EXT;
//            TxMessage.DLC=8;
//            TxMessage.Data[0]=0x00;
//            TxMessage.Data[1]=0x07;
//            TxMessage.Data[2]=0x88;
//            TxMessage.Data[3]=shouldturnflag;
//            TxMessage.Data[4]=0x00;
//            TxMessage.Data[5]=0x00;
//            TxMessage.Data[6]=0x00;
//            TxMessage.Data[7]=0x00;
//            CAN_Transmit(CAN1, &TxMessage);
//          }
//          else//outside two hitted
//          {
//            TxMessage.ExtId=0x00111213;
//            TxMessage.RTR=CAN_RTR_DATA;
//            TxMessage.IDE=CAN_ID_EXT;
//            TxMessage.DLC=8;
//            TxMessage.Data[0]=0x00;
//            TxMessage.Data[1]=0x30;
//            TxMessage.Data[2]=0x88;
//            TxMessage.Data[3]=shouldturnflag;
//            TxMessage.Data[4]=0x00;
//            TxMessage.Data[5]=0x00;
//            TxMessage.Data[6]=0x00;
//            TxMessage.Data[7]=0x00;
//            CAN_Transmit(CAN1, &TxMessage);
//          }
//         
//     }else
//     {
//        TxMessage.ExtId=0x00111213;
//            TxMessage.RTR=CAN_RTR_DATA;
//            TxMessage.IDE=CAN_ID_EXT;
//            TxMessage.DLC=8;
//            TxMessage.Data[0]=0x00;
//            TxMessage.Data[1]=0xa0;
//            TxMessage.Data[2]=0x88;
//            TxMessage.Data[3]=shouldturnflag;
//            TxMessage.Data[4]=0x00;
//            TxMessage.Data[5]=0x00;
//            TxMessage.Data[6]=0x00;
//            TxMessage.Data[7]=0x00;
//            CAN_Transmit(CAN1, &TxMessage);
//     }
//  }
}

void TIM3_IRQHandler(void)
{      
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) 
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  }
  //capture_number++;
  //TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE);
  //TIM_ITConfig(TIM3, TIM_IT_Update,DISABLE);


} 

void TIM4_IRQHandler(void)
{   

//   CanRxMsg RxMessage;	 
//	//u8 dw=0,wd=0;
//	//TIM_Configuration();
//        CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
//     
//    
//         if(RxMessage.ExtId == 0x00111217)
//         {
//           if(RxMessage.Data[2]==0x00)
//           {
//             distance_vaild_flag=FALSE;
//           }
//           else
//           {
//             if(RxMessage.Data[2]==0x01)
//             {
//             distance_vaild_flag=TRUE;         
//             }
//           }
//          y_bias=RxMessage.Data[4];
//          send_wireless_flag=TRUE;
//           //CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
//         }
  
   if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) 
  {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
//        TxMessage.ExtId=0x111214;
//        TxMessage.RTR=CAN_RTR_DATA;
//        TxMessage.IDE=CAN_ID_EXT;
//        TxMessage.DLC=8;
//        CAN_Transmit(CAN1, &TxMessage); //�����ǽ�����ֵ��1000������DSP ͨ��CAN ���������18.555 ����DSP ��18555
  }
} 
void CAN2_RX0_IRQHandler(void)
{      
        CanRxMsg RxMessage;	 
        CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);
    
}


void UART4_IRQHandler (void)      //����1 �жϷ������
{
  //unsigned int i;
  Delay(600);
  if(USART_GetFlagStatus(USARTy, USART_FLAG_RXNE) != RESET)	   //�ж϶��Ĵ����Ƿ�ǿ�
  { Delay(600);
    aa = USART_ReceiveData(USARTy);   //�����Ĵ��������ݻ��浽���ջ�������
  }

}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
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

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

