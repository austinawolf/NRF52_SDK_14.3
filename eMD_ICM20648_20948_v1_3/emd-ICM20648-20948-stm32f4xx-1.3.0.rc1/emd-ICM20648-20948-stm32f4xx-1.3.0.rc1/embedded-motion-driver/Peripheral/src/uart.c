
#include <stdio.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h" 
#include "misc.h" 
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"

/********************************* Defines ************************************/

//////////////////  USART1

/*#define USARTx                         USART1
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource9
#define USARTx_TX_AF                     GPIO_AF_USART1

#define USARTx_RX_PIN                    GPIO_Pin_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource10
#define USARTx_RX_AF                     GPIO_AF_USART1

#define USARTx_DMAx_CLK                  RCC_AHBPeriph_DMA1*/


//////////////////  USART6

//#define USART6                           USART6
#define USART6_CLK                       RCC_APB2Periph_USART6
#define USART6_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USART6_IRQn                      USART6_IRQn
#define USART6_IRQHandler                USART6_IRQHandler

#define USART6_TX_PIN                    GPIO_Pin_6
#define USART6_TX_GPIO_PORT              GPIOC
#define USART6_TX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USART6_TX_SOURCE                 GPIO_PinSource6
#define USART6_TX_AF                     GPIO_AF_USART6

#define USART6_RX_PIN                    GPIO_Pin_7
#define USART6_RX_GPIO_PORT              GPIOC
#define USART6_RX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USART6_RX_SOURCE                 GPIO_PinSource7
#define USART6_RX_AF                     GPIO_AF_USART6

#define USART6_DMAx_CLK                  RCC_AHBPeriph_DMA6


//////////////////  USART2
#define USARTx                           USART2
#define USARTx_CLK                       RCC_APB1Periph_USART2
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource2
#define USARTx_TX_AF                     GPIO_AF_USART2

#define USARTx_RX_PIN                    GPIO_Pin_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource3
#define USARTx_RX_AF                     GPIO_AF_USART2

#define USARTx_DMAx_CLK                  RCC_AHBPeriph_DMA1

/********************************* Globals ************************************/
/********************************* Prototypes *********************************/
/*******************************  Function ************************************/

void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);
  
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN | USARTx_RX_PIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  //GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  //GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  //GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
 
  USART_InitStructure.USART_BaudRate = 921600; //115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_OverSampling8Cmd(USARTx, ENABLE);
  USART_Init(USARTx, &USART_InitStructure);
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}

void USART_Config6(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USART6_TX_GPIO_CLK | USART6_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USART6_CLK_INIT(USART6_CLK, ENABLE);
  
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USART6_TX_GPIO_PORT, USART6_TX_SOURCE, USART6_TX_AF);
  GPIO_PinAFConfig(USART6_RX_GPIO_PORT, USART6_RX_SOURCE, USART6_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = USART6_TX_PIN | USART6_RX_PIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  //GPIO_InitStructure.GPIO_Pin = USART6_TX_PIN;
  //GPIO_Init(USART6_TX_GPIO_PORT, &GPIO_InitStructure);
  //GPIO_InitStructure.GPIO_Pin = USART6_RX_PIN;
  //GPIO_Init(USART6_RX_GPIO_PORT, &GPIO_InitStructure);
 
  USART_InitStructure.USART_BaudRate = 921600; //9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART6, &USART_InitStructure);
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

  /* Enable USART */
  USART_Cmd(USART6, ENABLE);
}

int fputc(int ch )
{
  //* Place your implementation of fputc here 
  //* e.g. write a character to the USART 
  USART_SendData(USARTx, (uint8_t) ch);

  //* Loop until the end of transmission 
  while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

int fputc6(int ch )
{
  //* Place your implementation of fputc here 
  //* e.g. write a character to the USART 
  USART_SendData(USART6, (uint8_t) ch);

  //* Loop until the end of transmission 
  while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

