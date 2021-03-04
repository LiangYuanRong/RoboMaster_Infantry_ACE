#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "stm32f4xx_usart.h"
#include "sys.h" 

#define USART1_RX_LEN 25
#define USART1_TX_LEN 25
void Usart1_Init(void);
extern volatile unsigned char Usart1_Rx[USART1_RX_LEN];	


#define USART2_RX_LEN 25
#define USART2_TX_LEN 25
void Usart2_Init(void);
extern u8 Usart2_Rx[USART2_RX_LEN];
extern u8 Usart2_Tx[USART2_TX_LEN];


#define USART3_RX_LEN 256
#define USART3_TX_LEN 256
void Usart3_Init(void);
extern u8 Usart3_Rx[USART3_RX_LEN];
extern u8 Usart3_Tx[USART3_TX_LEN];


#define USART6_RX_LEN 256
#define USART6_TX_LEN 256
void Usart6_Init(void);
extern u8 Usart6_Rx[USART6_RX_LEN];
extern u8 Usart6_Tx[USART6_TX_LEN];

#endif
