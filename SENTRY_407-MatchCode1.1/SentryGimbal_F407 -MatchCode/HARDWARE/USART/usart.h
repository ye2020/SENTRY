#ifndef __USART_H
#define __USART_H
#include "main.h"

//volatile
#define USART1_RX_LEN  18
#define USART1_TX_LEN  18
//extern uint8_t Usart1_Rx[USART1_RX_LEN];	
void Usart1_Init(uint8_t *rx1_buf, uint16_t dma_buf_num);

#define USART2_RX_LEN  25
#define USART2_TX_LEN  25
extern u8 Usart2_Rx[USART2_RX_LEN];
extern u8 Usart2_Tx[USART2_TX_LEN];
void Usart2_Init(void);

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


