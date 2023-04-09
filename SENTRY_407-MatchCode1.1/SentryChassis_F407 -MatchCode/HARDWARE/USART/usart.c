#include "usart.h"
#include "fifo_buff.h"



//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 0
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART3->SR & 0X40) == 0); //循环发送,直到发送完毕

    USART3->DR = (u8) ch;
    return ch;
}
#endif



/************************************************************串口一初始化**************************************************************/
void Usart1_Init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{
    #if 1  //通用板
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; //结构体声明

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //时钟使能

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure); //复用io口配置

    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 100000; //遥控接收波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_Mode = USART_Mode_Rx; //接收
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);                     //使能USART1
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); //使能串口DMA接收

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); //中断优先级

    DMA_Cmd(DMA2_Stream5, DISABLE); //关闭DMA

    while (DMA2_Stream5->CR & DMA_SxCR_EN)
        ; //等待 DMA 可配置

    DMA_DeInit(DMA2_Stream5); //重置为缺省值
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);   //源地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;              //目的地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //数据传输方向为外设到内存
    DMA_InitStructure.DMA_BufferSize = dma_buf_num;                         //设置数据的缓冲大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         //工作在循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE); //dma传输完成产生中断

    DMA_Cmd(DMA2_Stream5, ENABLE); //开启DMA
    #else  //官方板子（旧步兵）
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* config USART1 clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    /*遥控Pin口 复用PB7*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

//	#ifdef USART1_PA10_MODE
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA| RCC_AHB1Periph_DMA2 , ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA,&GPIO_InitStructure);
//	#endif

    /* USART1 mode config */
    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 100000;	 // 波特率						 											//SBUS 100K baudrate
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  // 停止位
    USART_InitStructure.USART_Parity = USART_Parity_Even;  // 校验位
    USART_InitStructure.USART_Mode = USART_Mode_Rx;      // USART 模式   接收模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 硬件流设置  无硬件流
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    DMA_Cmd(DMA2_Stream5, DISABLE);

    while (DMA2_Stream5->CR & DMA_SxCR_EN);

    DMA_DeInit(DMA2_Stream5);												 														//重置为缺省值
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);      //源地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;                 //目的地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                    //数据传输方向为外设到内存
    DMA_InitStructure.DMA_BufferSize = dma_buf_num;                            //设置数据的缓冲大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;           //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    //内存缓冲区地址自加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;    //数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;            //数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                            //工作在循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                    //最高优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA2_Stream5, ENABLE);
    #endif
}



/************************************************************串口二初始化**************************************************************/
u8 Usart2_Rx[USART2_RX_LEN] = {0};
u8 Usart2_Tx[USART2_TX_LEN] = {0};
void Usart2_Init(void)
{
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);    //使能USART3时钟
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //GPIOD8复用为USART3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); //GPIOD9复用为USART3

    /* -------------- Configure GPIO ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        NVIC_InitTypeDef NVIC_InitStructure;
        USART_InitTypeDef USART2_InitStruct;

        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOD, &GPIO_InitStruct);

        USART_DeInit(USART2);
        USART2_InitStruct.USART_BaudRate = 115200;
        USART2_InitStruct.USART_WordLength = USART_WordLength_8b;
        USART2_InitStruct.USART_StopBits = USART_StopBits_1;
        USART2_InitStruct.USART_Parity = USART_Parity_No; //无奇偶校验
        USART2_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART2_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART2, &USART2_InitStruct);

        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //DMA1_Stream1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_Init(&NVIC_InitStructure);
        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
        USART_Cmd(USART2, ENABLE);
    }

    /* -------------- Configure DMA -----------------------------------------*/
    {
        DMA_InitTypeDef DMA_InitStruct;

        //发送数据
        DMA_DeInit(DMA1_Stream6); //重置为缺省值
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);   //源地址
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart2_Tx);          //目的地址
        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //数据传输方向为外设到内存
        DMA_InitStruct.DMA_BufferSize = USART2_TX_LEN;                       //设置数据的缓冲大小
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //工作在循环缓存模式
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal; //DMA_MemoryBurst_Single;//
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream6, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream6, DISABLE);
        USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); //使能串口2的DMA发送

        // 接收数据
        DMA_DeInit(DMA1_Stream5); //重置为缺省值
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);   //源地址
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart2_Rx);          //目的地址
        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //数据传输方向为外设到内存
        DMA_InitStruct.DMA_BufferSize = USART2_RX_LEN;                       //设置数据的缓冲大小
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //工作在循环缓存模式
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal; //DMA_MemoryBurst_Single;//
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream5, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream5, ENABLE);
        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    }
}

/************************************************************串口三初始化**************************************************************/
u8 Usart3_Rx[USART3_RX_LEN] = {0};
u8 Usart3_Tx[USART3_TX_LEN] = {0};
void Usart3_Init(void)
{
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);    //使能USART3时钟
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3); //GPIOD8复用为USART3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); //GPIOD9复用为USART3
    /* -------------- Configure GPIO ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        NVIC_InitTypeDef NVIC_InitStructure;
        USART_InitTypeDef USART3_InitStruct;

        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOD, &GPIO_InitStruct);

        USART_DeInit(USART3);
        USART3_InitStruct.USART_BaudRate = 115200;
        USART3_InitStruct.USART_WordLength = USART_WordLength_8b;
        USART3_InitStruct.USART_StopBits = USART_StopBits_1;
        USART3_InitStruct.USART_Parity = USART_Parity_No; //无奇偶校验
        USART3_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART3_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART3, &USART3_InitStruct);

        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //DMA1_Stream1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_Init(&NVIC_InitStructure);
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
        USART_Cmd(USART3, ENABLE);
    }
    /* -------------- Configure DMA -----------------------------------------*/
    {
        //发送数据
        DMA_InitTypeDef DMA_InitStruct;
        DMA_DeInit(DMA1_Stream3); //重置为缺省值
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);   //源地址
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart3_Tx);          //目的地址
        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //数据传输方向为外设到内存
        DMA_InitStruct.DMA_BufferSize = USART3_TX_LEN;                       //设置数据的缓冲大小
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //工作在循环缓存模式
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal; //DMA_MemoryBurst_Single;//
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream3, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream3, DISABLE);
        USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE); //使能串口3的DMA发送

        // 接收数据
        DMA_DeInit(DMA1_Stream1); //重置为缺省值
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);   //源地址
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart3_Rx);          //目的地址
        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //数据传输方向为外设到内存
        DMA_InitStruct.DMA_BufferSize = USART3_RX_LEN;                       //设置数据的缓冲大小
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //工作在循环缓存模式
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal; //DMA_MemoryBurst_Single;//
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream1, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream1, ENABLE);
        USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    }
}

/*****************************************************************串口六初始化**************************************************************/
#define USART6_RX_LEN 512
#define USART6_TX_LEN 256
uint8_t Usart6_Rx[USART6_RX_LEN] = {0};
uint8_t Usart6_Tx[USART6_TX_LEN] = {0};
uint8_t Usart6_Tx_Buffer[USART6_TX_LEN] = {0};

fifo_rx_def fifo_usart_rx_6;
fifo_rx_def *pfifo_6 = &fifo_usart_rx_6;
fifo_rx_def fifo_usart_tx_6;

uint8_t DMA2_Stream6_working = 0;

uint8_t (*usart6_callback)(uint8_t *, uint16_t);

void usart6_init(uint8_t usart_NVIC)
{
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        NVIC_InitTypeDef NVIC_InitStructure;
        USART_InitTypeDef USART6_InitStruct;

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOC, &GPIO_InitStruct);

        USART_DeInit(USART6);
        USART6_InitStruct.USART_BaudRate = 115200;
        USART6_InitStruct.USART_WordLength = USART_WordLength_8b;
        USART6_InitStruct.USART_StopBits = USART_StopBits_1;
        USART6_InitStruct.USART_Parity = USART_Parity_No; //无奇偶校验
        USART6_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART6, &USART6_InitStruct);

        NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = usart_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_Init(&NVIC_InitStructure);

        USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); //开启相关中断
        USART_ITConfig(USART6, USART_IT_TC, DISABLE);
        USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
        USART_Cmd(USART6, ENABLE);
    }
    /* DMA发送 */
    {
        DMA_InitTypeDef DMA_InitStruct;
        NVIC_InitTypeDef NVIC_InitStructure;

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

        //发送数据
        DMA_DeInit(DMA2_Stream6); //重置为缺省值
        DMA_StructInit(&DMA_InitStruct);
        DMA_InitStruct.DMA_Channel = DMA_Channel_5;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);   //源地址
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart6_Tx);          //目的地址
        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //数据传输方向为外设到内存
        DMA_InitStruct.DMA_BufferSize = USART6_TX_LEN;                       //设置数据的缓冲大小
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;                           //正常工作模式
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream6, &DMA_InitStruct);

        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);  //开启发送完成中断
        DMA_Cmd(DMA2_Stream6, DISABLE);
        USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE); //使能串口2的DMA发送

        if (fifo_init(&fifo_usart_tx_6, Usart6_Tx_Buffer, USART6_TX_LEN) == -1)
        {
            // 必须 2 的幂次方
        }
    }
    /* DMA接收 */
    {
        DMA_InitTypeDef DMA_InitStruct;

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

        //接收数据
        DMA_DeInit(DMA2_Stream1); //重置为缺省值
        DMA_StructInit(&DMA_InitStruct);
        DMA_InitStruct.DMA_Channel = DMA_Channel_5;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);   //源地址
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart6_Rx);          //目的地址
        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //数据传输方向为外设到内存
        DMA_InitStruct.DMA_BufferSize = USART6_RX_LEN;                       //设置数据的缓冲大小
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //工作在循环缓存模式
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
//        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
//        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//        DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal;
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream1, &DMA_InitStruct);

        DMA_Cmd(DMA2_Stream1, ENABLE);
        USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

        if (fifo_init(&fifo_usart_rx_6, Usart6_Rx, USART6_RX_LEN) == -1)
        {
            // 必须 2 的幂次方
        }
    }
}


void USART6_IRQHandler(void)
{
    __IO uint16_t Len = 0;

    //发送完成中断
    /*
     * DMA中断时，只表示需要传送的所有数据字节全部传送到串口的发送数据寄存器中了。
     * 此时串口实际上还有2个字节并未发送完成，数据寄存器和移位寄存器中的2个字节还需要发送，并不能关闭串口发送。
     * 同理，如果是485切换方向，必须要等到发送完成，也就是移位寄存器发送完成-TC标志置位。
    */
    if (USART_GetITStatus(USART6, USART_IT_TC) == SET)
    {
        USART_ClearITPendingBit(USART6, USART_IT_TC);
        USART_ITConfig(USART6, USART_IT_TC, DISABLE);
        DMA2_Stream6_working = 0;
    }
    if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) //触发中断标志位
    {
        Len = USART6->SR;
        Len = USART6->DR;
        Len = DMA_GetCurrDataCounter(DMA2_Stream1); //DMA_GetCurrDataCounter用于获得传输中剩余的单元数

        if (pfifo_6 != 0)
        {
            // Len为当前接收索引
            pfifo_6->in += ((pfifo_6->last_cnt - Len) & (pfifo_6->size - 1)); //更新in
            pfifo_6->last_cnt = Len;

            if ((pfifo_6->in - pfifo_6->out) > pfifo_6->size)
            {
                pfifo_6->out = pfifo_6->in; // 清空缓存，注意赋值顺序，pfifo->in = pfifo->out 是错误的
                pfifo_6->error |= FIFO_DMA_ERROR_RX_FULL;
            }
        }
        else
        {
            pfifo_6->error |= FIFO_DMA_ERROR_RX_POINT_NULL;
        }
    }
}


/**
  * @brief          发送完成中断
  * @param[in]      void
  * @retval         void
  */
void DMA2_Stream6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream6, DMA_IT_TCIF6);
        DMA_Cmd(DMA2_Stream6, DISABLE); //不使能传输
        USART_ITConfig(USART6, USART_IT_TC, ENABLE);
    }
}

/**
  * @brief          串口六+DMA 发送
  * @param[in]      void
  * @retval         void
  */
uint32_t usart6_dma_send(uint8_t *data, uint16_t len)
{
    uint32_t result = fifo_write_buff(&fifo_usart_tx_6, data, len); //将数据放入循环缓冲区

    //result != 0 说明放入数据成功 DMA2_Stream6_working == 0 说明缓冲区里面没有数据
    if (DMA2_Stream6_working == 0)
    {
        len = fifo_read_buff(&fifo_usart_tx_6, Usart6_Tx, USART6_TX_LEN); //从循环缓冲区获取数据

        DMA_SetCurrDataCounter(DMA2_Stream6, len); //设定传输长度
        DMA_Cmd(DMA2_Stream6, ENABLE); //使能传输

        DMA2_Stream6_working = 1;
    }

    if (result == len)
    {
        return len;
    }
    else
    {
        return result;
    }
}