#include "usart.h"



//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 0
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while((USART3->SR & 0X40) == 0); //ѭ������,ֱ���������

    USART3->DR = (u8) ch;
    return ch;
}
#endif



/************************************************************����һ��ʼ��**************************************************************/
void Usart1_Init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{
    #if 1  //ͨ�ð�
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; //�ṹ������

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //ʱ��ʹ��

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure); //����io������

    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 100000; //ң�ؽ��ղ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_Mode = USART_Mode_Rx; //����
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);                     //ʹ��USART1
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); //ʹ�ܴ���DMA����

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); //�ж����ȼ�

    DMA_Cmd(DMA2_Stream5, DISABLE); //�ر�DMA

    while (DMA2_Stream5->CR & DMA_SxCR_EN)
        ; //�ȴ� DMA ������

    DMA_DeInit(DMA2_Stream5); //����Ϊȱʡֵ
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);   //Դ��ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;              //Ŀ�ĵ�ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //���ݴ��䷽��Ϊ���赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = dma_buf_num;                         //�������ݵĻ����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         //������ѭ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE); //dma������ɲ����ж�

    DMA_Cmd(DMA2_Stream5, ENABLE); //����DMA
    #else  //�ٷ����ӣ��ɲ�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* config USART1 clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    /*ң��Pin�� ����PB7*/
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
    USART_InitStructure.USART_BaudRate = 100000;	 // ������						 											//SBUS 100K baudrate
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �ֳ�
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  // ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_Even;  // У��λ
    USART_InitStructure.USART_Mode = USART_Mode_Rx;      // USART ģʽ   ����ģʽ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Ӳ��������  ��Ӳ����
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    DMA_Cmd(DMA2_Stream5, DISABLE);

    while (DMA2_Stream5->CR & DMA_SxCR_EN);

    DMA_DeInit(DMA2_Stream5);												 														//����Ϊȱʡֵ
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);      //Դ��ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;                 //Ŀ�ĵ�ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                    //���ݴ��䷽��Ϊ���赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = dma_buf_num;                            //�������ݵĻ����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;           //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    //�ڴ滺������ַ�Լ�
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;    //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;            //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                            //������ѭ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                    //������ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA2_Stream5, ENABLE);
    #endif
}



/************************************************************��������ʼ��**************************************************************/
u8 Usart3_Rx[USART3_RX_LEN] = {0};
u8 Usart3_Tx[USART3_TX_LEN] = {0};

void Usart3_Init(void)
{
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //ʹ��USART3ʱ��
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3); //GPIOD8����ΪUSART3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); //GPIOD9����ΪUSART3
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
        USART3_InitStruct.USART_Parity = USART_Parity_No; //����żУ��
        USART3_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART3_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART3, &USART3_InitStruct);

        NVIC_InitStructure.NVIC_IRQChannel						=	USART3_IRQn;		//DMA1_Stream1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1;
        NVIC_Init(&NVIC_InitStructure);
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
        USART_Cmd(USART3, ENABLE);
    }
    /* -------------- Configure DMA -----------------------------------------*/
    {
        //��������
        DMA_InitTypeDef DMA_InitStruct;
        DMA_DeInit(DMA1_Stream3);					//����Ϊȱʡֵ
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);			//Դ��ַ
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart3_Tx);			//Ŀ�ĵ�ַ
        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;				//���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStruct.DMA_BufferSize = USART3_TX_LEN;							//�������ݵĻ����С
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//�����ַ����
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;								//�ڴ滺������ַ�Լ�
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8λ�ֽڴ���
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				//���ݿ��Ϊ8λ
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;												//������ѭ������ģʽ
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;								//������ȼ�
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal;//DMA_MemoryBurst_Single;//
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream3, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream3, DISABLE);
        USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE); //ʹ�ܴ���3��DMA����

        // ��������
        DMA_DeInit(DMA1_Stream1);					//����Ϊȱʡֵ
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);			//Դ��ַ
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart3_Rx);			//Ŀ�ĵ�ַ
        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;				//���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStruct.DMA_BufferSize = USART3_RX_LEN;							//�������ݵĻ����С
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//�����ַ����
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;								//�ڴ滺������ַ�Լ�
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8λ�ֽڴ���
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				//���ݿ��Ϊ8λ
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;												//������ѭ������ģʽ
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;								//������ȼ�
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal;//DMA_MemoryBurst_Single;//
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream1, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream1, ENABLE);
        USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    }
}



/************************************************************���ڶ���ʼ��**************************************************************/
//u8 Usart2_Rx[USART2_RX_LEN] = {0};
//u8 Usart2_Tx[USART2_TX_LEN] = {0};

//void Usart2_Init(void)
//{
//    /* -------------- Enable Module Clock Source ----------------------------*/
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);    //ʹ��USART3ʱ��
//    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //GPIOD8����ΪUSART3
//    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); //GPIOD9����ΪUSART3

//    /* -------------- Configure GPIO ---------------------------------------*/
//    {
//        GPIO_InitTypeDef GPIO_InitStruct;
//        NVIC_InitTypeDef NVIC_InitStructure;
//        USART_InitTypeDef USART2_InitStruct;

//        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
//        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//        GPIO_Init(GPIOD, &GPIO_InitStruct);

//        USART_DeInit(USART2);
//        USART2_InitStruct.USART_BaudRate = 115200;
//        USART2_InitStruct.USART_WordLength = USART_WordLength_8b;
//        USART2_InitStruct.USART_StopBits = USART_StopBits_1;
//        USART2_InitStruct.USART_Parity = USART_Parity_No; //����żУ��
//        USART2_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//        USART2_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//        USART_Init(USART2, &USART2_InitStruct);

//        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //DMA1_Stream1_IRQn;
//        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//        NVIC_Init(&NVIC_InitStructure);
//        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
//        USART_Cmd(USART2, ENABLE);
//    }

//    /* -------------- Configure DMA -----------------------------------------*/
//    {
//        DMA_InitTypeDef DMA_InitStruct;

//        //��������
//        DMA_DeInit(DMA1_Stream6); //����Ϊȱʡֵ
//        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
//        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);   //Դ��ַ
//        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart2_Tx);          //Ŀ�ĵ�ַ
//        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //���ݴ��䷽��Ϊ���赽�ڴ�
//        DMA_InitStruct.DMA_BufferSize = USART2_TX_LEN;                       //�������ݵĻ����С
//        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
//        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
//        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
//        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
//        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //������ѭ������ģʽ
//        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
//        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal; //DMA_MemoryBurst_Single;//
//        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//        DMA_Init(DMA1_Stream6, &DMA_InitStruct);
//        DMA_Cmd(DMA1_Stream6, DISABLE);
//        USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); //ʹ�ܴ���2��DMA����

//        // ��������
//        DMA_DeInit(DMA1_Stream5); //����Ϊȱʡֵ
//        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
//        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);   //Դ��ַ
//        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart2_Rx);          //Ŀ�ĵ�ַ
//        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //���ݴ��䷽��Ϊ���赽�ڴ�
//        DMA_InitStruct.DMA_BufferSize = USART2_RX_LEN;                       //�������ݵĻ����С
//        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
//        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
//        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
//        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
//        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //������ѭ������ģʽ
//        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
//        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal; //DMA_MemoryBurst_Single;//
//        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//        DMA_Init(DMA1_Stream5, &DMA_InitStruct);
//        DMA_Cmd(DMA1_Stream5, ENABLE);
//        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
//    }
//}

/*****************************************************************��������ʼ��**************************************************************/
u8 Usart6_Rx[USART6_RX_LEN] = {0};
u8 Usart6_Tx[USART6_TX_LEN] = {0};

void Usart6_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART6_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);
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
    USART6_InitStruct.USART_Parity = USART_Parity_No; //����żУ��
    USART6_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &USART6_InitStruct);

    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

    USART_Cmd(USART6, ENABLE);

    DMA_DeInit(DMA2_Stream1); //����Ϊȱʡֵ
    DMA_InitStruct.DMA_Channel = DMA_Channel_5;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);   //Դ��ַ
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart6_Rx);          //Ŀ�ĵ�ַ
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //���ݴ��䷽��Ϊ���赽�ڴ�
    DMA_InitStruct.DMA_BufferSize = USART6_RX_LEN;                       //�������ݵĻ����С
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //������ѭ������ģʽ
    DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1, &DMA_InitStruct);
    DMA_Cmd(DMA2_Stream1, ENABLE);

    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
}




