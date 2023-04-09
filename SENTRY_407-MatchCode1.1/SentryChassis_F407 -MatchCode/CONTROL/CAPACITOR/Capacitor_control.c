#include "Capacitor_control.h"
#include "rmmotor.h"
#include "maths.h"
#include "usart.h"
#include "Task_Chassis.h"
#include "RefereeDeal.h"


static u8 GyroDataBuffer_rx[USART2_RX_LEN];

//static int16_t USART2_Receive_Flag = 0;
static u16 usart2_dataLen = 0;

static u8 SuperCap_normal_flag = 0;


static Super_Cap_t SuperCap;


static void SupeCap_receive(uint8_t *usart2_receive);


/**
  * @brief          ����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
  * @param[in]      none
  * @retval         ����ң�������Ʊ��� &rc_ctrl
  * @attention
  */
const Super_Cap_t *get_SuperCap_Control_Point(void)
{
    return &SuperCap;
}

/**
  * @brief          ���ڶ���ʼ��
  * @param[in]      none
  * @retval         none
  * @attention  
  */
void Capacitance_Control_Init(void)
{
	Usart2_Init();  //���ڶ���ʼ��
}

void USART2_IRQHandler(void) // ���������ж�
{
    u16 i;
    u8 num = 0;
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) //�����жϱ�־λ
    {
        DMA_Cmd(DMA1_Stream5, DISABLE); //�ر�DMA,��ֹ�����ڼ�������
        while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
		
        num = USART2->SR;
        num = USART2->DR;
		
        num = USART2_RX_LEN - DMA_GetCurrDataCounter(DMA1_Stream5); //��ȡ��ǰʣ����������С�ĺ���
        usart2_dataLen = num;
		
        for (i = 0; i < usart2_dataLen; i++)
            GyroDataBuffer_rx[i] = Usart2_Rx[i];
		
		SupeCap_receive(GyroDataBuffer_rx);
		
        DMA_SetCurrDataCounter(DMA1_Stream5, USART2_RX_LEN); //���ö�Ӧ�� DMA �������������������С
        DMA_Cmd(DMA1_Stream5, ENABLE); //����DMA
    }
}

/**
  * @brief          �������ݽ��պ���
  * @param[in]      usart2_receive: �������ݻ���
  * @retval         none
  * @attention  
  */
void SupeCap_receive(uint8_t *usart2_receive)
{
	if (usart2_receive[0] == 0xAC && usart2_receive[3] == 0xBD)
	{
		SuperCap.SupeCap_init_flag = usart2_receive[1];
		SuperCap.Recharged_flag = usart2_receive[2];
	}
}

/**
  * @brief          �������ݿ��Ʒ��ͺ���
  * @param[in]      send_3: �ŵ翪�ر�־λ��
  * @retval         none
  * @attention  
  */
void SuperCap_Send(u8 send_3)
{
	if (SuperCap_normal_flag == 1)
	{
		Capacitance_Power_Send(send_3);
		Capacitance_usart_iwdg();
	}
}


/**
  * @brief          �������ݵ��������ݿ��ư�
  * @param[in]      send_3: �ŵ翪�ر�־λ��
  * @retval         none
  * @attention  
  */
void Capacitance_Power_Send(u8 send_3)
{
    u8 i = 0;
    u8 SendBuff[8];
	u32 send_1;
	u16 send_2;
	
	send_1 = referee_chassis_power();  //��ȡ���̹���
	send_2 = referee_chassis_power_buffer();  //��ȡ���̻��幦��
	
	SendBuff[0] = 0xCA;
    SendBuff[1] = send_1 >> 24;
    SendBuff[2] = send_1 >> 16;
    SendBuff[3] = send_1 >> 8;
    SendBuff[4] = send_1;
    SendBuff[5] = send_2 >> 8;
    SendBuff[5] = send_2;
    SendBuff[6] = send_3;
    SendBuff[7] = 0xDB;

    for (i = 0; i < 2; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); //�������ݵ�����2
		
        USART_SendData(USART2, (uint8_t)SendBuff[i]); //�ȴ��ϴδ������
    }
}


/**
  * @brief          ���������ݿ��ư�ι��
  * @param[in]      none
  * @retval         none
  * @attention  
  */
void Capacitance_usart_iwdg(void)
{
    u8 i = 0;
    u8 temp[1];
    temp[0] = 0x01;
	
    for (i = 0; i < 1; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); //�������ݵ�����2
		
        USART_SendData(USART2, (uint8_t)temp[i]); //�ȴ��ϴδ������
    }
}


void SuperCap_OFF(void)
{
	SuperCap_normal_flag = 0;
}


