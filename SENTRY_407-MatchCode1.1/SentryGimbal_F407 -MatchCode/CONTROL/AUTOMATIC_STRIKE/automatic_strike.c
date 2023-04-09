#include "automatic_strike.h"  //����
#include "usart.h"



Vision_Auto_Data_t Vision_Auto_Data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};



void automatic_aiming_init(void)
{
	Usart3_Init();  //��������ʼ�� 
}

//�����Ӿ�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
Vision_Auto_Data_t *Get_Auto_Control_Point(void)
{
    return &Vision_Auto_Data;
}


/**
  * @brief          �������Ӿ��жϺ���
  * @param[in]      none
  * @retval         none
  * @attention      ���ܣ��Ӿ����ݽ���,�����ڽ���������
  */
int16_t USART3_Receive_Flag = 0;
u16 usart3_dataLen = 0;
u8 MSDataBuffer[USART3_RX_LEN];

void USART3_IRQHandler(void)			// ������С���Խ��������ж�
{
    u16 i;
    u8 num = 0;

    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//�����жϱ�־λ
    {
        DMA_Cmd(DMA1_Stream1, DISABLE);                    //�ر�DMA,��ֹ�����ڼ�������

        while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);

        num = USART3->SR;
        num = USART3->DR;
        num = USART3_RX_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);
        usart3_dataLen = num;

        for(i = 0; i < num; i++)
        {
            MSDataBuffer[i] = Usart3_Rx[i];
        }

        DMA_SetCurrDataCounter(DMA1_Stream1, USART3_RX_LEN);
        DMA_Cmd(DMA1_Stream1, ENABLE);//����DMA
        USART3_Receive_Flag = 1;
    }
}


/*
*���ܣ��������ݵ�MiniPC����������
*���룺������ɫ��pitch�Ƕ�   ����
*�������
*Э�飺֡ͷ0xF5  У���0xF5 0x00 0x00  ֡β��0xF6
*/
char SendBuff[5];

void MiniPC_Send_Data(u8 data, u8 mode, u8 shoot_speed)
{
    static unsigned char i = 0;

    SendBuff[0] = 0xFF;
    SendBuff[1] = data;
    SendBuff[2] = mode;
    SendBuff[3] = shoot_speed;
    SendBuff[4] = 0xFE;

    for(i = 0; i < 5; i++)
    {
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);  	 //�������ݵ�����3

        USART_SendData(USART3, (uint8_t)SendBuff[i]); //�ȴ��ϴδ������
    }
}


/*
*���ƣ�MiniPC���ڽ������ݴ���
*���ܣ�ͨ�����������ղ�����MiniPC����������
*���룺��
*�������
*/

void MiniPC_Data_Deal(void) //�Ӿ������У�ϵͳ����
{
    u16 i;

    if(USART3_Receive_Flag)
    {
        for(i = 0; i < usart3_dataLen; i++)
        {
            if(MSDataBuffer[i] == 0xFF && MSDataBuffer[i + 7] == 0xFE)
            {
                Vision_Auto_Data.auto_yaw_angle = (hex2Float(MSDataBuffer[i + 2], MSDataBuffer[i + 1]) / 100); //�Զ������y��Ƕȼ���/100
                Vision_Auto_Data.auto_pitch_angle = (hex2Float(MSDataBuffer[i + 4], MSDataBuffer[i + 3]) / 100); //�Զ������p��Ƕȼ���
                Vision_Auto_Data.len = (hex2Float(MSDataBuffer[i + 6], MSDataBuffer[i + 5]) / 100); //����

                i = i + 7;
            }
        }

        USART3_Receive_Flag = 0;
    }
}


/*
*���ܣ��Ӿ����������ݳ�ʼ��
*
*/
void MiniPC_Kalman_Data_Init(void)
{
    {
        kalman_Filter_Init.A_data[0] = 1;
        kalman_Filter_Init.A_data[2] = 1;
        kalman_Filter_Init.A_data[5] = 1;
        kalman_Filter_Init.A_data[7] = 1;
        kalman_Filter_Init.A_data[10] = 1;
        kalman_Filter_Init.A_data[15] = 1;
        //�۲����
        kalman_Filter_Init.H_data[0] = 1;
        kalman_Filter_Init.H_data[5] = 1;
        //״̬ת��Э�������
        kalman_Filter_Init.P_data[0] = 1;
        kalman_Filter_Init.P_data[5] = 1;
        kalman_Filter_Init.P_data[10] = 1;
        kalman_Filter_Init.P_data[15] = 1;
        //�۲���������
        kalman_Filter_Init.R_data[0] = 1;
        kalman_Filter_Init.R_data[3] = 1;
        //״̬ת��Э�������
        kalman_Filter_Init.Q_data[0] = 10;
        kalman_Filter_Init.Q_data[5] = 10;
        kalman_Filter_Init.Q_data[10] = 0.01;
        kalman_Filter_Init.Q_data[15] = 0.01;
    }
    Kalman_Filter_Init(&kalman_Filter, &kalman_Filter_Init);
}





/*
*���ܣ��ߵͰ�λ��������
*/
float hex2Float(uint8_t HighByte, uint8_t LowByte)
{
    float high = (float) (HighByte & 0x7f);
    float low  = (float) LowByte;

    if (HighByte & 0x80)//MSB is 1 means a negative number
    {
        return (high * 256.0f + low) - 32768;
    }
    else
    {
        return (high * 256.0f + low);
    }
}


