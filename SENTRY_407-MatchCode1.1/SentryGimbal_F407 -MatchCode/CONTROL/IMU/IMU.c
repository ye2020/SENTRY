#include "IMU.h"
#include "usart.h"



#if IMU_BMI160
IMU_Data IMU_t;

/*
*����2
*������ģ�飬�Լ����ʰ����ݻ�ȡ
*/

u8 GyroDataBuffer_tx[USART2_TX_LEN];
u8 GyroDataBuffer_rx[USART2_RX_LEN];


void IMU_control_init(void)
{
    Usart2_Init();  //���ڶ���ʼ�����������������ݣ�
}


/*
*���ƣ����ڶ���������ǽ����ж�
*���ܣ����������ݽ��ղ�����
*/
int16_t USART2_Receive_Flag = 0;
u16 usart2_dataLen = 0;
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

        for (i = 0; i < num; i++)
            GyroDataBuffer_rx[i] = Usart2_Rx[i];

        DMA_SetCurrDataCounter(DMA1_Stream5, USART2_RX_LEN); //���ö�Ӧ�� DMA �������������������С
        DMA_Cmd(DMA1_Stream5, ENABLE); //����DMA
        USART2_Receive_Flag = 1;

        IMU_Data_Deal();
    }
}

/* ������������ֵ */
void IMU_Data_Deal(void)
{
    u16 i;

    if (USART2_Receive_Flag)
    {
        for (i = 0; i < usart2_dataLen; i++)
        {
            if (GyroDataBuffer_rx[i] == 0xfe && GyroDataBuffer_rx[i + 13] == 0xee)
            {

                IMU_t.Gyro_X = ((short)(GyroDataBuffer_rx[1] << 8 | GyroDataBuffer_rx[2])) / 100.0f;
                IMU_t.Gyro_Y = ((short)(GyroDataBuffer_rx[3] << 8 | GyroDataBuffer_rx[4])) / 100.0f;
                IMU_t.Gyro_Z = -((short)(GyroDataBuffer_rx[5] << 8 | GyroDataBuffer_rx[6])) / 100.0f; //������ģ�鳯�ϣ��Ӹ��ţ���֮
                IMU_t.roll_angle = ((short)(GyroDataBuffer_rx[7] << 8 | GyroDataBuffer_rx[8])) / 100.0f;
                IMU_t.pitch_angle = ((short)(GyroDataBuffer_rx[9] << 8 | GyroDataBuffer_rx[10])) / 100.0f;
                IMU_t.yaw_angle = -((short)(GyroDataBuffer_rx[11] << 8 | GyroDataBuffer_rx[12])) / 100.0f; //������ģ�鳯�ϣ��Ӹ��ţ���֮

                i = i + 13;
            }
        }

        USART2_Receive_Flag = 0;
    }
}


/*
*���ܣ��������ݵ�������ģ����г�ʼ��У׼����̨���ڶ���
*Э�飺֡ͷ  У���0x12 0x34  ֡β
*/
void BMI160_Zero_Correct(void)
{
    u8 i = 0;
    u8 SendBuff_Correct[2];
    SendBuff_Correct[0] = 0x12;
    SendBuff_Correct[1] = 0x34;

    for (i = 0; i < 2; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); //�������ݵ�����2

        USART_SendData(USART2, (uint8_t)SendBuff_Correct[i]); //�ȴ��ϴδ������
    }
}

/*
*���ܣ���������ι��������̨���ڶ���
*Э�飺0x01
*/

void Gyro_usart_iwdg(void)
{
    u8 i = 0;
    u8 gyro[1];
    gyro[0] = 0x01;

    for (i = 0; i < 1; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
            ;                                     //�������ݵ�����2

        USART_SendData(USART2, (uint8_t)gyro[i]); //�ȴ��ϴδ������
    }
}

#endif
