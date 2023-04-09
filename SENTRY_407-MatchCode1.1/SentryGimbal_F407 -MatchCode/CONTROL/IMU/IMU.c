#include "IMU.h"
#include "usart.h"



#if IMU_BMI160
IMU_Data IMU_t;

/*
*串口2
*陀螺仪模块，以及功率板数据获取
*/

u8 GyroDataBuffer_tx[USART2_TX_LEN];
u8 GyroDataBuffer_rx[USART2_RX_LEN];


void IMU_control_init(void)
{
    Usart2_Init();  //串口二初始化（接收陀螺仪数据）
}


/*
*名称：串口二外接陀螺仪接收中断
*功能：陀螺仪数据接收并保存
*/
int16_t USART2_Receive_Flag = 0;
u16 usart2_dataLen = 0;
void USART2_IRQHandler(void) // 接收数据中断
{
    u16 i;
    u8 num = 0;

    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) //触发中断标志位
    {
        DMA_Cmd(DMA1_Stream5, DISABLE); //关闭DMA,防止处理期间有数据

        while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);

        num = USART2->SR;
        num = USART2->DR;
        num = USART2_RX_LEN - DMA_GetCurrDataCounter(DMA1_Stream5); //获取当前剩余数据量大小的函数
        usart2_dataLen = num;

        for (i = 0; i < num; i++)
            GyroDataBuffer_rx[i] = Usart2_Rx[i];

        DMA_SetCurrDataCounter(DMA1_Stream5, USART2_RX_LEN); //设置对应的 DMA 数据流传输的数据量大小
        DMA_Cmd(DMA1_Stream5, ENABLE); //开启DMA
        USART2_Receive_Flag = 1;

        IMU_Data_Deal();
    }
}

/* 接收陀螺仪数值 */
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
                IMU_t.Gyro_Z = -((short)(GyroDataBuffer_rx[5] << 8 | GyroDataBuffer_rx[6])) / 100.0f; //陀螺仪模块朝上，加负号，反之
                IMU_t.roll_angle = ((short)(GyroDataBuffer_rx[7] << 8 | GyroDataBuffer_rx[8])) / 100.0f;
                IMU_t.pitch_angle = ((short)(GyroDataBuffer_rx[9] << 8 | GyroDataBuffer_rx[10])) / 100.0f;
                IMU_t.yaw_angle = -((short)(GyroDataBuffer_rx[11] << 8 | GyroDataBuffer_rx[12])) / 100.0f; //陀螺仪模块朝上，加负号，反之

                i = i + 13;
            }
        }

        USART2_Receive_Flag = 0;
    }
}


/*
*功能：发送数据到陀螺仪模块进行初始化校准（云台串口二）
*协议：帧头  校验段0x12 0x34  帧尾
*/
void BMI160_Zero_Correct(void)
{
    u8 i = 0;
    u8 SendBuff_Correct[2];
    SendBuff_Correct[0] = 0x12;
    SendBuff_Correct[1] = 0x34;

    for (i = 0; i < 2; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); //发送数据到串口2

        USART_SendData(USART2, (uint8_t)SendBuff_Correct[i]); //等待上次传输完成
    }
}

/*
*功能：给陀螺仪喂狗粮（云台串口二）
*协议：0x01
*/

void Gyro_usart_iwdg(void)
{
    u8 i = 0;
    u8 gyro[1];
    gyro[0] = 0x01;

    for (i = 0; i < 1; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
            ;                                     //发送数据到串口2

        USART_SendData(USART2, (uint8_t)gyro[i]); //等待上次传输完成
    }
}

#endif
