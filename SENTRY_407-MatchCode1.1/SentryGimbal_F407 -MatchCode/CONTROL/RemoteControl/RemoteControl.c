/**
  ******************************************************************************
  * @file       RemoteControl.c/h
  * @brief      ���ң�������ݽ��ա�
  ******************************************************************************
  */
#include "RemoteControl.h"
#include "maths.h"
#include "usart.h"
#include "iwdg.h"

/*
ch0Ϊ�ұߵ�����
ch1Ϊ�ұߵ�����
ch2Ϊ��ߵ�����
ch3Ϊ��ߵ�����
ch4Ϊ���������Ͻǣ�
s1Ϊ����
s2Ϊ����
*/

/*****************************������λ˵����**********************************************************/
/* һ��ң��ģʽ��
         1.���̸���  ����������
         2.Ť��ģʽ  ����������
         3.����С���ݣ���������
         4.���ģʽ  ���������У���������̨�������ƶ��������棩
         5.����ģʽ  ����������
         6.����ģʽ  ����������
         7.����      ���ڳ�ʼ��������ҿ��ش��ϻ�������£����Ͻǲ��������Ϸ���������ֹͣ�䵯
         8.�ػ�      ������
   ��������ģʽ��
         1.�����˶���WASD
         2.��̨�˶������
         3.���䣺    ���������㵥������ס����
         4.���٣����ݷŵ磩��    ��סshift����Ϻ����
         5.Ť��ģʽ�� F  ����һ�½��룬�ٰ�һ�·��أ�
         6.����ģʽ�� ����Ҽ�  ����ס��
         7.����ģʽ��  G  ����һ����̨��һ��ת�����굯�ٰ�һ�»�����
         8.������ģʽ��C ����һ�ν��룩�����ģʽû���ˣ�֮ǰȫ��������ٴ�
         9.������ģʽ��V ����һ�ν��룩
         10.�˵�ģʽ�� Z  ����ס��
         11.��̨ģʽ�� Ctrl ����ס��ֻ�ܿ�����̨�����̲�����
         12.���ģʽ�� X ��һ�ν��룩
         13.С����ģʽ��R����һ�ν��룩
                                                                                                    */
/****************************************************************************************************/


#define rc_deadline_limit(input, output, dealine)    \
{                                                    \
	if ((input) > (dealine) || (input) < -(dealine)) \
	{                                                \
		(output) = (input);                          \
	}                                                \
	else                                             \
	{                                                \
		(output) = 0;                                \
	}                                                \
}


//ң����������������
#define RC_CHANNAL_ERROR_VALUE 700

static uint8_t Usart1_Rx[USART1_RX_LEN]; //������ջ���
int RC_FLAG;

RC_ctrl_t rc_ctrl;
//static RC_ctrl_t rc_ctrl;

//ң������ʼ��
void remote_control_init(void)
{
	Usart1_Init(Usart1_Rx, USART1_RX_LEN);
}

//����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
const RC_ctrl_t *Get_Remote_Control_Point(void)
{
    return &rc_ctrl;
}



//�ж�ң���������Ƿ����
void RC_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    if (int16_t_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error;
    }
    if (int16_t_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error;
    }
    if (int16_t_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error;
    }
    if (int16_t_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error;
    }
    if (rc_ctrl.rc.s1 == RC_SW_ERROR)
    {
        goto rc_error;
    }
    if (rc_ctrl.rc.s2 == RC_SW_ERROR)
    {
        goto rc_error;
    }
    return ;

rc_error:
	rc_ctrl.rc.ch[0] = 0;
	rc_ctrl.rc.ch[1] = 0;
	rc_ctrl.rc.ch[2] = 0;
	rc_ctrl.rc.ch[3] = 0;
	rc_ctrl.rc.ch[4] = 0;
//	rc_ctrl.rc.s1 = RC_SW_ERROR;  //���ִ���Ϊ0
//	rc_ctrl.rc.s2 = RC_SW_ERROR;  //���ִ���Ϊ0
//	rc_ctrl.rc.s1 = RC_SW_MID;   //��
//	rc_ctrl.rc.s2 = RC_SW_DOWN;  //��
	rc_ctrl.mouse.x = 0;
	rc_ctrl.mouse.y = 0;
	rc_ctrl.mouse.z = 0;
	rc_ctrl.mouse.press_l = 0;
	rc_ctrl.mouse.press_r = 0;
	rc_ctrl.key.v = 0;
	
//	//����ң����
//	delay_ms(2);
//	RC_restart(USART1_RX_LEN);  //18
//	delay_ms(2);
}


/* ҡ�������� */
void Remote_reload(void)
{
	rc_ctrl.rc.ch[0] = 0;
	rc_ctrl.rc.ch[1] = 0;
	rc_ctrl.rc.ch[2] = 0;
	rc_ctrl.rc.ch[3] = 0;
	rc_ctrl.rc.ch[4] = 0;
	rc_ctrl.mouse.x = 0;
	rc_ctrl.mouse.y = 0;
	rc_ctrl.mouse.z = 0;
	rc_ctrl.mouse.press_l = 0;
	rc_ctrl.mouse.press_r = 0;
	rc_ctrl.key.v = 0;
}


/* ң�������� */
void RC_restart(uint16_t dma_buf_num)
{
	USART_Cmd(USART1, DISABLE);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_SetCurrDataCounter(DMA2_Stream2, dma_buf_num);  //���ö�Ӧ�� DMA �������������������С

	USART_ClearFlag(USART1, USART_FLAG_IDLE);

	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	USART_Cmd(USART1, ENABLE);
}



static void RC_Deal(RC_ctrl_t *rc_ctrl, volatile const uint8_t *sbus_buf)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)  return;
	
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s1 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                    //!< Switch left
    rc_ctrl->rc.s2 = ((sbus_buf[5] >> 4) & 0x0003);                         //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = ((int16_t)sbus_buf[16]|((int16_t)sbus_buf[17]<<8))&0x07FF;//ң�����Ͻǲ���

	rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;  // -ͨ����ֵ1024
	rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;  // �� -660 ~ 660 ��
	rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	
	rc_deadline_limit(rc_ctrl->rc.ch[0], rc_ctrl->rc.ch[0], 10);  //��������
	rc_deadline_limit(rc_ctrl->rc.ch[1], rc_ctrl->rc.ch[1], 10);  //��������
	rc_deadline_limit(rc_ctrl->rc.ch[2], rc_ctrl->rc.ch[2], 10);  //��������
	rc_deadline_limit(rc_ctrl->rc.ch[3], rc_ctrl->rc.ch[3], 10);  //��������
}



void DMA2_Stream5_IRQHandler(void)		//ң�����ݵĽ���
{
	//�ж��Ƿ�ΪDMA��������ж�
	if(DMA_GetFlagStatus(DMA2_Stream5,DMA_IT_TCIF5)==SET) 
	{
		LEDE0 = 0; 
		
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5); 
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		
		//�����յ������ݱ��浽Usart1_Rx��
		RC_Deal(&rc_ctrl, Usart1_Rx);
		
#ifdef watch_dog
		RC_FLAG++;
		if(500 == RC_FLAG)
		{
			IWDG_Feed();
			RC_FLAG = 0;
		}
#endif

	}
}


/**
  * @brief          ����s1����ֵ
  * @param[in]      none
  * @retval         ����s1����ֵ
  * @attention      
  */
char Return_RC_S1num(void)
{
	return (rc_ctrl.rc.s1);
}

/**
  * @brief          ����s2����ֵ
  * @param[in]      none
  * @retval         ����s2����ֵ
  * @attention      
  */
char Return_RC_S2num(void)
{
	return (rc_ctrl.rc.s2);
}





