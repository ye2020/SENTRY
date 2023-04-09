#include "RefereeDeal.h"
#include "usart.h"
#include "crc.h"


/**
  * @brief          串口六初始化
  * @param[in]      none
  * @retval         none
  * @attention
  */
void Referee_System_Init(void)
{
    Usart6_Init();  //裁判系统
}

//static
REFEREE_t REFEREE;    //裁判系统数据结构体
xFrameHeader_t FrameHeader;  //发送帧头信息


/**
  * @brief          获得裁判系统数据控制量
  * @param[in]      none
  * @retval         裁判系统数据控制指针 &REFEREE
  * @attention
  */
REFEREE_t *Return_Referee_Point(void)
{
    return &REFEREE;
}


/*数据段长度*/
static const u8 HeaderLen = 5;
static const u8 CmdIdLen = 2;
static const u8 CRC16Len = 2;

bool_t Judge_Data_TF = 0;//裁判数据是否可用,辅助函数调用

#if JUDGE_VERSION == JUDGE_SZ

#elif JUDGE_VERSION == JUDGE_ACE
/*比赛状态*/
static void GAME_STATUS(REFEREE_t *referee, unsigned char k);
/*机器人状态*/
static void ROBOT_STATUS(REFEREE_t *referee, unsigned char k);
/*功率热量*/
static void POWER_HEAT(REFEREE_t *referee, unsigned char k);
/*伤害状态*/
static void DAMAGE_STATUS(REFEREE_t *referee, unsigned char k);

/*比赛状态数据段长度*/
static const u8 GameStatusLen = (3 + CmdIdLen + CRC16Len);
/*机器人状态数据段长度*/
static const u8 RobotStatusLen = (18 + CmdIdLen + CRC16Len);
/*功率热量数据段长度*/
static const u8 PowerHeatLen = (16 + CmdIdLen + CRC16Len);

/*裁判数据接收数据处理*/
void RefereeDataDeal(REFEREE_t *referee)
{
    u8 i, k;

    for (i = 0; i < referee->DataLen; i++)
    {
        if (referee->RefereeData[i] == 0xA5) //帧头
        {
            if (Verify_CRC8_Check_Sum(referee->RefereeData, HeaderLen) == 1) //CRC8校验
            {
                referee->RealLen = ((referee->RefereeData[i + 1]) | (referee->RefereeData[i + 2] << 8)); //数据帧中 data 的长度
                referee->Cmd_ID = ((referee->RefereeData[i + HeaderLen]) | (referee->RefereeData[i + HeaderLen + 1] << 8)); //命令码ID

                for (k = 0; k < 7; k++)
                    referee->RealData[k] = referee->RefereeData[k + i]; //提取数据 也就是接收：frame_header+cmd_id

                /* 判断cmd_id，接收对应的数据信息 */
                if (referee->Cmd_ID == 0x0001)
                {
                    GAME_STATUS(referee, (i + HeaderLen + CmdIdLen)); //比赛状态数据处理
                    i = i + HeaderLen + GameStatusLen - 1;
                }
                else if (referee->Cmd_ID == 0x0002) //比赛结果数据
                {
                }
                else if (referee->Cmd_ID == 0x0003) //机器人血量
                {
                }
                else if (referee->Cmd_ID == 0x0004) //飞镖发射状态
                {
                }
                else if (referee->Cmd_ID == 0x0101) //场地事件数据
                {
                }
                else if (referee->Cmd_ID == 0x0102) //场地补给站动作标识数据，动作发生后发送
                {
                }
                else if (referee->Cmd_ID == 0x0104) //裁判警告数据，警告发生后发送
                {
                }
                else if (referee->Cmd_ID == 0x0105) //飞镖发射口倒计时，1Hz 周期发送
                {
                }
                else if (referee->Cmd_ID == 0x0201) //机器人状态数据
                {
                    ROBOT_STATUS(referee, (i + HeaderLen + CmdIdLen));
                    i = i + HeaderLen + RobotStatusLen - 1;
                }
                else if (referee->Cmd_ID == 0x0202) //功率热量数据
                {
                    POWER_HEAT(referee, (i + HeaderLen + CmdIdLen));
                    i = i + HeaderLen + PowerHeatLen - 1;
                }
                else if (referee->Cmd_ID == 0x0203) //机器人位置数据，10Hz 发送
                {
                }
                else if (referee->Cmd_ID == 0x0204) //机器人增益数据
                {
                }
                else if (referee->Cmd_ID == 0x0205) //飞机数据
                {
                }
                else if (referee->Cmd_ID == 0x0206) //伤害状态
                {
                    DAMAGE_STATUS(referee, (i + HeaderLen + CmdIdLen));
                }
                else if (referee->Cmd_ID == 0x0207) //实时射击信息
                {
                }
                else if (referee->Cmd_ID == 0x0208) //子弹剩余发射数
                {
                }
                else if (referee->Cmd_ID == 0x0209) //RFID状态
                {
                }
            }
        }
    }

    REFEREE.RECEIVE_FLAG = 0; //处理完一组数据
}

/*比赛状态数据*/
static void GAME_STATUS(REFEREE_t *referee, unsigned char k) //0x0001
{
    u8 j;

    for (j = 0; j < GameStatusLen; j++) //数据重组
        referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];  //接收 data 位置

    if (Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen + GameStatusLen)) == 1) //CRC16校验
    {
        referee->GameStatus.game_type = referee->RealData[7];
        referee->GameStatus.game_progress = (referee->RealData[7] >> 4);
        referee->GameStatus.stage_remain_time = ((referee->RealData[8]) | (referee->RealData[9] << 8));
        referee->GameStatus.error = 0;
    }
    else
    {
        referee->GameStatus.error = 1;
    }
}

/*机器人状态数据*/
static void ROBOT_STATUS(REFEREE_t *referee, unsigned char k) //0x0201
{
    u8 j;

    for (j = 0; j < RobotStatusLen; j++) //数据重组
        referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];

    if (Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen + RobotStatusLen)) == 1) //CRC16校验
    {
        referee->RobotStatus.robot_id = referee->RealData[7];
        referee->RobotStatus.robot_level = referee->RealData[8];
        referee->RobotStatus.remain_HP = ((referee->RealData[9]) | (referee->RealData[10] << 8));
        referee->RobotStatus.max_HP = ((referee->RealData[11]) | (referee->RealData[12] << 8));
        referee->RobotStatus.shooter_heat0_cooling_rate = ((referee->RealData[13]) | (referee->RealData[14] << 8));
        referee->RobotStatus.shooter_heat0_cooling_limit = ((referee->RealData[15]) | (referee->RealData[16] << 8));
        referee->RobotStatus.shooter_heat1_cooling_rate = ((referee->RealData[17]) | (referee->RealData[18] << 8));
        referee->RobotStatus.shooter_heat1_cooling_limit = ((referee->RealData[19]) | (referee->RealData[20] << 8));
        referee->RobotStatus.shooter_heat0_speed_limit = (referee->RealData[21]);
        referee->RobotStatus.shooter_heat1_speed_limit = (referee->RealData[22]);
        referee->RobotStatus.max_chassis_power = (referee->RealData[23]);
        referee->RobotStatus.mains_power_gimbal_output = referee->RealData[24];
        referee->RobotStatus.mains_power_chassis_output = (referee->RealData[24] >> 1);
        referee->RobotStatus.mains_power_shooter_output = (referee->RealData[24] >> 2);

        referee->RobotStatus.error = 0;
    }
    else
    {
        referee->RobotStatus.error = 1;
    }
}

/*功率热量数据*/
static void POWER_HEAT(REFEREE_t *referee, unsigned char k) //0x0202
{
    u8 j;
    Float_t F;

    for (j = 0; j < PowerHeatLen; j++) //数据重组
        referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];

    if (Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen + PowerHeatLen)) == 1) //CRC16校验
    {
        referee->PowerHeat.chassis_volt = ((referee->RealData[7]) | (referee->RealData[8] << 8));
        referee->PowerHeat.chassis_current = ((referee->RealData[9]) | (referee->RealData[10] << 8));
        F.float_byte.LB = (referee->RealData[11]);
        F.float_byte.MLB = (referee->RealData[12]);
        F.float_byte.MHB = (referee->RealData[13]);
        F.float_byte.HB = (referee->RealData[14]);
        referee->PowerHeat.chassis_power = F.value;
        referee->PowerHeat.chassis_power_buffer = ((referee->RealData[15]) | (referee->RealData[16] << 8));
        referee->PowerHeat.shooter_heat0 = ((referee->RealData[17]) | (referee->RealData[18] << 8));
        referee->PowerHeat.shooter_heat1 = ((referee->RealData[19]) | (referee->RealData[20] << 8));
        referee->PowerHeat.mobile_shooter_heat2 = ((referee->RealData[21]) | (referee->RealData[22] << 8));

        referee->PowerHeat.error = 0;
    }
    else
    {
        referee->PowerHeat.error = 1;
    }
}

/*伤害状态*/
static void DAMAGE_STATUS(REFEREE_t *referee, unsigned char k)
{
    u8 j;

    for (j = 0; j < PowerHeatLen; j++) //数据重组
        referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];

    if (Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen + PowerHeatLen)) == 1) //CRC16校验
    {
        referee->RobotHurt.armor_id = referee->RealData[7];
        referee->RobotHurt.hurt_type = (referee->RealData[7] >> 4);

        referee->RobotHurt.error = 0;
    }
    else
    {
        referee->RobotHurt.error = 1;
    }
}

#endif

/*中断*/
void USART6_IRQHandler(void)
{
    unsigned short i = 0;
    unsigned short Len = 0;

    if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) //触发中断标志位
    {
        DMA_Cmd(DMA2_Stream1, DISABLE); //关闭DMA,防止处理期间有数据

        while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);

        Len = USART6->SR;
        Len = USART6->DR;
        Len = USART6_RX_LEN - DMA_GetCurrDataCounter(DMA2_Stream1); //DMA_GetCurrDataCounter用于获得传输中剩余的单元数
        REFEREE.DataLen = Len;

        for (i = 0; i < Len; i++)
        {
            REFEREE.RefereeData[i] = Usart6_Rx[i];
        }

        Referee_Read_Data(&REFEREE, Usart6_Rx);
        memset(Usart6_Rx, 0, USART6_RX_LEN);

//		RefereeDataDeal(&REFEREE);
        DMA_SetCurrDataCounter(DMA2_Stream1, USART6_RX_LEN); //设置要传入的数据单元数
        DMA_Cmd(DMA2_Stream1, ENABLE);                       //开启DMA
        REFEREE.RECEIVE_FLAG = 1;
    }
}




/**
  * @brief          读取裁判数据,中断中读取保证速度
  * @param[in]      接收裁判系统数据结构体,缓存数据
  * @retval         是否对正误判断做处理
  * @attention      在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
u8 Referee_Read_Data(REFEREE_t *referee, uint8_t *usart6_receive)
{
    bool_t retval_tf = 0;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误

    uint16_t judge_length;//统计一帧数据长度

    int CmdID = 0;//数据命令码解析

    //无数据包，则不作任何处理
    if (usart6_receive == NULL)
    {
        return 0;
    }

    //写入帧头数据,用于判断是否开始存储裁判数据
    memcpy(&FrameHeader, referee->RefereeData, HeaderLen);

    //判断帧头数据是否为0xA5
    if(usart6_receive[0] == JUDGE_FRAME_HEADER)
    {
        //帧头CRC8校验
        if (Verify_CRC8_Check_Sum( usart6_receive, HeaderLen ) == 1)
        {
            //统计一帧数据长度,用于CR16校验
            judge_length = usart6_receive[1] + HeaderLen + CmdIdLen + CRC16Len;;

            //帧尾CRC16校验
            if(Verify_CRC16_Check_Sum(usart6_receive, judge_length) == 1)
            {
                retval_tf = 1;//都校验过了则说明数据可用

                CmdID = (usart6_receive[6] << 8 | usart6_receive[5]);

                //解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                switch(CmdID)
                {
//                    case 0x0001: //比赛状态数据
//                        memcpy(&referee->GameStatus, (usart6_receive + DATA), LEN_game_state);
//                        break;

//                    case 0x0002: //比赛结果数据
//                        memcpy(&referee->GmaeResult, (usart6_receive + DATA), LEN_game_result);
//                        break;

//                    case 0x0003:  //机器人血量数据
//                        memcpy(&referee->RobotHP, (usart6_receive + DATA), LEN_game_robot_survivors);
//                        break;

//                    case 0x0004:  //飞镖发射状态
//                        memcpy(&referee->DartStatus, (usart6_receive + DATA), LEN_game_robot_survivors);
//                        break;

//                    case 0x0101:  //场地事件数据
//                        memcpy(&referee->EventData, (usart6_receive + DATA), LEN_event_data);
//                        break;

                    case 0x0102:  //补给站动作标识
                        memcpy(&referee->SupplyAction, (usart6_receive + DATA), LEN_supply_projectile_action);
                        break;

//                    case 0x0104:  //裁判警告信息
//                        memcpy(&referee->RefereeWarning, (usart6_receive + DATA), LEN_supply_projectile_booking);
//                        break;

//                    case 0x0105:  //飞镖发射口倒计时
//                        memcpy(&referee->RemainingTime, (usart6_receive + DATA), LEN_supply_projectile_booking);
//                        break;

                    case 0x0201:  //比赛机器人状态
                        memcpy(&referee->RobotStatus, (usart6_receive + DATA), LEN_game_robot_state);
                        break;

                    case 0x0202: //实时功率热量数据
                        memcpy(&referee->PowerHeat, (usart6_receive + DATA), LEN_power_heat_data);
                        break;

                    case 0x0203:  //机器人位置
                        memcpy(&referee->RobotPos, (usart6_receive + DATA), LEN_game_robot_pos);
                        break;

//                    case 0x0204:  //机器人增益
//                        memcpy(&referee->Buff, (usart6_receive + DATA), LEN_buff_musk);
//                        break;

//                    case 0x0205:  //空中机器人能量状态
//                        memcpy(&referee->AerialEnergy, (usart6_receive + DATA), LEN_aerial_robot_energy);
//                        break;

                    case 0x0206:  //伤害状态
                        memcpy(&referee->RobotHurt, (usart6_receive + DATA), LEN_robot_hurt);

//                        if(RobotHurt.hurt_type == 0)//非装甲板离线造成伤害
//                        {
//                            Hurt_Data_Update = 1;	   //装甲数据每更新一次则判定为受到一次伤害
//                        }

                        break;

                    case 0x0207:  //实时射击信息
                        memcpy(&referee->ShootData, (usart6_receive + DATA), LEN_shoot_data);
                        break;

                    case 0x0208:  //子弹剩余发射数
                        memcpy(&referee->BulletNum, (usart6_receive + DATA), LEN_bullet_remaining);
                        break;

//                    case 0x0209:  //机器人 RFID 状态
//                        memcpy(&referee->RFIDStatus, (usart6_receive + DATA), LEN_aerial_robot_energy);
//                        break;
                }
            }
        }

        //首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
        if(*(usart6_receive + sizeof(FrameHeader) + CmdIdLen + FrameHeader.DataLength + CRC16Len) == 0xA5)
        {
            //如果一个数据包出现了多帧数据,则再次读取
            Referee_Read_Data(referee, (usart6_receive + sizeof(FrameHeader) + CmdIdLen + FrameHeader.DataLength + CRC16Len));
        }
    }

    return retval_tf;//对数据正误做处理
}







///********************裁判数据辅助判断函数***************************/

///**
//  * @brief          数据是否可用
//  * @param[in]      void
//  * @retval         TRUE可用   FALSE不可用
//  * @attention      在裁判读取函数中实时改变返回值
//  */
//bool_t JUDGE_sGetDataState(void)
//{
//    return Judge_Data_TF;
//}

///**
//  * @brief          统计发弹量
//  * @param[in]      void
//  * @retval         void
//  * @attention
//  */
//portTickType shoot_time;//发射延时测试
//portTickType shoot_ping;//计算出的最终发弹延迟
//float Shoot_Speed_Now = 0;
//float Shoot_Speed_Last = 0;
//uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
//void judge_shoot_num_count(void)
//{
//    Shoot_Speed_Now = REFEREE.ShootData.bullet_speed;

//    if(Shoot_Speed_Last != Shoot_Speed_Now)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
//    {
//        ShootNum++;
//        Shoot_Speed_Last = Shoot_Speed_Now;
//    }

////    shoot_time = xTaskGetTickCount();//获取弹丸发射时的系统时间
////    shoot_ping = shoot_time - REVOL_uiGetRevolTime();//计算延迟
//}


///**
//  * @brief          读取发弹量
//  * @param[in]      void
//  * @retval         发弹量
//  * @attention      不适用于双枪管
//  */
//uint16_t judge_usgetshootnum(void)
//{
//    return ShootNum;
//}


///**
//  * @brief          发弹量清零
//  * @param[in]      void
//  * @retval         void
//  * @attention
//  */
//void judge_shootnum_clear(void)
//{
//    ShootNum = 0;
//}


/* ID: 0x0102  Byte:  4    补给站动作标识 */
uint8_t referee_supply_projectile_id(void) //补给站口 ID：
{
	return (REFEREE.SupplyAction.supply_projectile_id);
}
uint8_t referee_supply_robot_id(void) //补弹机器人 ID：
{
	return (REFEREE.SupplyAction.supply_robot_id);
}
uint8_t referee_supply_projectile_step(void) //出弹口开闭状态：
{
	return (REFEREE.SupplyAction.supply_projectile_step);
}
uint8_t referee_supply_projectile_num(void) //补弹数量：
{
	return (REFEREE.SupplyAction.supply_projectile_num);
}


/* ID: 0X0201  Byte: 15    比赛机器人状态 */
uint8_t referee_robot_id(void) //本机器人 ID
{
	return (REFEREE.RobotStatus.robot_id);
}
uint8_t referee_robot_level(void) //机器人等级
{
	return (REFEREE.RobotStatus.robot_level);
}
uint16_t referee_remain_HP(void) //机器人剩余血量
{
	return (REFEREE.RobotStatus.remain_HP);
}
uint16_t referee_max_HP(void) //机器人上限血量
{
	return (REFEREE.RobotStatus.max_HP);
}
uint16_t referee_shooter_id1_17mm_cooling_rate(void) //机器人 1 号 17mm 枪口每秒冷却值
{
	return (REFEREE.RobotStatus.shooter_id1_17mm_cooling_rate);
}
uint16_t referee_shooter_id1_17mm_cooling_limit(void) //机器人 1 号 17mm 枪口热量上限
{
	return (REFEREE.RobotStatus.shooter_id1_17mm_cooling_limit);
}
uint16_t referee_shooter_id1_17mm_speed_limit(void) //机器人 1 号 17mm 枪口上限速度 单位 m/s
{
	return (REFEREE.RobotStatus.shooter_id1_17mm_speed_limit);
}
uint16_t referee_shooter_id2_17mm_cooling_rate(void) //机器人 2 号 17mm 枪口每秒冷却值
{
	return (REFEREE.RobotStatus.shooter_id2_17mm_cooling_rate);
}
uint16_t referee_shooter_id2_17mm_cooling_limit(void) //机器人 2 号 17mm 枪口热量上限
{
	return (REFEREE.RobotStatus.shooter_id2_17mm_cooling_limit);
}
uint16_t referee_shooter_id2_17mm_speed_limit(void) //机器人 2 号 17mm 枪口上限速度 单位 m/s
{
	return (REFEREE.RobotStatus.shooter_id2_17mm_speed_limit);
}
uint16_t referee_shooter_id1_42mm_cooling_rate(void) //机器人 42mm 枪口每秒冷却值
{
	return (REFEREE.RobotStatus.shooter_id1_42mm_cooling_rate);
}
uint16_t referee_shooter_id1_42mm_cooling_limit(void) //机器人 42mm 枪口热量上限
{
	return (REFEREE.RobotStatus.shooter_id1_42mm_cooling_limit);
}
uint16_t referee_shooter_id1_42mm_speed_limit(void) //机器人 42mm 枪口上限速度 单位 m/s
{
	return (REFEREE.RobotStatus.shooter_id1_42mm_speed_limit);
}
uint16_t referee_chassis_power_limit(void) //机器人底盘功率限制上限
{
	return (REFEREE.RobotStatus.chassis_power_limit);
}

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
uint16_t referee_chassis_volt(void) //底盘输出电压 单位 毫伏
{
	return (REFEREE.PowerHeat.chassis_volt);
}
uint16_t referee_chassis_current(void) //底盘输出电流 单位 毫安
{
	return (REFEREE.PowerHeat.chassis_current);
}
float referee_chassis_power(void) //底盘输出功率 单位 W 瓦
{
	return (REFEREE.PowerHeat.chassis_power);
}
uint16_t referee_chassis_power_buffer(void) //底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
{
	return (REFEREE.PowerHeat.chassis_power_buffer);
}
uint16_t referee_shooter_id1_17mm_cooling_heat(void) //1 号 17mm 枪口热量
{
	return (REFEREE.PowerHeat.shooter_id1_17mm_cooling_heat);
}
uint16_t referee_shooter_id2_17mm_cooling_heat(void) //2 号 17mm 枪口热量
{
	return (REFEREE.PowerHeat.shooter_id2_17mm_cooling_heat);
}
uint16_t referee_shooter_id1_42mm_cooling_heat(void) //42mm 枪口热量
{
	return (REFEREE.PowerHeat.shooter_id1_42mm_cooling_heat);
}


/* ID: 0x0203  Byte: 16    机器人位置数据 */
float referee_yaw(void) //位置枪口，单位度
{
	return (REFEREE.RobotPos.yaw);
}

/* ID: 0x0206  Byte:  1    伤害状态数据 */
uint8_t referee_armor_id(void) //bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0
{
	return (REFEREE.RobotHurt.armor_id);
}
uint8_t referee_hurt_type(void) //bit 4-7：血量变化类型
{
	/*
	0x0 装甲伤害扣血；
	0x1 模块掉线扣血；
	0x2 超射速扣血；
	0x3 超枪口热量扣血；
	0x4 超底盘功率扣血；
	0x5 装甲撞击扣血
	*/
	return (REFEREE.RobotHurt.hurt_type);
}


/* ID: 0x0207  Byte:  6    实时射击数据 */
uint8_t bullet_type(void) //子弹类型: 1：17mm 弹丸 2：42mm 弹丸
{
	return (REFEREE.ShootData.bullet_type);
}
uint8_t shooter_id(void) //发射机构 ID：
{
	return (REFEREE.ShootData.shooter_id);
}
uint8_t bullet_freq(void) //子弹射频 单位 Hz
{
	return (REFEREE.ShootData.bullet_freq);
}
float bullet_speed(void) //子弹射速 单位 m/s
{
	return (REFEREE.ShootData.bullet_speed);
}


/* ID: 0x0208  Byte:  2    实时射击数据 */
uint16_t bullet_remaining_num_17mm(void) //17mm 子弹剩余发射数目
{
	return (REFEREE.BulletNum.bullet_remaining_num_17mm);
}
uint16_t bullet_remaining_num_42mm(void) //42mm 子弹剩余发射数目
{
	return (REFEREE.BulletNum.bullet_remaining_num_42mm);
}
uint16_t coin_remaining_num(void) //剩余金币数量
{
	return (REFEREE.BulletNum.coin_remaining_num);
}

	

