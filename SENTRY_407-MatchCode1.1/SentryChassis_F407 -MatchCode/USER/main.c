#include "main.h"
#include "SysInit.h"
#include "Task_Start.h"
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"

/************************** Dongguan-University of Technology -ACE**************************
 *@project   SentryChassis
 *@Author    Dongguan-University of Technology  ACE  刘沅东edddddge
 *@Date      2021-03-01
                                                                                                    
                    P                          :u7  :Ii              .                              
                   QBQ                     sQBBQBB  PBBBBQI.        XQBBBBBBBBBQBBBBBBBBBBBBM       
                  bBBBZ                 .MQBQBBBQB  5BBBBBBBBi      uBBBBBBBBBQBBBBBBBBBQBBBP       
                 bBBQQB5               XBBBRQQBBBP  sQBQBQQBBBZ     IBBBBBBBBBBBBBBBBBBBBBBBD       
                 rBBgRQBY             BBQQRBBQr        rgBBBQr                                      
               .  iBBgRQB7           BBQRgBQ:            iE.                                        
              :BY  7BBgRQB:         sBQMgBB                                                         
             .BBB:  uBBgRBB.        BBMDQQ:                         rSU57  UQPdPbPPPPqPPbPdQs       
             BBQBB:  XBQgRBB        QBggQB                          sBEQ1  QBBBBQBBBBBBBBBBBZ       
            BBQgBBB   KBRDRBB       BBgDBB                          jBDQU  QBBBBBBBBBBBQBBBBg       
           BBQgRBB     dQggQBB      BBggQB.                         iXJS7  uDK5XXK5KXKXXSSXg7       
          gBQgRQB   BBggQDggQBQ     YBQDMBB                                                         
         PBQgRBB   BBBBBRQgMgQBg     BBQgRBB:            iZ:                                        
        2BQgMBB.  BBBBBBBBBQRgQBK     BBBRQBBQL.      .rRBBQBr       ..                   ..        
       vQBgRQB:  :uriiiiiirBQQgBB1     XQBQQQBBBBE  uBQBQBQBBBD     SBBBBBBBBBBBBBBBBBBBQBBBD       
      7QBQBBBr             :BBBQBBY     .ZBQBBBBBB  qBBQBBBBB:      UBBBBQBBBBBBBBBBBBBBBBBBd       
     LBBBBBBJ               7BBBBBQu       YRBBBQB  KBBBBBJ.        IBQBBBBBQBBBBBBBBBBBBBBBZ       
                                                7i  .7.                                             
*************************** Dongguan-University of Technology -ACE**************************/
int main(void)
{
	System_Init();          //系统初始化
	StartTask();            //开始任务
	vTaskStartScheduler();  //开启任务调度

    while (1)
	{
//		CAN1_Chassis_Gimbal_Fire(1000, 1000, 1000);
//		CAN1_Chassis_SetMsg(500, 500);
//		delay_ms(500);
//		CAN1_Chassis_SetMsg(-500,-500);
	}
}
