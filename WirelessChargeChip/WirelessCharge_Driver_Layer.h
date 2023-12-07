#ifndef __WIRELESSCHARGE_DRIVER_LAYER__
#define __WIRELESSCHARGE_DRIVER_LAYER__

#include <math.h>  
#include <stdint.h>
#include <stdio.h>
#include <nrfx.h>

#define WirelessCharge_Device_Addr		0x60		  /* 无线充电IC的设备地址 */
#define WRITE_BIT 										0         /* I2C写 */
#define READ_BIT 											1         /* I2C读 */
					
#define CHIP_ID_L	  									0x00			/* 芯片ID，默认值:0x1680,2字节 */
#define CHIP_ID_H											0x01

/* General Purpose Registers */
#define CURRENT_STATE									0x03			/* 当前状态,只读寄存器 */
#define LDO_ON_BIT_POS 								0		 			/* 0:LDO电源关闭,1:LDO电源打开 */
#define LDO_ON_BIT_MASK 						 	(0x1UL << LDO_ON_BIT_POS) 	
#define RECT_ON_BIT_POS 							1		 			/* 0:整流器MOSFET禁用,1:整流器MOSFET使能 */
#define REC_ON_BIT_MASK 							(0x1UL << RECT_ON_BIT_POS) 	
#define OCP_BIT_POS 									3		 			/* 0:无过电流发生,1:发生过流 */
#define OCP_BIT_MASK 						 		 	(0x1UL << OCP_BIT_POS) 	
#define OVP2_BIT_POS 									4		 			/* 0:VRECT未发生高电平电压,1:VRECT高电平电压发生 */
#define OVP2_BIT_MASK 						 	 	(0x1UL << OVP2_BIT_POS) 	
#define OVP1_BIT_POS 									5		 			/* 0:VRECT未发生低电平电压,1:VRECT处低电平电压发生 */
#define OVP1_BIT_MASK 						 	 	(0x1UL << OVP1_BIT_POS)
#define OPT_BIT_POS 									6		 			/* 0:IC节点未发生过温现象,1:IC节点发生过温 */
#define OPT_BIT_MASK 						 	  	(0x1UL << OPT_BIT_POS)
#define TEMP_BIT_POS 									7		 			/* 0:TEMP/VBAT均未发生过温情况,1:TEMP/VBAT发生过热 */
#define TEMP_BIT_MASK 						 	  (0x1UL << TEMP_BIT_POS)

#define CONTROL												0x04			/* 控制寄存器,读/写 */
#define CONTROL_MASK									0x1F			/* 控制寄存器位数 */
#define FORCE_LDO_ON_BIT_POS 					0		 			/* 0:正常功率LDO开关操作,1:强制打开电源LDO */
#define FORCE_LDO_ON_BIT_MASK 				(0x1UL << FORCE_LDO_ON_BIT_POS)
#define FORECT_RECT_ON_BIT_POS 				1		 			/* 0:普通整流MOSFET开/关操作,1:任何负载下使能整流器MOSFET */
#define FORECT_RECT_ON_BIT_MASK 		  (0x1UL << FORECT_RECT_ON_BIT_POS)
#define FORCE_LDO_OFF_BIT_POS 				2		 			/* 0:正常功率LDO开关操作,1:强制关闭电源LDO */
#define FORCE_LDO_OFF_BIT_MASK 			  (0x1UL << FORCE_LDO_OFF_BIT_POS)
#define FORECT_RECT_OFF_BIT_POS 			3		 			/* 0:整流器MOSFET的开/关取决于FORCE_RECT_ON,1:强制关闭所有四个整流器MOSFET,无论其他条件如何 */
#define FORECT_RECT_OFF_BIT_MASK 		  (0x1UL << FORECT_RECT_OFF_BIT_POS)
#define FORCE_VBAT_TRK_OFF_BIT_POS 		4		 			/* 0:跟踪VBAT功能取决于MTP_OPTION中的MTP_VBAT_TRK_EN,1:关闭跟踪功能,无论MTP_VBAT_TRK_EN如何。 */
#define FORCE_VBAT_TRK_OFF_BIT_MASK   (0x1UL << FORCE_VBAT_TRK_OFF_BIT_POS)

#define AP_EPT												0x05			/* EPT控制寄存器,读/写 */
#define EPT_MESSAGE_BIT_POS 					0		 			/* EPT数据包中包含的消息内容 */
#define EPT_MESSAGE_BIT_MASK 				  (0x7FUL << EPT_MESSAGE_BIT_POS)
#define AP_EPT_EN_BIT_POS 						7		 			/* 0:禁用包括消息EPT_MESSAGE放入EPT包中,1:启用,包括消息EPT_MESSAGE放入EPT包中 */
#define AP_EPT_EN_BIT_MASK 					  (0x1UL << AP_EPT_EN_BIT_POS)

/* 中断和保护事件标志寄存器,如果AP/MCU收到中断信号,则先读取该寄存器,然后再对其他寄存器进行操作,否则该字节将被清除 */
#define INT_FLAG 										 	0x06			/* 中断和保护事件标志寄存器,读 */
#define STARTUP_FLAG_BIT_POS 					1		 			/* V5V上升至UVLO后,该位将被设置,读取它会清除该位 */
#define STARTUP_FLAG_BIT_MASK 			  (0x1UL << STARTUP_FLAG_BIT_POS)
#define OCP_FLAG_BIT_POS 							3		 			/* OCP事件设置该位,将EPT发送到Tx,读取它会清除该位 */
#define OCP_FLAG_BIT_MASK 				 	  (0x1UL << OCP_FLAG_BIT_POS)
#define OVP2_FLAG_BIT_POS 						4		 			/* OVP2事件设置该位,发送EPT到Tx,读取它会清除该位 */
#define OVP2_FLAG_BIT_MASK 				 	  (0x1UL << OVP2_FLAG_BIT_POS)
#define OVP1_FLAG_BIT_POS 						5		 			/* OVP1件设置该位,如果MTP_OVP1_EPT_EN=1,则将EPT发送到Tx,读取它会清除该位 */
#define OVP1_FLAG_BIT_MASK 				    (0x1UL << OVP1_FLAG_BIT_POS)
#define OTP_FLAG_BIT_POS 							6		 			/* OTP(IC芯片过热保护)事件设置该位,将EPT发送到Tx,读取它会清除该位 */
#define OTP_FLAG_BIT_MASK 				 	  (0x1UL << OTP_FLAG_BIT_POS)
#define TEMP_FLAG_BIT_POS 						7		 			/* TEMP/VBAT(感测外部组件)过温事件设置该位,将EPT发送到Tx,读取它会清除该位 */
#define TEMP_FLAG_BIT_MASK 				 	  (0x1UL << TEMP_FLAG_BIT_POS)

/* Parameter Configure Registers */
#define MFG_CODE_H										0x10			/* 制造信息高字节,读/写 */
#define MFG_CODE_L										0x11			/* 制造信息低字节,读/写 */
#define DEVICE_ID_B6									0x12			/* 设备ID信息,读/写 */
#define INFO1_LOCK										0x13			/* OTP程序的锁定位,读/写 */
#define INFO1_LOCK_BIT_POS 						0		 			/* OTP程序的锁定位 */
#define INFO1_LOCK_BIT_MASK 				 	(0x1UL << INFO1_LOCK_BIT_POS)

#define MTP_ACR												0x14			/* LC谐振回路等效电阻(ACR)为FOD参数,ACR=参考设计工具,读/写 */
#define MTP_ACR_BIT_POS 							0		 			
#define MTP_ACR_BIT_MASK 				 		 	(0x1FUL << MTP_ACR_BIT_POS)

#define MTP_OPTION										0x15 /* 读/写 */
#define MTP_VBAT_TRK_EN_BIT_POS 			0		 /* 0:禁用输出跟踪VBAT功能,1:使能输出跟踪VBAT功能 */			
#define MTP_VBAT_TRK_EN_BIT_MASK 		 	(0x1UL << MTP_VBAT_TRK_EN_BIT_POS)
#define MTP_TEMP_EPT_EN_BIT_POS 			1		/* 0:禁用发送EPT TEMP/VBAT超温,1:启用发送EPT TEMP/VBAT过温 */	 			
#define MTP_TEMP_EPT_EN_BIT_MASK 		 	(0x1UL << MTP_TEMP_EPT_EN_BIT_POS)
#define MTP_TEMP_LOW_EN_BIT_POS 			2		/* 0:禁用低温保护,1: 使能低温保护 */	
#define MTP_TEMP_LOW_EN_BIT_MASK 		  (0x1UL << MTP_TEMP_LOW_EN_BIT_POS)
#define MTP_OVP1_EPT_EN_BIT_POS 			3		/* 0:OVP1发生时禁止发送EPT,并更快地发送CE,1:使能发送EPT */	
#define MTP_OVP1_EPT_EN_BIT_MASK 		  (0x1UL << MTP_OVP1_EPT_EN_BIT_POS)
#define MTP_CE_LARGE_BIT_POS 					4		/* 0:定义CE=0(VRECT-VRECT_Targe)在[+40mV,-40mV]之间,1:定义CE=0(VRECT-VRECT_Target)在[+80mV,-40mV]之间 */	
#define MTP_CE_LARGE_BIT_MASK 			  (0x1UL << MTP_CE_LARGE_BIT_POS)

#define MTP_VBAT_DELTA								0x16 /* 读/写 */
#define MTP_VBAT_DELTA_BIT_POS 				0		 /* Vout和TEMP/VBAT引脚电压之间的差值,00:500毫伏,01:400毫伏,10:300毫伏,11:600毫伏 */			
#define MTP_VBAT_DELTA_BIT_MASK 		  (0x3UL << MTP_VBAT_DELTA_BIT_POS)

#define MTP_VBAT_LOWLMT								0x17 /* 读/写 */
#define MTP_VBAT_LOWLMT_BIT_POS 			0		 /* 使用跟踪VBAT功能时Vout的最小限制,00:4.5V,01:4.3V,10:4.1V,11:4.7V */			
#define MTP_VBAT_LOWLMT_BIT_MASK 		  (0x3UL << MTP_VBAT_LOWLMT_BIT_POS)

#define MTP_OFFSET										0x18 /* 读/写 */
#define MTP_OFFSET_BIT_POS 						0		 /* FOD参数的接收功率偏移 */			
#define MTP_OFFSET_BIT_MASK 		 		  (0xFUL << MTP_OFFSET_BIT_POS)

#define MTP_DUMMY										  0x1C /* 读/写 */
#define MTP_DUMMY_BIT_POS 						0		 /* 负载#0处的假负载,Dummy=MTP_DUMMY[3:0]*3.5mA */			
#define MTP_DUMMY_BIT_MASK 		 		    (0xFUL << MTP_DUMMY_BIT_POS)

#define MTP_VOUT_SET									0x1D /* 读/写,Vout输出设置,Vout=MTP_VOUT_SET[7:0]*39.06mV */

#define MTP_ILIM_SET									0x1E /* 读/写 */
#define MTP_ILIM_SET_BIT_POS 					0		 /* 过流保护极限,000:1.4A,001:1.65A,010:1.1A,011:0.74A,100:0.365A,101:0.45A,110:0.29A,111:0.215A */			
#define MTP_ILIM_SET_BIT_MASK 		 	  (0x7UL << MTP_ILIM_SET_BIT_POS)

#define MTP_TEMP_TH									  0x1F /* 读/写 */
#define MTP_TEMP_TH_BIT_POS 					0		 /* 如果将此引脚配置为温度传感,请将R25=100K,beta=4250NTC连接到TEMP/VBAT的引脚,00:80°C,01:60°C,10:50℃,11:42°C */			
#define MTP_TEMP_TH_BIT_MASK 		 		  (0x3UL << MTP_TEMP_TH_BIT_POS)

#define MTP_VDELTA 									  0x21 /* 读/写 */
#define MTP_VDELTA_BIT_POS 						0		 /* 设置负载#2处的Vrect和Vout之间的差值,00:200毫伏,01:280mV,10:360毫伏,11:150mV */			
#define MTP_VDELTA_BIT_MASK 		 		  (0x3UL << MTP_VDELTA_BIT_POS)

/* 1,设置负载#0和#1处的Vrect和Vout之间的附加差值,所以Vrect=Vout+MTP_VDELTA+MTP_VLIGHT
00:2.00V(负载#0),1.00V(负载#1),01:2.50V(负载#0),1.25V(负载#1)
10:1.00V(负载#0),0.50V(负载#1),11:0.50V(负载#0),0.25V(负载#1)
2,负载状态区域定义的阈值和迟滞
00:负载#0到负载#1:50mA,迟滞10mA;负载#1到负载#2:100mA,迟滞20mA；
01:负载#0到负载#1:80mA,迟滞16mA;负载#1到负载#2:160mA,迟滞32mA；
10:负载#0到负载#1:100mA,迟滞20mA;负载#1到负载#2:200mA,迟滞40mA；
11:负载#0到负载#1:40mA,迟滞8mA;负载#1至负载#2:80mA,迟滞16mA */
#define MTP_VLIGHT 									0x22 /* 读/写 */
#define MTP_VLIGHT_BIT_POS 					0		 		
#define MTP_VLIGHT_BIT_MASK 		 		(0x3UL << MTP_VLIGHT_BIT_POS)

#define MTP_CE_LIMIT   							0x23 /* 控制误差的最大极限,读/写 */
#define MTP_CE_LIMIT_BIT_POS 				0		 /* 00:50,01:30,10:12,11:100 */
#define MTP_CE_LIMIT_BIT_MASK 		 	(0x3UL << MTP_CE_LIMIT_BIT_POS)

#define INFO2_LOCK   								0x24 /* MTP1程序的锁定位,读/写 */
#define INFO2_LOCK_BIT_POS 					0	
#define INFO2_LOCK_BIT_MASK 		 		(0x1UL << INFO2_LOCK_BIT_POS)

#define INFO3_LOCK    							0x25 /* MTP2程序的锁定位,读/写 */
#define INFO3_LOCK_BIT_POS 					0	
#define INFO3_LOCK_BIT_MASK 		 		(0x1UL << INFO3_LOCK_BIT_POS)

#define I2C_OTP_CTRL     						0x72 /* ,读/写,程序密码,仅在测试模式下可用,0x3D:一次性密码,0x3E:MTP1,0x3F:MTP2 */

/* ADC Channel Registers */
#define IOUT_FILT    								0x30 /* 读,Iout电流,Iout=IOUT_FILT[9:0]*1.953mA */
#define IOUT_FILT_BIT_POS 					0		 /* 连续读2字节 */
#define IOUT_FILT_BIT_MASK 		 	 	 	(0x1FFUL << IOUT_FILT_BIT_POS)

#define VBAT_FILT    								0x32 /* 读,TEMP/VBAT电压,VBAT=VBAT_FILT[9:0]*9.766mV */
#define VBAT_FILT_BIT_POS 					0		 /* 连续读2字节 */
#define VBAT_FILT_BIT_MASK 		 	 	  (0x1FFUL << VBAT_FILT_BIT_POS)

#define VRECT_FILT    							0x34 /* 读,Vrect电压,Vrect=VRECT_FILT[9:0]*9.766mV */
#define VRECT_FILT_BIT_POS 					0		 /* 连续读2字节 */
#define VRECT_FILT_BIT_MASK 		 	  (0x1FFUL << VRECT_FILT_BIT_POS)

#define TEMP_CONV    								0x38 /* 读,NTC电阻,内部4uA通过TEMP/VBAT引脚,RNTC=TEMP_CONV[9:0]*0.488KΩ */
#define TEMP_CONV_BIT_POS 					0		 /* 连续读2字节 */
#define TEMP_CONV_BIT_MASK 		 	 	  (0x1FFUL << TEMP_CONV_BIT_POS)

#define INTB_CONV    								0x3A /* 读,配置启动期间通过OS1引脚的内部4uA负载#0处的FOD偏移,Code_OFFSET=0.128*R_INT_B/OS1(KΩ) */
#define INTB_CONV_BIT_POS 					0		 /* 连续读2字节 */
#define INTB_CONV_BIT_MASK 		 	 	  (0x1FFUL << INTB_CONV_BIT_POS)

#define SDA_CONV    								0x3C /* 读,配置启动期间通过SDA引脚的内部4uA负载#1和#2处的FOD偏移,Code_OFFSET=0.128*R_SDA(KΩ) */
#define SDA_CONV_BIT_POS 						0		 /* 连续读2字节 */
#define SDA_CONV_BIT_MASK 		 	 	  (0x1FFUL << SDA_CONV_BIT_POS)

#define SCL_ACR_CONV    						0x3E /* 读,在启动期间通过SCL/ACR引脚配置FOD的ACR,内部4uA电流,Code_ACR=0.256*R_SCL/ACR(KΩ) */
#define SCL_ACR_CONV_BIT_POS 				0		 /* 连续读2字节 */
#define SCL_ACR_CONV_BIT_MASK 		  (0x1FFUL << SCL_ACR_CONV_BIT_POS)

#ifdef __cplusplus
extern "C" {
#endif

uint16_t Get_WirelessChargeChip_ID(void);
void WirelessChargeChip_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIRELESSCHARGE_DRIVER_LAYER__ */

