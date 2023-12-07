#include "WirelessCharge_Driver_Layer.h"
#include "WirelessCharge_Interface_Layer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "board_rtc.h"


/* 获取无线充电IC的ID编号并返回：0x1680 */
uint16_t Get_WirelessChargeChip_ID(void)
{
	ret_code_t err_code;
	uint16_t chip_id = 0xFFFF;	
	
	uint8_t rx_buf[2] = {0x00,0x01};			/* chipID的地址在0x00,0x01处 */
	
	/* 先发送写0x00,告诉I2C要读取的地址 */
	err_code = wirelesscharge_i2c_master_write(WirelessCharge_Device_Addr, &rx_buf[0], 1);	
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("Get_WirelessChargeChip_ID write error:%d",err_code);
		return chip_id;
  }
	/* 在发送读,读取0x00地址处的值 */
	err_code = wirelesscharge_i2c_master_read(WirelessCharge_Device_Addr, rx_buf, sizeof(rx_buf));
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("Get_WirelessChargeChip_ID read error:%d",err_code);
		return chip_id;
  }
	
	chip_id = (((uint16_t)rx_buf[0]<<8)|rx_buf[1]);
	//NRF_LOG_INFO("Get_WirelessChargeChip_ID:%d",chip_id);

	return chip_id;
}

/* 获取当前无线充电的电流 */
float Get_WirelessChargeCurrent(void)
{
	ret_code_t err_code;
	float current = 0.0f;	
	
	uint8_t rx_buf[2] = {IOUT_FILT};			/* 要获取的设备的地址 */
	
	/* 先发送写0x30,告诉I2C要读取的地址 */
	err_code = wirelesscharge_i2c_master_write(WirelessCharge_Device_Addr, &rx_buf[0], 1);	
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("Get_WirelessChargeCurrent write error:%d",err_code);
		return current;
  }
	/* 在发送读,读取0x30地址处的值 */
	err_code = wirelesscharge_i2c_master_read(WirelessCharge_Device_Addr, rx_buf, sizeof(rx_buf));
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("Get_WirelessChargeCurrent read error:%d",err_code);
		return current;
  }	
	/* 计算当前充电电流 */
	current = ((((uint16_t)rx_buf[1]<<2)|(rx_buf[0]&0x03))&IOUT_FILT_BIT_MASK)*1.953f;	/* Iout=IOUT_FILT[9:0]*1.953mA */
	NRF_LOG_INFO("Get_WirelessChargeCurrent:" NRF_LOG_FLOAT_MARKER "ma", NRF_LOG_FLOAT(current));

	return current;
}

/* 获取当前VRECT引脚的电压 */
float Get_WirelessChargeVRECT_Voltage(void)
{
	ret_code_t err_code;
	float voltage = 0.0f;	
	
	uint8_t rx_buf[2] = {VRECT_FILT};			/* 要获取的设备的地址 */
	
	/* 先发送要读取的地址 */
	err_code = wirelesscharge_i2c_master_write(WirelessCharge_Device_Addr, &rx_buf[0], 1);	
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("Get_WirelessChargeVRECT_Voltage write error:%d",err_code);
		return voltage;
  }
	/* 读取地址处的值 */
	err_code = wirelesscharge_i2c_master_read(WirelessCharge_Device_Addr, rx_buf, sizeof(rx_buf));
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("Get_WirelessChargeVRECT_Voltage read error:%d",err_code);
		return voltage;
  }	
	/* 计算当前充电电流 */
	voltage = ((((uint16_t)rx_buf[1]<<2)|(rx_buf[0]&0x03))&VRECT_FILT_BIT_MASK)*9.766f;	/* Vrect=VRECT_FILT[9:0]*9.766mV */
	NRF_LOG_INFO("Get_WirelessChargeCurrent:" NRF_LOG_FLOAT_MARKER "mv", NRF_LOG_FLOAT(voltage));

	return voltage;
}

/* 获取当前无线充电电池电压 */
float Get_WirelessChargeVBAT_Voltage(void)
{
	ret_code_t err_code;
	float voltage = 0.0f;	
	
	uint8_t rx_buf[2] = {VBAT_FILT};			/* 要获取的设备的地址 */
	
	/* 先发送要读取的地址 */
	err_code = wirelesscharge_i2c_master_write(WirelessCharge_Device_Addr, &rx_buf[0], 1);	
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("Get_WirelessChargeVBAT_Voltage write error:%d",err_code);
		return voltage;
  }
	/* 读取地址处的值 */
	err_code = wirelesscharge_i2c_master_read(WirelessCharge_Device_Addr, rx_buf, sizeof(rx_buf));
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("Get_WirelessChargeVBAT_Voltage read error:%d",err_code);
		return voltage;
  }	
	/* 计算当前充电电流 */
	voltage = ((((uint16_t)rx_buf[1]<<2)|(rx_buf[0]&0x03))&VBAT_FILT_BIT_MASK)*9.766f;	/* VBAT=VBAT_FILT[9:0]*9.766mV */
	NRF_LOG_INFO("Get_WirelessChargeVBAT_Voltage:" NRF_LOG_FLOAT_MARKER "mv", NRF_LOG_FLOAT(voltage));

	return voltage;
}

/* 强制打开或者关闭无线电源的输出,参数true就是开，flase就是关,成功返回true,失败返回false */
uint8_t ForceWirelessChargeON_OR_OFF(bool is_on)
{
	ret_code_t err_code;
	uint8_t control_cmd = CONTROL;				/* 控制命令 */
	uint8_t rx_buf[2] = {0};							/* 读取的数据 */
	
	/* 先写入要读取的寄存器地址 */
	err_code = wirelesscharge_i2c_master_write(WirelessCharge_Device_Addr, &control_cmd, 1);	
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("ForceWirelessChargeOFF write error:%d",err_code);
		return 1;
  }
	/* 先读出原有的控制寄存器数据 */
	err_code = wirelesscharge_i2c_master_read(WirelessCharge_Device_Addr, &rx_buf[1], 1);
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("ForceWirelessChargeOFF read error:%d",err_code);
		return 1;
  }	

	rx_buf[0] = control_cmd;
	if(is_on){	/* 打开LDO输出与MOSFET */
		rx_buf[1] &= ~(FORCE_LDO_OFF_BIT_MASK|FORECT_RECT_OFF_BIT_MASK);	/* 禁用OFF位 */
		rx_buf[1] |= FORCE_LDO_ON_BIT_MASK|FORECT_RECT_ON_BIT_MASK;		/* 使能启用位 */
	}else{
		rx_buf[1] &= ~(FORCE_LDO_ON_BIT_MASK|FORECT_RECT_ON_BIT_MASK); /* 禁用使能位 */
		rx_buf[1] |= FORCE_LDO_OFF_BIT_MASK|FORECT_RECT_OFF_BIT_MASK;  /* 使能禁用位 */
	}
	rx_buf[1] &= CONTROL_MASK;
	err_code = wirelesscharge_i2c_master_write(WirelessCharge_Device_Addr, rx_buf, 2);	
	if (NRF_SUCCESS != err_code){					/* 通信出错 */
		NRF_LOG_INFO("ForceWirelessChargeOFF write error:%d",err_code);
		return 1;
  }
	
	return 0;
}


/* 一下代码仅用于临时测试 */

APP_TIMER_DEF(wirelesschargechip_timer_id);

/* 这里是定时器到期时要执行的代码 */
void wirelesscharge_timer_handler(void * p_context) 
{
	uint8_t result;
	static uint8_t count = 0;
	count++;
	
	if(count>=2){
		count = 0;
		result = ForceWirelessChargeON_OR_OFF(false);
		NRF_LOG_INFO("ForceWirelessChargeOFF write error:%d",result);
	}else
	{
		result = ForceWirelessChargeON_OR_OFF(true);
		NRF_LOG_INFO("ForceWirelessChargeON write error:%d",result);
	}
	Get_WirelessChargeCurrent();
}

/* 这是用于单独测试的程序 */
void WirelessChargeChip_Test(void)
{
	wirelesscharge_i2c_master_init();
	
	app_timer_create(&wirelesschargechip_timer_id, APP_TIMER_MODE_REPEATED, wirelesscharge_timer_handler);
	app_timer_start(wirelesschargechip_timer_id, APP_TIMER_TICKS(10000), NULL);
}
