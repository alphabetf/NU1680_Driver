#include "WirelessCharge_Interface_Layer.h"

/* I2C写 */
uint8_t wirelesscharge_i2c_master_write(uint8_t device_addr, const uint8_t* tx_buf, uint8_t tx_size)
{
	return hal_i2c_master_write(device_addr, tx_buf, tx_size, false);
}

/* I2C读 */
uint8_t wirelesscharge_i2c_master_read(uint8_t device_addr, uint8_t* rx_buf, uint8_t rx_buf_size)
{
	return hal_i2c_master_read(device_addr, rx_buf, rx_buf_size);
}

/* I2C初始化 */
void wirelesscharge_i2c_master_init(void)
{
	hal_i2c_master_init();
}

