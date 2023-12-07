#ifndef __WIRELESSCHARGE_INTERFACE_LAYER__
#define __WIRELESSCHARGE_INTERFACE_LAYER__

#include "HAL_I2C_Master.h"
#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t wirelesscharge_i2c_master_write(uint8_t device_addr, const uint8_t* tx_buf, uint8_t tx_size);
uint8_t wirelesscharge_i2c_master_read(uint8_t device_addr, uint8_t* rx_buf, uint8_t rx_buf_size);
void wirelesscharge_i2c_master_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIRELESSCHARGE_INTERFACE_LAYER__ */

