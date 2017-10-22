/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-08
*
*/

#include "HPTG_i2c.h"
#include "laser_log.h"


/** @brief laura indirect address write
*
*	@param dev_t the laser focus controller
*	@param i_reg_addr_lsb the lsb register address which store lsb indirect address
*	@param i_reg_addr_msb the msb register address which store msb indirect address
*	@param indirect_addr the indirect address
*	@param register_addr the register address which store data
*	@param i2c_write_data the write data
*	@param num_word the size of write data
*
*/
int Laura_device_indirect_addr_write(struct msm_laser_focus_ctrl_t *dev_t, uint16_t indirect_addr,
														uint16_t *i2c_write_data, uint32_t num_word)
{
	int status;
	int i;

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = CCI_I2C_WrWord(dev_t, 0x18, indirect_addr);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: write indirect address %x failed\n", __func__, indirect_addr);
		return status;
	}

	for(i = 0; i < num_word; i++) {
		status = CCI_I2C_WrWord(dev_t, I2C_DATA_PORT, i2c_write_data[i]);
		if (status < 0) {
			LOG_Handler(LOG_ERR, "%s: write data to I2C_DATA_PORT(0x1A) failed at index %d\n", __func__, i);
			return status;
		}		
	}

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

