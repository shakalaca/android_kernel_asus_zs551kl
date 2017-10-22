/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "laser_focus_i2c.h"
#include "laser_log.h"

/** @brief Write one byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the data which will be written
*
*/
#define CCI_DELAY	5
#define I2C_RETRY_COUNT 10


int APPS_I2C_RxData(struct msm_laser_focus_ctrl_t *dev_t, uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	uint8_t subaddr[1];

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = subaddr,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};
	subaddr[0] = cmd;

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(dev_t->apps_i2c_client->adapter, msg, 2) > 0)
			break;

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		LOG_Handler(LOG_ERR, "%s retry over %d\n", __func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0; 
}

int APPS_I2C_TxData(struct msm_laser_focus_ctrl_t *dev_t, uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(dev_t->apps_i2c_client->adapter, msg, 1) > 0)
			break;

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		LOG_Handler(LOG_ERR, "%s retry over %d\n", __func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}


int CCI_I2C_WrByte(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t i2c_write_data)
{
	uint8_t buffer[2];
	int ret = 0;

	/* Debug use */
	LOG_Handler(LOG_REG, "%s: [0x%x, 0x%02x, 0x%02x]\n", __func__, dev_t->slaveAddr, register_addr, i2c_write_data);

	buffer[0] = (uint8_t)(register_addr & 0xff);
	buffer[1] = (uint8_t)(i2c_write_data & 0xff);

	ret = APPS_I2C_TxData(dev_t, dev_t->slaveAddr, buffer, 2);
	if (ret < 0) {
		LOG_Handler(LOG_ERR, "%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

/** @brief Write one word via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the data which will be written
*
*/
int CCI_I2C_WrWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t write_data)
{
	uint8_t buffer[3];
	int ret = 0;

	/* Debug use */
	LOG_Handler(LOG_REG, "%s: [0x%x, 0x%02x, 0x%04x]\n", __func__, dev_t->slaveAddr, register_addr, write_data);

	buffer[0] = (uint8_t)(register_addr & 0xff);
	buffer[1] = (uint8_t)(write_data & 0xff);
	buffer[2] = (uint8_t)((write_data & 0xff00) >> 8);

	ret = APPS_I2C_TxData(dev_t, dev_t->slaveAddr, buffer, 3);
	if (ret < 0) {
		LOG_Handler(LOG_ERR, "%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

#if 0
/** @brief Write seqence byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the value which will be written
*	@param num_bytes the size of write data
*
*/
int CCI_I2C_WrByteSeq(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint8_t *i2c_write_data, uint32_t num_byte)
{
	int status;
	struct msm_camera_i2c_client *sensor_i2c_client;

	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write_seq(sensor_i2c_client, register_addr, 
		i2c_write_data, num_byte);

	if (status < 0) 
		LOG_Handler(LOG_ERR, "%s: write register(0x%x) failed\n", __func__, register_addr);

	return status;
}
#endif /* Vincent Remove cci */


/** @brief Read one byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the variable which will be assigned read result
*
*/
int CCI_I2C_RdByte(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t *i2c_read_data)
{
	uint8_t buffer[1];
	int ret = 0;
#ifdef VINCENT_DEBUG
	struct timeval start,now;
	int elapse_time;

	O_get_current_time(&start);
#endif

	if (i2c_read_data == NULL)
		return -EFAULT;

	ret = APPS_I2C_RxData(dev_t, dev_t->slaveAddr, register_addr, buffer, 1);
	if (ret < 0) {
		LOG_Handler(LOG_ERR, "%s: I2C_RxData fail [0x%x, 0x%x]\n", __func__, dev_t->slaveAddr, register_addr);
		return ret;
	}

	*i2c_read_data = buffer[0];

#ifdef VINCENT_DEBUG
	O_get_current_time(&now);
	DeltaTime_ms(start, now, &elapse_time);

	LOG_Handler(LOG_REG, "%s: I2C_RxData[0x%x, 0x%x] = 0x%x, elapse time = %dms\n", 
						__func__, dev_t->slaveAddr, register_addr, *i2c_read_data, elapse_time);
#endif

	return ret;
}

/** @brief Read one word via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_write_data the variable which will be assigned read result
*
*/
int CCI_I2C_RdWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint16_t *i2c_read_data)
{
	uint8_t buffer[2];
	int ret = 0;
#ifdef VINCENT_DEBUG	
	int elapse_time;
	struct timeval start,now;
	O_get_current_time(&start);	
#endif

	if (i2c_read_data == NULL)
		return -EFAULT;

	ret = APPS_I2C_RxData(dev_t, dev_t->slaveAddr, register_addr, buffer, 2);
	if (ret < 0) {
		LOG_Handler(LOG_ERR, "%s: I2C_RxData fail [0x%x, 0x%2x]\n", __func__, dev_t->slaveAddr, register_addr);
		return ret;
	}

	*i2c_read_data = (buffer[1] << 8) | buffer[0];
	//swap_data(i2c_read_data);

#ifdef VINCENT_DEBUG
	/* Debug use */
	O_get_current_time(&now);
	DeltaTime_ms(start, now, &elapse_time);
	LOG_Handler(LOG_REG, "%s: I2C_RxData[0x%x, 0x%x] = 0x%x, elapse time = %dms\n", 
						__func__, dev_t->slaveAddr, register_addr, *i2c_read_data, elapse_time);
#endif

	return ret;
}

#if 0
/** @brief Read two words via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_read_data the variable which will be assigned read result
*	@param num_byte number of the bytes which will be read
*
*/
int CCI_I2C_RdDWord(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint32_t *i2c_read_data)
{
	int status;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	struct msm_camera_i2c_client *sensor_i2c_client;

	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	reg_setting.reg_data_size = 4;

	status = (int)sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		reg_setting.reg_data, reg_setting.reg_data_size);
	
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	
	*i2c_read_data=((uint32_t)reg_setting.reg_data[0]<<24)|((uint32_t)reg_setting.reg_data[1]<<16)|((uint32_t)reg_setting.reg_data[2]<<8)|((uint32_t)reg_setting.reg_data[3]);
	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	return status;
}

/** @brief Read seqence byte via CCI i2c
*	
*	@param dev_t the laser focus controller
*	@param register_addr the register address
*	@param i2c_read_data the variable which will be assigned read result
*	@param num_bytes the size of read data
*
*/
int CCI_I2C_RdByteSeq(struct msm_laser_focus_ctrl_t *dev_t, uint32_t register_addr, uint8_t *i2c_read_data, uint32_t num_byte)
{
	int status;
	uint8_t buf[num_byte];
	struct msm_camera_i2c_client *sensor_i2c_client;
	
	sensor_i2c_client = dev_t->i2c_client;
	if (!sensor_i2c_client) {
		LOG_Handler(LOG_ERR, "%s: failed: %p \n", __func__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		buf, num_byte);
	
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	LOG_Handler(LOG_REG, "%s: read register(0x%x) : 0x%x \n", __func__, register_addr, i2c_read_data);

	return status;
}
#endif /* Vincent Remove cci */


