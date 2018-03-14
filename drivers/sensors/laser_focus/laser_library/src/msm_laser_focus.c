/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/
//#include "Laura_shipping_func.h"
#include "msm_laser_focus.h"
#include "laser_log.h"
#include "HPTG_debug.h"
#include "HPTG_interface.h"
#include "HPTG_factory_func.h"
#include "HPTG_shipping_func.h"
#include "laser_focus_hepta.h"
#include <linux/of.h>
#include <linux/of_gpio.h>
#include "laser_focus_hepta.h"
#include "HPTG_shipping_func.h"
#include <linux/regulator/consumer.h>
#include <linux/pm.h>

int g_factory = 0;			//default for shipping
bool timedMeasure = true; //default for shipping
bool dump0x09 = false;
bool CSCmode = false;
uint16_t Laser_log_cnt=0;

int Laser_Product = PRODUCT_UNKNOWN;
int FirmWare;
extern uint16_t module_id[34];

extern struct msm_laser_focus_ctrl_t *laura_t;

//Error table
extern uint16_t thd;
extern uint16_t limit;
extern uint8_t thd_near_mm;

#define proc(file,mod,fop)	\
	LOG_Handler(LOG_DBG,"proc %s %s\n",file,proc_create(file,mod,NULL,fop)? "success":"fail");

int IDexistance(struct msm_laser_focus_ctrl_t *dev_t){

	if (!dev_t) {
		LOG_Handler(LOG_ERR, "%s: failed: %p\n", __func__, dev_t);
		return -EINVAL;
	}
	if (!(dev_t->apps_i2c_client && dev_t->sensor_name)) {
		LOG_Handler(LOG_ERR, "%s: failed: %p %p\n", __func__, 
			dev_t->apps_i2c_client, 
			dev_t->sensor_name);
		return -EINVAL;
	}
	return 0;
}
extern uint16_t chipID;


int VerifyID(struct msm_laser_focus_ctrl_t *dev_t){

	int rc = 0;
	uint16_t chip_id = 0;

	// Get ID
	rc = CCI_I2C_RdWord(dev_t, 0x28, &chip_id);	
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: %s read id failed\n", __func__, dev_t->sensor_name);
		return rc;
	}

	chipID=chip_id;
	
	// Verify ID,  &&0x01AD for HEPT and wont disturb ST
	LOG_Handler(LOG_CDBG, "%s: read id: 0x%x expected id 0x%x\n", __func__, chip_id, dev_t->sensor_id);	
	if (chip_id != dev_t->sensor_id && (chip_id != 0xAD02)) {
		LOG_Handler(LOG_ERR, "%s: chip id doesnot match\n", __func__);
		return -ENODEV;
	}
	return rc;
}

int Laser_Match_ID(struct msm_laser_focus_ctrl_t *dev_t)
{
	int rc = 0;

	rc = IDexistance(dev_t);
	if(rc==0)
		rc = VerifyID(dev_t);
	
	return rc;
}

/** @brief Power on component
*	
*	@param dev_t the laser focus controller
*
*/
int32_t PowerUp(struct msm_laser_focus_ctrl_t *dev_t)
{
	int rc = 0;
	int err = 0;

	err = regulator_set_load(dev_t->reg, LASER_MEASURE_CURRENT);
	if (err < 0) {
		dev_err(&dev_t->apps_i2c_client->dev, "Failed to set opt mode for vdd reg %d\n", err);
		return err;
	}	
	
	dev_t->laser_focus_state = LASER_FOCUS_POWER_UP;

	/* Set GPIO vdig to high for sku4 */
#if 0
	GPIO_Handler(dev_t, SENSOR_GPIO_VDIG, GPIO_HIGH);
#endif

	
	return rc;
}


/** @brief Power off component
*	
*	@param dev_t the laser focus controller
*
*/
int32_t PowerDown(struct msm_laser_focus_ctrl_t *dev_t)
{
	int32_t rc = 0;
	int err = 0;

	/* Check device status */
	if (dev_t->laser_focus_state != LASER_FOCUS_POWER_DOWN) {
		err = regulator_set_load(dev_t->reg, LASER_STANDBY_CURRENT);
		if (err < 0) {
			dev_err(&dev_t->apps_i2c_client->dev, "Failed to set opt mode for vdd reg %d\n", err);
			return err;
		}
		dev_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	}

	/* Set GPIO vdig to low for sku4 */
#if 0
	//GPIO_Handler(dev_t, SENSOR_GPIO_VDIG, GPIO_LOW);
#endif

	return rc;
}


#define CPE_LEN	5
#define PRODUCT_VERSION	5

void Product_Family(struct msm_laser_focus_ctrl_t *dev_t){
	char Laura[CPE_LEN] = {'0', 'M', 'G', 'B', 'X'};
	char Olivia[CPE_LEN] = {'0', 'M', 'L', 'A', 'X'};
	int PV,i;

	PV = module_id[PRODUCT_VERSION];

	for(i=1; i < CPE_LEN; i++)
		if(module_id[i] != Olivia[i])
			break;
			
	if(i == CPE_LEN){
		Laser_Product = PRODUCT_OLIVIA;		
		if(PV == 0)
			FirmWare = Laser_Product;
		else 
			LOG_Handler(LOG_DBG, "%s: unknown FW version: %d\n", __func__, PV);		
	}
	
	for(i=1; i < CPE_LEN; i++)
		if(module_id[i] != Laura[i])
			break;		

	if(i==CPE_LEN){
		Laser_Product = PRODUCT_LAURA;				
		if(PV == 0)
			FirmWare = Laser_Product;
		else if(PV == 2)
			LOG_Handler(LOG_CDBG, "%s: FW version is OLIVIA\n", __func__, PV);
		else
			LOG_Handler(LOG_ERR, "%s: unknown FW version: %d\n", __func__, PV);
			
	}


}

extern void Get_ModuleID(struct msm_laser_focus_ctrl_t *dev_t);

void HPTG_Match_Module(struct msm_laser_focus_ctrl_t *dev_t){

	Get_ModuleID(dev_t);
	
	Product_Family(dev_t);
	
	LOG_Handler(LOG_CDBG, "%s: %d\n", __func__, Laser_Product);
}


/** @brief Check device status
*	
*	@param dev_t the laser focus controller
*
*/
int HPTG_I2C_status_check(struct msm_laser_focus_ctrl_t *dev_t){
	int32_t rc=0;

	PowerUp(dev_t);
	rc = Laser_Match_ID(dev_t);
	if (rc < 0) {
		LOG_Handler(LOG_ERR,"%s fail, rc (%d)\n", __func__, rc);
		PowerDown(dev_t);
		return I2C_STATUS_FAIL;
	}

	HPTG_Match_Module(dev_t);
	PowerDown(dev_t);
	
	if(rc < 0)
		return I2C_STATUS_FAIL;
	else
		return PRODUCER_HPTG;
}

/** @brief Mutex controller handles mutex action
*	
*	@param dev_t the laser focus controller
*	@param ctrl the action of mutex
*
*/
int mutex_ctrl(struct msm_laser_focus_ctrl_t *dev_t, int ctrl)
{
	int rc = 0;
	
	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	if(!dev_t){
		LOG_Handler(LOG_ERR, "%s: fail dev_t %p is NULL\n", __func__, dev_t);
		return -EFAULT;
	}

	switch(ctrl){
		/* Allocate mutex */
		case MUTEX_ALLOCATE:
			rc = _mutex_allocate(&dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_allocate\n", __func__);
				return rc;
			}
			break;
		/* Initialize mutex */
		case MUTEX_INIT:
			_mutex_init(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_init\n", __func__);
				return rc;
			}
			break;
		/* Lock mutex */
		case MUTEX_LOCK:			
			_mutex_lock(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_lock\n", __func__);
				return rc;
			}
			break;
		/* Try lock mutex*/
		case MUTEX_TRYLOCK:
			_mutex_trylock(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_trylock\n", __func__);
				return rc;
			}
			break;
		/* Unlock mutex */
		case MUTEX_UNLOCK:
			_mutex_unlock(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_unlock\n", __func__);
				return rc;
			}
			break;
		/* Destroy mutex */
		case MUTEX_DESTROY:
			_mutex_destroy(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_destroy\n", __func__);
				return rc;
			}
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail !!\n", __func__);
			break;
	}
	
	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;;
}



static ssize_t Laser_CSCmode_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val;
	char messages[8]="";
	if (len > 8)	len = 8;

	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);


	if(val)
		CSCmode = true;
	else
		CSCmode = false;

	printk("Laser CSCmode(%d)\n",CSCmode);
	
	return len;
}


int Laser_CSCmode_read(struct seq_file *buf, void *v)
{

	seq_printf(buf,"CSCmode(%d)\n", (int)CSCmode);
	
       return 0;
}

int Laser_CSCmode_open(struct inode *inode, struct  file *file)
{
        return single_open(file, Laser_CSCmode_read, NULL);
}

const struct file_operations Laser_CSCmode_fops = {
        .owner = THIS_MODULE,
        .open = Laser_CSCmode_open,
        .write = Laser_CSCmode_write,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};





static ssize_t dump_Laser_value_check_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val;
	char messages[8]="";
	if (len > 8)	len = 8;

	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);


	g_factory = val;

	printk("Laser g_factory(%d)\n",g_factory);
	
	return len;
}


int dump_Laser_value_check_read(struct seq_file *buf, void *v)
{
	timedMeasure = !timedMeasure;
	printk("timedMeasure switch to (%d)\n", (int)timedMeasure);
	seq_printf(buf,"timedMeasure switch to (%d)\n", (int)timedMeasure);
	seq_printf(buf,"PASS\n");
       return 0;
}

int dump_Laser_value_check_open(struct inode *inode, struct  file *file)
{
        return single_open(file, dump_Laser_value_check_read, NULL);
}

const struct file_operations dump_Laser_value_check_fops = {
        .owner = THIS_MODULE,
        .open = dump_Laser_value_check_open,
        .write = dump_Laser_value_check_write,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

int Laura_laser_focus_log_contorl_read(struct seq_file *buf, void *v)
{
	return 0;
}

int Laura_laser_focus_log_contorl_open(struct inode *inode, struct file *file)
{
	return single_open(file, Laura_laser_focus_log_contorl_read, NULL);
}

ssize_t Laura_laser_focus_log_contorl_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;
	
	rc = Laser_Focus_log_contorl(buff, len);
	
	return rc;
}

const struct file_operations laser_focus_log_contorl_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_log_contorl_open,
	.write = Laura_laser_focus_log_contorl_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};



extern int Read_Kdata_From_File(struct seq_file *vfile, uint16_t *cal_data);
//HPTG proc for CE catch part of K data (10 procs)+++

#if 0
static ssize_t dump_CE_debug_value1_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	
	char messages[8]="";
	
	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	thd = (int)simple_strtol(messages, NULL, 10);

	printk("HEPT Threshold %d\n",thd);

	return len;
}
#endif

static int dump_CE_debug_value1_read(struct seq_file *buf, void *v)
{
	int rc = 0;
	uint16_t conf_level;

	rc = CCI_I2C_RdWord(laura_t, RSLT_CONFIDENCE_REGISTER, &conf_level);	
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: read RSLT_CONFIDENCE_REGISTER failed!!\n", __func__);
		return rc;
	}
	
	conf_level = (conf_level & VEC_AMPL) >> VEC_AMPL_BIT_SHIFT;

	seq_printf(buf,"%d\n", conf_level);	
	return 0;
}

static int dump_debug_Kvalue1_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value1_read, NULL);
}

static const struct file_operations dump_debug_Kvalue1_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue1_open,
//	.write = dump_CE_debug_value1_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t dump_CE_debug_value2_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	
	char messages[8]="";
	
	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	limit = (int)simple_strtol(messages, NULL, 10);

	printk("HEPT Limit %d\n",limit);

	return len;
}

static int dump_CE_debug_value2_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);	

       seq_printf(buf,"%d\n",cal_data[20]);
	return 0;
}

static int dump_debug_Kvalue2_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value2_read, NULL);
}

static const struct file_operations dump_debug_Kvalue2_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue2_open,
	.write = dump_CE_debug_value2_write,	
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static ssize_t dump_CE_debug_value3_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	
	char messages[8]="";
	
	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	thd_near_mm = (int)simple_strtol(messages, NULL, 10);

	printk("HEPT Limit %d\n",thd_near_mm);

	return len;
}


static int dump_CE_debug_value3_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);	

       seq_printf(buf,"%d\n",cal_data[29]);
	return 0;
}

static int dump_debug_Kvalue3_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value3_read, NULL);
}

static const struct file_operations dump_debug_Kvalue3_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue3_open,
	.write = dump_CE_debug_value3_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value4_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);	

       seq_printf(buf,"%d\n",cal_data[22]);
	return 0;
}

static int dump_debug_Kvalue4_open(struct inode *inode, struct  file *file)
{

	return single_open(file, dump_CE_debug_value4_read, NULL);
}

static const struct file_operations dump_debug_Kvalue4_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue4_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value5_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);	

       seq_printf(buf,"%d\n",cal_data[31]);
	return 0;
}

static int dump_debug_Kvalue5_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value5_read, NULL);
}

static const struct file_operations dump_debug_Kvalue5_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue5_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int dump_CE_debug_value6_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);	

       seq_printf(buf,"%d\n",cal_data[23]);
	return 0;
}

static int dump_debug_Kvalue6_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value6_read, NULL);
}

static const struct file_operations dump_debug_Kvalue6_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue6_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value7_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);	

       seq_printf(buf,"%d\n",cal_data[32]);
	return 0;
}

static int dump_debug_Kvalue7_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value7_read, NULL);
}

static const struct file_operations dump_debug_Kvalue7_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue7_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value8_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);	

       seq_printf(buf,"%d\n",cal_data[24]);
	return 0;
}

static int dump_debug_Kvalue8_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value8_read, NULL);
}

static const struct file_operations dump_debug_Kvalue8_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue8_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value9_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);	

       seq_printf(buf,"%d\n",cal_data[33]);
	return 0;
}

static int dump_debug_Kvalue9_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value9_read, NULL);
}

static const struct file_operations dump_debug_Kvalue9_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue9_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//HPTG proc for CE catch part of K data (10 procs)---

static int HPTG_hardware_read(struct seq_file *buf, void *v)
{
	seq_printf(buf,"%d\n",Laser_Product);
	return 0;
}

static int HPTG_hardware_open(struct inode *inode, struct  file *file)
{
	return single_open(file, HPTG_hardware_read, NULL);
}

static const struct file_operations HPTG_hardware_fops = {
	.owner = THIS_MODULE,
	.open = HPTG_hardware_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};



static int Laser_Kdata_position_read(struct seq_file *buf, void *v)
{
	seq_printf(buf,"%d\n",g_factory);
	return 0;
}

static int Laser_Kdata_position_open(struct inode *inode, struct  file *file)
{
	return single_open(file, Laser_Kdata_position_read, NULL);
}

static const struct file_operations Laser_Kdata_position_fops = {
	.owner = THIS_MODULE,
	.open = Laser_Kdata_position_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int who_am_i_read(struct seq_file *buf, void *v){
	
	seq_printf(buf, "PASS\n");
	return 0;

}

static int who_am_i_open(struct inode *inode, struct  file *file)
{
	return single_open(file, who_am_i_read, NULL);
}

static const struct file_operations laser_who_am_i_fops = {
	.owner = THIS_MODULE,
	.open = who_am_i_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


extern const struct file_operations Laser_status_check_fops;
extern const struct file_operations Laser_check_producer_fops;

extern const struct file_operations ATD_laser_focus_device_enable_fops;
extern const struct file_operations ATD_laser_focus_device_get_range_fos;
extern const struct file_operations ATD_laser_focus_device_get_range_more_info_fos;

//Laser_get_raw_Kdata_fops: transfer raw Kdata, which derived from mailbox, to library
extern const struct file_operations Laser_calibration_fops;
extern const struct file_operations Laser_get_raw_Kdata_fops;

extern const struct file_operations dump_laser_focus_register_fops;
extern const struct file_operations dump_HPTG_debug_register_fops;


//setK and product_family are seldom to be used 
extern const struct file_operations laser_focus_set_K_fops;
extern const struct file_operations laser_focus_product_family;

extern const struct file_operations Laser_continuous_measure_fops;
extern const struct file_operations Laser_refresh_rate_fops;
extern const struct file_operations Laser_operation_mode_fops;
extern const struct file_operations Laser_proximity_parameter_fops;
extern const struct file_operations Laser_proximity_enable_fops;
extern const struct file_operations Laser_errortable_enable_fops;
extern const struct file_operations Laser_ambient_fops;
extern const struct file_operations Laser_amplitudeRatio_fops;
extern const struct file_operations Laser_suspend_fops;

void HEPTAGON_create_proc_file(void)
{
	proc(STATUS_PROC_FILE, 0664, &Laser_status_check_fops);
	proc(STATUS_PROC_FILE_FOR_CAMERA, 0664, &Laser_check_producer_fops);

	
	proc(DEVICE_TURN_ON_FILE, 0664, &ATD_laser_focus_device_enable_fops);
	proc(DEVICE_GET_VALUE, 0664, &ATD_laser_focus_device_get_range_fos);
	proc(DEVICE_GET_VALUE_MORE_INFO, 0664, &ATD_laser_focus_device_get_range_more_info_fos);


	proc(DEVICE_SET_CALIBRATION, 0664, &Laser_calibration_fops);
	proc(DEVICE_GET_CALIBRATION_INPUT_DATA, 0664, &Laser_get_raw_Kdata_fops);

	//for debug+
	proc(DEVICE_DUMP_REGISTER_VALUE, 0664, &dump_laser_focus_register_fops);
	proc(DEVICE_DUMP_DEBUG_VALUE, 0664, &dump_HPTG_debug_register_fops);
	proc(DEVICE_LOG_CTRL_FILE, 0664, &laser_focus_log_contorl_fops);
	
	//++vincent
	proc(DEVICE_CONTINUOUS_MEASURE_FEATURE, 0664, &Laser_continuous_measure_fops);
	proc(DEVICE_OPERATION_MODE_FEATURE, 0664, &Laser_operation_mode_fops);
	proc(DEVICE_REFRESH_RATE, 0664, &Laser_refresh_rate_fops);
	proc(DEVICE_PROXIMITY_PARAMETER, 0664, &Laser_proximity_parameter_fops);
	proc(DEVICE_PROXIMITY_ENABLE, 0664, &Laser_proximity_enable_fops);
	proc(DEVICE_ERRORTABLE_ENABLE, 0664, &Laser_errortable_enable_fops);
	proc(DEVICE_AMBIENT_VALUE, 0664, &Laser_ambient_fops);
	proc(DEVICE_AMPLITUDE_RATIO, 0664, &Laser_amplitudeRatio_fops);
	proc(DEVICE_SUSPEND_ENABLE, 0664, &Laser_suspend_fops);
	//for debug-
	
	//for ce+
	proc(DEVICE_DEBUG_VALUE1, 0664, &dump_debug_Kvalue1_fops);
	proc(DEVICE_DEBUG_VALUE2, 0664, &dump_debug_Kvalue2_fops);
	proc(DEVICE_DEBUG_VALUE3, 0664, &dump_debug_Kvalue3_fops);
	proc(DEVICE_DEBUG_VALUE4, 0664, &dump_debug_Kvalue4_fops);
	proc(DEVICE_DEBUG_VALUE5, 0664, &dump_debug_Kvalue5_fops);	
	proc(DEVICE_DEBUG_VALUE6, 0664, &dump_debug_Kvalue6_fops);
	proc(DEVICE_DEBUG_VALUE7, 0664, &dump_debug_Kvalue7_fops);
	proc(DEVICE_DEBUG_VALUE8, 0664, &dump_debug_Kvalue8_fops);
	proc(DEVICE_DEBUG_VALUE9, 0664, &dump_debug_Kvalue9_fops);
	proc(DEVICE_VALUE_CHECK, 0664, &dump_Laser_value_check_fops);
	//for ce-
	proc(DEVICE_HPTG_HW, 0664, &HPTG_hardware_fops);
	proc(DEVICE_DATA_POSITION, 0664, &Laser_Kdata_position_fops);
	proc(DEVICE_CSC_MODE, 0664, &Laser_CSCmode_fops);


	//for dit+
	proc(DEVICE_IOCTL_SET_K, 0664, &laser_focus_set_K_fops);
	proc(DEVICE_IOCTL_PRODUCT_FAMILY, 0444, &laser_focus_product_family);
	//for dit-
}

int set_laser_state(struct msm_laser_focus_ctrl_t *dev_t){

	dev_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	dev_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
	return 0;
}


