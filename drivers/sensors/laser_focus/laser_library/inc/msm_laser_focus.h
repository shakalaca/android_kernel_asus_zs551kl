/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *	Author:	Jheng-Siou, Cai
 *	Time:	2015-05
 */

#ifndef MSM_LASER_FOCUS_H
#define MSM_LASER_FOCUS_H

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/unistd.h>
#include <linux/workqueue.h> 
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>


#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

struct msm_laser_focus_ctrl_t;

#include "laser_focus.h"


/* Laser focus state */
enum msm_laser_focus_state_t {
	LASER_FOCUS_POWER_UP,	/* Power up */
	LASER_FOCUS_POWER_DOWN,	/* Power down */
};

/* Laser focuse data type */
enum msm_laser_focus_data_type {
	MSM_LASER_FOCUS_BYTE_DATA = 1,	/* Byte */
	MSM_LASER_FOCUS_WORD_DATA,		/* Word */
};

/* Laser focus address type */
enum msm_laser_focus_addr_type {
	MSM_LASER_FOCUS_BYTE_ADDR = 1,	/* Byte */
	MSM_LASER_FOCUS_WORD_ADDR,		/* Word */
};

/* Laser focus status controller */
enum msm_laser_focus_atd_device_trun_on_type {
	MSM_LASER_FOCUS_DEVICE_OFF = 0,	/* Device power off */
	MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION,	/* Device power on and apply calibration, then continuous single measurement */
	MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION,	/* Device power but no apply calibration, then single measurement */
	MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION_SINGLE, /* Device power on and apply calibration, then single measurement */
};

enum laser_state{
	STOP_STATE =0,
	OPENING_STATE,
	RUNNING_STATE,
	CLOSING_STATE,
};
enum laser_product_family{
	PRODUCT_UNKNOWN,
	PRODUCT_LAURA,
	PRODUCT_OLIVIA,
};

/* For HPTG fw error table setting */
struct sErrLimitConfig {
    uint16_t  u16MinConfLimit;
    uint16_t  u16MaxItLimit;
    uint32_t  u32MinConfMaxDistLimit;
};

struct sIntGAmplitudeRatio {
    uint16_t  objectThreshold;
    uint16_t  referenceThresold;
    uint16_t  smudgeThreshold;
};

/* Laser focus controller */
struct msm_laser_focus_ctrl_t {
	const char *sensor_name;
	u32 sensor_id;
	uint16_t fw_version;
	int slaveAddr;
	struct regulator *reg;
	struct i2c_client *apps_i2c_client;
	unsigned gpio;

	struct mutex *laser_focus_mutex;

	int16_t device_state;
	enum msm_laser_focus_state_t laser_focus_state;
	wait_queue_head_t suspend_wait_q;
	bool suspend;

	/* For calibration */
	int32_t laser_focus_offset_value;
	uint32_t laser_focus_cross_talk_offset_value;

	/* Elisa new feature */
	bool elisa_fw19;
	bool enable_errortable;	
	bool continuous_measurement;
	bool conditional_measurement;
	bool cal_start;	
	uint16_t operation_mode;
	uint16_t enable_proximity;	
	uint16_t u16RefreshRate_msec;

	/* Threshold of ambient sensitivity */
	uint16_t intg_sat_lvl;

	/* Set IntG amplitude ratio thresholds */
	struct sIntGAmplitudeRatio amplitudeRatio;

	/* Conditional Measurement */	
    uint16_t u16MinimumThreshold_mm;
    uint16_t u16MaximumThreshold_mm;
    int eMode;

	/* Proximity Config */
    uint16_t nearfield_distance_threshold;
    uint16_t intMin;
};


#define 	DEFAULT_AMBIENT				1600
#define 	DEFAULT_CONFIDENCE10		1300		
#define 	DEFAULT_CONFIDENCE_THD		5600
#define 	DEFAULT_IT					5000
#define 	DEFAULT_CONFIDENCE_FACTOR	(DEFAULT_TOF_NORMAL_3 & LASER_BIT_MASK(11, 0))
#define 	DEFAULT_DISTANCE_THD		1500
#define 	DEFAULT_ELISA_DISTANCE_THD	2000
#define 	DEFAULT_NEAR_LIMIT			99
#define		MAX_DIS_BIT_RANGE			2047
#define		MAX_DIS_ELISA_BIT_RANGE		4095


#define 	DEFAULT_TOF_NORMAL_0			0xE101
#define 	DEFAULT_TOF_NORMAL_1			0x10FF
#define 	DEFAULT_TOF_NORMAL_2			0x47D0
#define 	DEFAULT_TOF_NORMAL_3			0x5008
#define 	DEFAULT_TOF_NORMAL_4			0xA041
#define 	DEFAULT_TOF_NORMAL_5			0x45FC
/* Disable smudge detection */
/* #define 	DEFAULT_TOF_NORMAL_5			0x45F4 */

#define 	DEFAULT_TOF_K10_0				0xE101
#define 	DEFAULT_TOF_K10_1				0x10FF
#define 	DEFAULT_TOF_K10_2				0x47D0
#define 	DEFAULT_TOF_K10_3				0x5008
#define 	DEFAULT_TOF_K10_4				0x1C80
#define 	DEFAULT_TOF_K10_5				0x4174

#define 	DEFAULT_TOF_K40_0				0xE101
#define 	DEFAULT_TOF_K40_1				0x10FF
#define 	DEFAULT_TOF_K40_2				0x47D0
#define 	DEFAULT_TOF_K40_3				0x5008
#define 	DEFAULT_TOF_K40_4				0xFC80
#define 	DEFAULT_TOF_K40_5				0x4174

/* For FW version newer than v1.9 chip */
#define 	NEWFW_TOF_K10_5				0x4154
#define 	NEWFW_TOF_K40_5				0x4114


enum HPTG_config_enum {
	AMBIENT = 0,
	CONFIDENCE10,		
	CONFIDENCE_THD,
	IT,
	CONFIDENCE_FACTOR,
	DISTANCE_THD,
	NEAR_LIMIT,
	TOF_NORMAL_0,
	TOF_NORMAL_1,
	TOF_NORMAL_2,
	TOF_NORMAL_3,
	TOF_NORMAL_4,
	TOF_NORMAL_5,
	TOF_K10_0,
	TOF_K10_1,
	TOF_K10_2,
	TOF_K10_3,
	TOF_K10_4,
	TOF_K10_5,
	TOF_K40_0,
	TOF_K40_1,
	TOF_K40_2,
	TOF_K40_3,
	TOF_K40_4,
	TOF_K40_5,
	NUMBER_OF_SETTINGS,
};

/* Current consumption */
#define LASER_STANDBY_CURRENT  15			//UA
#define LASER_MEASURE_CURRENT  20000		//UA

/* Calibration parameter */
#define MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION		0
#define MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION		10	/* Calibration 10cm*/
#define MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION	40	/* Calibration 40cm*/
#define MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION		70	/* Calibration infinity */

#ifndef LOG_SAMPLE_RATE
/* Log sample rate  */
#define LOG_SAMPLE_RATE 50
#endif

#define RANGE_NOT_UPDATED		9900

/* Laser focus control file path */
#define	STATUS_PROC_FILE		"driver/LaserFocus_Status"	/* Status */
#define	STATUS_PROC_FILE_FOR_CAMERA	"driver/LaserFocus_Status_For_Camera"	/* Status (check on prob only) */
#define	DEVICE_TURN_ON_FILE		"driver/LaserFocus_on"	/* Power on/off */
#define	DEVICE_GET_VALUE		"driver/LaserFocus_value"	/* Get range value */
#define	DEVICE_GET_VALUE_MORE_INFO	"driver/LaserFocus_value_more_info"	/* Get range value, DMax and error code*/
#define	DEVICE_SET_CALIBRATION		"driver/LaserFocus_CalStart" /* Calibration */
#define	DEVICE_DUMP_REGISTER_VALUE	"driver/LaserFocus_register_dump"	/* Dump register value right */
#define	DEVICE_DUMP_DEBUG_VALUE	"driver/LaserFocus_debug_dump"	/* Dump register value for vl6180x debug */
#define DEVICE_CONTINUOUS_MEASURE_FEATURE "driver/LaserFocus_conti_measure_enable"	/* Check if continuous measure feature is enabled */
#define DEVICE_REFRESH_RATE "driver/LaserFocus_refresh_rate"	/* Check/Set refresh rate */
#define DEVICE_OPERATION_MODE_FEATURE "driver/LaserFocus_op_mode_enable"	/* Check/Set operation mode */
#define DEVICE_PROXIMITY_ENABLE "driver/LaserFocus_proximity_enable"	/* Proximity enable */
#define DEVICE_PROXIMITY_PARAMETER "driver/LaserFocus_proximity_parameter"	/* Proximity parameter */
#define DEVICE_ERRORTABLE_ENABLE "driver/LaserFocus_errortable_enable"	/* Error Table enable */
#define DEVICE_AMBIENT_VALUE "driver/LaserFocus_ambient"	/* Error Table enable */
#define DEVICE_AMPLITUDE_RATIO "driver/LaserFocus_amplitudeRatio"	/* Error Table enable */
#define DEVICE_SUSPEND_ENABLE "driver/LaserFocus_suspend_enable"	/* Error Table enable */
#define	DEVICE_ENFORCE_FILE	"driver/LaserFocus_enforce"	/* Disable laser focus value */
#define	DEVICE_LOG_CTRL_FILE	"driver/LaserFocus_log_ctrl"	/* Log contorl */
#define DEVICE_DEBUG_VALUE1	"driver/LaserFocus_log_value1" /* Debug value for CE collect data */
#define DEVICE_DEBUG_VALUE2	"driver/LaserFocus_log_value2" 
#define DEVICE_DEBUG_VALUE3	"driver/LaserFocus_log_value3" 
#define DEVICE_DEBUG_VALUE4	"driver/LaserFocus_log_value4" 
#define DEVICE_DEBUG_VALUE5	"driver/LaserFocus_log_value5" 
#define DEVICE_DEBUG_VALUE6	"driver/LaserFocus_log_value6"
#define DEVICE_DEBUG_VALUE7	"driver/LaserFocus_log_value7"
#define DEVICE_DEBUG_VALUE8	"driver/LaserFocus_log_value8"
#define DEVICE_DEBUG_VALUE9	"driver/LaserFocus_log_value9"
#define DEVICE_DEBUG_VALUE10	"driver/LaserFocus_log_value10"
#define DEVICE_VALUE_CHECK	"driver/LaserFocus_value_check" 
#define DEVICE_IOCTL_SET_K	"driver/LaserFocus_setK"
#define DEVICE_IOCTL_PRODUCT_FAMILY	"driver/LaserFocus_ProductFamily"
#define  DEVICE_WHO_AM_I		"driver/LaserFocus_who_am_i"
#define DEVICE_HPTG_HW		"driver/LaserFocus_HPTG_hw"
#define DEVICE_DATA_POSITION	"driver/LaserFocus_data_position"
#define	DEVICE_CSC_MODE	"driver/LaserFocus_CSCmode"



/* Right of laser focus control file*/
#define	STATUS_PROC_FILE_MODE 0660	/* Status right */
#define	STATUS_PROC_FILE_FOR_CAMERA_MODE 0660	/* Status (check on prob only) right */
#define	DEVICE_TURN_ON_FILE_MODE 0660	/* Power on/off right */
#define	DEVICE_GET_VALUE_MODE 0664	/* Get value right */
#define	DEVICE_GET_VALUE_MODE_MORE_INFO 0664	/* Get value more info right */
#define	DEVICE_SET_CALIBRATION_MODE 0660	/* Calibration right */
#define	DEVICE_DUMP_REGISTER_VALUE_MODE 0660	/* Dump register value right */
#define	DEVICE_DUMP_DEBUG_VALUE_MODE 0660	/* Dump register value right for vl6180x debug */
#define	DEVICE_ENFORCE_MODE	0660	/* Laser focus disable right */
#define	DEVICE_LOG_CTRL_MODE	0660	/* Log contorl right */


#define	PRODUCER_UNKNOWN	0
#define	PRODUCER_ST			1
#define	PRODUCER_HPTG		2
#define	I2C_STATUS_FAIL		0
#define	I2C_STATUS_PASS		1

#define IOCTL_SUCCESS	0
#define IOCTL_FAILED	-1


#define REF_K_DATA_NUMBER	4
#define	DMAX_K_DATA_NUMBER 3
//DEFINES for laser ST ---

//unit us
#define DEFAULT_DELAY_TIME 1000 
#define MCPU_DELAY_TIME 700	
#define READ_DELAY_TIME 500	

/* Size of Laura calibration data */
#define SIZE_OF_LAURA_CALIBRATION_DATA 10
//#define SIZE_OF_OLIVIA_CALIBRATION_DATA 38
#define SIZE_OF_OLIVIA_CALIBRATION_DATA 44
#define CONFIDENCE_LENGTH	2


/* Check device verify number */
int Laser_Match_ID(struct msm_laser_focus_ctrl_t *dev_t);
/* Voltage control */
//int32_t vreg_control(struct msm_laser_focus_ctrl_t *dev_t, int config);
/* Power on component */
int32_t PowerUp(struct msm_laser_focus_ctrl_t *dev_t);
/* Power off component */
int32_t PowerDown(struct msm_laser_focus_ctrl_t *dev_t);

//void Laser_Match_Module(struct msm_laser_focus_ctrl_t *dev_t);

/* Check device status */
int HPTG_I2C_status_check(struct msm_laser_focus_ctrl_t *dev_t);

/* Mutex controller handles mutex action */
int mutex_ctrl(struct msm_laser_focus_ctrl_t *dev_t, int ctrl);
void HEPTAGON_create_proc_file(void);
int set_laser_state(struct msm_laser_focus_ctrl_t *dev_t);

#endif
