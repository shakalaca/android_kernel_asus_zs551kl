/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#include "HPTG_shipping_func.h"
//#include "HPTG_interface.h"
#include "laser_log.h"
#include <linux/delay.h>
#include <linux/unistd.h>
/* Log count */
int read_range_log_count = 0; // Read range log count

/* Module id */
static bool module_id_flag = false;
uint16_t module_id[34];
uint16_t FW_version[2];
uint16_t chipID;
uint16_t f0_data[CAL_MSG_F0_ELISA*2];

bool newKdata = true;

extern int ErrCode1;
extern int ErrCode2;
extern uint16_t Range1;
extern uint16_t Range2;

extern bool timedMeasure;
extern bool dump0x09;
extern bool ioctrl_close;
extern bool repairing_state;

extern bool CSCmode;

static uint16_t debug_raw_range = 0;
static uint16_t debug_raw_confidence = 0;

extern int g_factory;
extern int Laser_log_cnt;
extern int proc_log_cnt;
extern uint16_t Settings[NUMBER_OF_SETTINGS];

extern int LaserState;
extern int client;

extern struct msm_laser_focus_ctrl_t *laura_t;

static void init_debug_raw_data(void){
		debug_raw_range = 0;
		debug_raw_confidence = 0;
}

uint16_t get_debug_raw_range(void){
	return debug_raw_range;
}

uint16_t get_debug_raw_confidence(void){
	return debug_raw_confidence;
}


/** @brief Swap high and low of the data (e.g 0x1234 => 0x3412)
*
*	@param register_data the data which will be swap
*
*/
void swap_data(uint16_t* register_data){
	*register_data = ((*register_data >> 8) | ((*register_data & 0xff) << 8)) ;
}


/** @brief Mailbox: create calibration data
*		  This mailbox command is used to retrieve data to be used for the computation of calibration parameters.
*		  This is a singleentry MBX command with MBX message response with Msg_len = 6
*
*	@param dev_t the laser focus controller
*	@param cal_data the calibration ouput data
*
*/
int Check_Ready_via_ICSR(struct msm_laser_focus_ctrl_t *dev_t){
	int status =0;
	uint16_t i2c_read_data, i2c_read_data2;
	
	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0)
			return status;

		if((i2c_read_data&(NEW_DATA_IN_MBX)) == GO_AHEAD)
			return status;

		/* Busy pending MCPU Msg */
		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data2);
		if (status < 0)
			return status;

		msleep(1);
		LOG_Handler(LOG_ERR, "%s: register(0x00, 0x10): (0x%x, 0x%x)\n", __func__,  i2c_read_data, i2c_read_data2);

	}
}

int Wait_for_Notification_from_ICSR(struct msm_laser_focus_ctrl_t *dev_t){
	int status =0;
	uint16_t i2c_read_data;
	int count = 0;

	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0 || (i2c_read_data & NEW_DATA_IN_MBX))
			return status;

		LOG_Handler(LOG_DBG,"%s: Poll ICSR again\n", __FUNCTION__);
		if(++count > 1000)
			return -1;
		
		msleep(1);
	}
}

int Olivia_Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, uint16_t cmd, int16_t cal_data[], int cal_msg_len){
	int status = 0, msg_index = 0, M2H_Msg_Len = 0;
	uint16_t i2c_read_data;
	status = Check_Ready_via_ICSR(dev_t);
	//if(status !=0)

	//single entry message? -> Cmd_len=0
	LOG_Handler(LOG_DBG, "Issue mailbox command 0x%04x to trigger calibration\n", cmd);
	status = CCI_I2C_WrWord(dev_t, H2M_MBX, cmd);
	if (status < 0){
		return status;
	}

	status = Wait_for_Notification_from_ICSR(dev_t);
	if (status < 0){
		LOG_Handler(LOG_ERR,"%s: Poll ICSR timeout 1\n", __FUNCTION__);
		return status;
	}
	//if(status !=0)

	status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
	if (status < 0) {
		return status;
	}
	
	M2H_Msg_Len = (i2c_read_data & CMD_LEN_MASK) >> 8;
	LOG_Handler(LOG_DBG, "%s: got response (0x%04x), Length (%d)\n", __FUNCTION__, i2c_read_data & 0xff, M2H_Msg_Len);
	
	if(M2H_Msg_Len != cal_msg_len) {
		LOG_Handler(LOG_ERR,"Message length %d is not in expect(%d)\n", M2H_Msg_Len, cal_msg_len);
		return -1;
	}
	
	if((i2c_read_data & MESSAGE_ID_MASK) == 0xCC) {
		for(msg_index = 0; msg_index < cal_msg_len; msg_index++){
			status = Wait_for_Notification_from_ICSR(dev_t);
			if (status < 0){
				LOG_Handler(LOG_ERR,"%s: Poll ICSR timeout 2\n", __FUNCTION__);
				return status;
			}			
			//if(status !=0)
       		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
			if (status < 0){
				LOG_Handler(LOG_CDBG, "Read payload fail, break at index %d\n",msg_index);
              		return status;
       		}
			/* Append to previosly saved data */
			cal_data[msg_index] = i2c_read_data;
			//LOG_Handler(LOG_CDBG, "%s: Calibration data[%d]: 0x%x\n", __func__, msg_index, cal_data[msg_index]);
			//print  these at proc debug dump
		}
		LOG_Handler(LOG_DBG, "end%d \n",msg_index);
	}
	else{
		LOG_Handler(LOG_ERR, "%s: M2H_MBX(7:0): 0x%x, Msg_Len: %d\n", __func__, i2c_read_data&0xFF, M2H_Msg_Len);
		return -1;
	}
	return status;
}


int hptg_h2m_mailbox_command(struct msm_laser_focus_ctrl_t *dev_t, uint16_t opCode, uint16_t *u16MbxH2mData, int mbxDataLength)
{
	uint16_t i2c_read_data;
	int cnt = 0, i = 0;
	int status;

	status = CCI_I2C_WrWord(dev_t, H2M_MBX, opCode);
    if (status < 0){
        LOG_Handler(LOG_ERR, "%s::%d - Unable to write H2M MBX(%d)\n", __FUNCTION__, __LINE__, status);
        return -1;
    }
    else
        LOG_Handler(LOG_DBG, "%s: Issued Mailbox Command OpCode (0x%04x), payload length (%d)\n", __FUNCTION__, opCode, mbxDataLength);
	

	while(1){
		CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if(i2c_read_data & MCPU_HAVE_READ_MBX){
			cnt=0;
			break;
		}
		if(++cnt > 50){
			LOG_Handler(LOG_ERR, "%s: timeout 1\n",__func__);
			return -1;
		}
		msleep(1);
	}

	for (i = 0; i < mbxDataLength; i++) {
		status = CCI_I2C_WrWord(dev_t, H2M_MBX, u16MbxH2mData[i]);
        if (status < 0){
            LOG_Handler(LOG_ERR, "%s::%d - Unable to write H2M MBX(%d)\n", __FUNCTION__, __LINE__, status);
            return -1;
        }
        else
            LOG_Handler(LOG_DBG, "%s: Issued Mailbox Command Payload (%d) successfully\n", __FUNCTION__, u16MbxH2mData[i]);
		
		while(1){
			CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
			if(i2c_read_data & MCPU_HAVE_READ_MBX){
				cnt=0;
				break;
			}
			if(++cnt > 50){
				LOG_Handler(LOG_ERR, "%s: timeout 2\n",__func__);
				return -1;
			}
			msleep(1);
		}

		status = CCI_I2C_RdWord(dev_t, 0x12, &i2c_read_data);
		if(status < 0){
			LOG_Handler(LOG_ERR, "%s::%d - unable to read MCPU to Host Mailbox(%d)\n", __FUNCTION__, __LINE__, status);
		}
	}
	return status;
}


int ambient_setting(struct msm_laser_focus_ctrl_t *dev_t){

	int status=0;
	uint16_t mbxPayload;
#if 0
	LOG_Handler(LOG_DBG, "%s: ambient setting(%d)\n",__func__,Settings[AMBIENT]);
	mbxPayload = Settings[AMBIENT];
#else
	LOG_Handler(LOG_DBG, "%s: ambient setting(%d)\n",__func__, laura_t->intg_sat_lvl);
	mbxPayload = laura_t->intg_sat_lvl;
#endif
	hptg_h2m_mailbox_command(dev_t, 0x81C7, &mbxPayload, 1);

	return status;
}

int hptg_set_intG_amplitude_ratio(struct msm_laser_focus_ctrl_t *dev_t)
{
    int status;
    uint16_t mbxPayload[3];
    
	mbxPayload[0] = laura_t->amplitudeRatio.objectThreshold;
	mbxPayload[1] = laura_t->amplitudeRatio.referenceThresold;
	mbxPayload[2] = laura_t->amplitudeRatio.smudgeThreshold;

	LOG_Handler(LOG_DBG, "%s: Set intG amplitude ratio thresholds\n", __FUNCTION__);
	LOG_Handler(LOG_DBG, "%s: objectThreshold   => 0x%04x (%d)\n", 
							__FUNCTION__, 
							mbxPayload[0], mbxPayload[0]);
	LOG_Handler(LOG_DBG, "%s: referenceThresold => 0x%04x (%d)\n", __FUNCTION__, 
							mbxPayload[1], mbxPayload[1]);
	LOG_Handler(LOG_DBG, "%s: smudgeThreshold   => 0x%04x (%d)\n", __FUNCTION__, 
							mbxPayload[2], mbxPayload[2]);
	
    status = hptg_h2m_mailbox_command(dev_t, 0x83CE, mbxPayload, 3);

    if (status < 0)
        LOG_Handler(LOG_ERR, "%s::%d - unable to set intG amplitude ratio thresholds(%d)\n", __FUNCTION__, __LINE__, status);

    return status;
}

int hptg_set_cont_meas_ready_event_param(struct msm_laser_focus_ctrl_t *dev_t, uint16_t u16MinimumThreshold_mm, uint16_t u16MaximumThreshold_mm, int eMode)
{
    int status;
    uint16_t mbxPayload[3];
    mbxPayload[0] = u16MinimumThreshold_mm;
    mbxPayload[1] = u16MaximumThreshold_mm;
    mbxPayload[2] = eMode;

	LOG_Handler(LOG_DBG, "%s: Set continuous measurement params\n", __FUNCTION__);
    status = hptg_h2m_mailbox_command(dev_t, 0x83CA, mbxPayload, 3);

    if (status < 0)
        LOG_Handler(LOG_ERR, "%s::%d - unable to set continuous measurement params(%d)\n", __FUNCTION__, __LINE__, status);

    return status;
}

int hptg_set_err_limit_config(struct msm_laser_focus_ctrl_t *dev_t, struct sErrLimitConfig sConfig)
{
	int status;
	uint16_t mbxPayload[4];

	LOG_Handler(LOG_DBG, "%s: Set default error limit configs\n", __FUNCTION__);
	LOG_Handler(LOG_DBG, "%s: u16MinConfLimit => 0x%04x (%d)\n", __FUNCTION__, sConfig.u16MinConfLimit, sConfig.u16MinConfLimit);
	LOG_Handler(LOG_DBG, "%s: u16MaxItLimit => 0x%04x (%d)\n", __FUNCTION__, sConfig.u16MaxItLimit, sConfig.u16MaxItLimit);
	LOG_Handler(LOG_DBG, "%s: u32MinConfMaxDistLimit => 0x%08x (%ld)\n", __FUNCTION__, sConfig.u32MinConfMaxDistLimit, sConfig.u32MinConfMaxDistLimit);
	mbxPayload[0] = sConfig.u16MinConfLimit;
	mbxPayload[1] = sConfig.u16MaxItLimit;
	mbxPayload[2] = sConfig.u32MinConfMaxDistLimit & 0x0000FFFF;
	mbxPayload[3] = sConfig.u32MinConfMaxDistLimit >> 16;

	status = hptg_h2m_mailbox_command(dev_t, 0x84CB, mbxPayload, 4);

	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s::%d - unable to set current error limit configs(%d)\n", __FUNCTION__, __LINE__, status);
	}

	return status;
}

int hptg_config_proximity(struct msm_laser_focus_ctrl_t *dev_t)
{
	uint16_t mbxPayload;
	int status;

	mbxPayload = dev_t->nearfield_distance_threshold;
	LOG_Handler(LOG_DBG, "%s: MBX: Set Proximity detection distance (%d mm)\n", __FUNCTION__, mbxPayload/4);
	status = hptg_h2m_mailbox_command(dev_t, 0x81C9, &mbxPayload, 1); /*Set INTG saturation level*/
	if (status < 0) {
	    LOG_Handler(LOG_ERR, "%s::%d - unable to set Proximity detection distance(%d)\n", __FUNCTION__, __LINE__, status);
	    goto FAIL;
	}

	mbxPayload = dev_t->intMin;
	LOG_Handler(LOG_DBG, "%s: MBX: Set minimum Integration Time (%d us)\n", __FUNCTION__, mbxPayload);
	status = hptg_h2m_mailbox_command(dev_t, 0x81C3, &mbxPayload, 1); /*Set INTG saturation level*/
	if (status < 0) {
	    LOG_Handler(LOG_ERR, "%s::%d - unable to set minimum Integration Time(%d)\n", __FUNCTION__, __LINE__, status);
	    goto FAIL;
	}

FAIL:
	return status;
	
}

int tof_get_extended_err_bitfield(void)
{
    int result = 0;
    int status = 0;
    uint16_t errbit = 0;

    status = Olivia_Mailbox_Command(laura_t, 0x0008, &errbit, 1);

    if (status < 0) {
        LOG_Handler(LOG_ERR, "%s::%d - MBOX Command failed!(%d)\n", __FUNCTION__, __LINE__, status);
        return -1;
    }

    LOG_Handler(LOG_ERR, "%s: Extended error bit: 0x%04x\n", __FUNCTION__, errbit);
    if (errbit == 0) {
        //TOF_LOG(LOG_ERR, "%s: (EXT_ERROR_NO_ERROR)\n", __FUNCTION__);
        LOG_Handler(LOG_ERR, "(EXT_ERROR_NO_ERROR)\n");
        result = 0;
    }
    else {
        if (errbit & BIT(0)) {
        /*Error generated by object pixel with integration gate above saturated limit (clamped pixels).
        Integration gate saturation limit defined by mailbox command 0xC7.
        Condition:
        {Number of valid pixel*** < Rslt_CnfdR(0x030A):Bit(3:0)}
        and {Number of clamped pixel > Number of low amplitude pixel*}
        and {Number of clamped pixel > Number of high amplitude pixel**}
         
         * Low amplitude threshold defined by mailbox command 0xCE
         ** High amplitude threshold defined by HFCfg_0(0x0320):[11:0]
         *** Valid pixels defined as no-clamped, no-low-amplitude and no-high-amplitude pixels*/
            LOG_Handler(LOG_ERR, "%s: (EXT_ERROR_CLAMPED_PIXELS)\n", __FUNCTION__);
            result = 1;
        }
        else if (errbit & BIT(1)) {
        /*Error generated by no-valid reference pixel value*/
            LOG_Handler(LOG_ERR, "%s: (EXT_ERROR_INVALID_REFERENCE)\n", __FUNCTION__);
            result = 2;
        }
        else if (errbit & BIT(2)) {
        /*Error generated if any of the two smudge pixels is saturated*/
            LOG_Handler(LOG_ERR, "%s: (EXT_ERROR_SMUDGE_FRAME_SATURATED)\n", __FUNCTION__);
            result = 3;
        }
        else if (errbit & BIT(3)) {
        /*Error generated if distance is equal to zero and no error condition was detected*/
            LOG_Handler(LOG_ERR, "%s: (EXT_ERROR_DISTANCE_ZERO)\n", __FUNCTION__);
            result = 4;
        }
        else if (errbit & BIT(4)) {
        /*Error generated if confidence value below threshold ConfLimit and integration time below threshold IntLimit*/
            LOG_Handler(LOG_ERR, "%s: (EXT_ERROR_CONF_INT_FILTER)\n", __FUNCTION__);
            result = 5;
        }
        else if (errbit & BIT(5)) {
        /*Error generated if confidence value below threshold defined as ConfDistLimit / distance*/
            LOG_Handler(LOG_ERR, "%s: (EXT_ERROR_CONF_DIST_FILTER)\n", __FUNCTION__);
            result = 6;
        }
        else if (errbit & BIT(6)) {
            LOG_Handler(LOG_ERR, "%s: (EXT_ERROR_SM_ABS_AMP_TH0)\n", __FUNCTION__);
            result = 7;
        }
        else if (errbit & BIT(7)) {
            LOG_Handler(LOG_ERR, "%s: (EXT_ERROR_SM_ABS_AMP_TH1)\n", __FUNCTION__);
            result = 8;
        }
    }
    return result;
}

int Olivia_device_Load_Calibration_Value(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t indirect_addr;
	static uint16_t data[SIZE_OF_OLIVIA_CALIBRATION_DATA+CONFIDENCE_LENGTH];
	static bool gotKdata = false;
	static int cnt=0;
#if DEBUG_LOG_FLAG	
	uint16_t data_verify;
#else
	uint16_t header_verify0,header_verify1;
	uint16_t header_verify[3];
#endif
	int i = 0;

	LOG_Handler(LOG_DBG, "%s: Load calibration start!!\n", __func__);

	/* Read Calibration data, addr is swapped */
	indirect_addr = 0xC010;

	//notice that CSCmode changed at running time
	if(CSCmode)
		gotKdata = false;

	if(!gotKdata){
		status = Larua_Read_Calibration_Data_From_File(data, SIZE_OF_OLIVIA_CALIBRATION_DATA+CONFIDENCE_LENGTH);
		if(status < 0){
//			if(++cnt>=50){
				LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
				cnt=0;
//			}	
			return status;
		}

#if 0
		//swap confidence data back
		swap_data(&data[SIZE_OF_OLIVIA_CALIBRATION_DATA]);
		swap_data(&data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1]);	
#endif /* Vincent Remove cci */

		//FAE suggest filtering-range
		if(500<data[SIZE_OF_OLIVIA_CALIBRATION_DATA]&&
			2047>data[SIZE_OF_OLIVIA_CALIBRATION_DATA]&&
			2500<data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1]&&
			10235>data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1]){
			Settings[CONFIDENCE10] =  data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
			Settings[CONFIDENCE_THD] = data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1];
			newKdata = true;
		}
		else{
			data[SIZE_OF_OLIVIA_CALIBRATION_DATA] = DEFAULT_CONFIDENCE10;
			data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1] = DEFAULT_CONFIDENCE_THD;
			newKdata = false;
		}

		if(g_factory)
			gotKdata = false;
		else
			gotKdata = true;
	}

	Settings[CONFIDENCE10] =  data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
	Settings[CONFIDENCE_THD] = data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1];
	
	Laura_device_indirect_addr_write(dev_t, indirect_addr, data, SIZE_OF_OLIVIA_CALIBRATION_DATA);

	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
#if DEBUG_LOG_FLAG
	for(i = 0; i < SIZE_OF_OLIVIA_CALIBRATION_DATA; i++){
		CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &data_verify);
		LOG_Handler(LOG_DBG, "%s: 0x1A: 0x%04x\n", __func__, data_verify);
	}
#else

	for(i=0; i<3; i++){
		CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &header_verify0);
		CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &header_verify1);
		header_verify0 &= 0x00ff;
		header_verify1 &= 0x00ff;
		header_verify[i] = (header_verify1<<8)|header_verify0;
	}
	LOG_Handler(LOG_DBG, "%s: header 0x%04x 0x%04x 0x%04x, expect 0x%04x 0x%04x 0x%04x\n", 
					__func__, header_verify[0],header_verify[1],header_verify[2], data[0], data[1], data[2]);
#endif

	return status;
}


int Read_Kdata_From_File(struct seq_file *vfile, uint16_t *cal_data){
	int status = 0, i = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = Larua_Read_Calibration_Data_From_File(cal_data, SIZE_OF_OLIVIA_CALIBRATION_DATA);
        if(status < 0){
                LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		if(vfile!=NULL){
			seq_printf(vfile,"No calibration data!!\n");
		}
                return status;
        }

#if 0
	for(i = 0; i < SIZE_OF_OLIVIA_CALIBRATION_DATA; i++){
                swap_data(cal_data+i);
        }
#endif /* Vincent Remove cci */

	LOG_Handler(LOG_DBG,"part Cal data: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
		cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],
		cal_data[5], cal_data[6], cal_data[7], cal_data[8], cal_data[9]);	

	if(vfile!=NULL){
		for(i=0; i<SIZE_OF_OLIVIA_CALIBRATION_DATA; i++)
			seq_printf(vfile,"%04x",cal_data[i]);
		seq_printf(vfile,"\n");

	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}


#define	MCPU_SWITCH_TIMEOUT_ms	50
#define	MEASURE_TIME_OUT_ms		120

int Verify_Range_Data_Ready(struct msm_laser_focus_ctrl_t *dev_t){
	
	uint16_t i2c_read_data = 0;
	int status=0;

#ifdef VINCENT_DEBUG
	int elapse_time;
	struct timeval start,now;
	O_get_current_time(&start);
#endif
	
	while(1) {
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0)
       		break;
	
		if(i2c_read_data & NEW_DATA_IN_RESULT_REG){
#ifdef VINCENT_DEBUG			
			O_get_current_time(&now);
			DeltaTime_ms(start, now, &elapse_time);

			LOG_Handler(LOG_DBG, "%s: range data ready, elapse time = %dms\n", __func__, elapse_time);
#endif
			break;
		}

		O_get_current_time(&now);
		if(is_timeout(start,now,MEASURE_TIME_OUT_ms)) {
			LOG_Handler(LOG_ERR, "%s: fail (time out)\n", __func__);
			status = -(OUT_OF_RANGE);
			break;
		}

		if(laura_t->suspend)
			break;

		/* Delay: waitting laser sensor sample ready */
		usleep_range(3000, 5000);
	}
	   
	return status;
}


bool compareConfidence(uint16_t confidence, uint16_t Range, uint16_t thd, uint16_t limit){
	return  		confidence <= thd*limit/Range;
}


uint16_t thd = 16;
uint16_t limit = 1500;
uint8_t thd_near_mm = 0; 

int	Read_Range_Data(struct msm_laser_focus_ctrl_t *dev_t){
	
	uint16_t RawRange = 0, Range = 0, error_status = 0;
	uint16_t RawConfidence = 0, confidence_level =0;
	int status;
	int errcode;
	int confirm=0;
	int max_dis_bit_range = MAX_DIS_ELISA_BIT_RANGE;
	int16_t cal_data[CAL_MSG_LEN_ELISA];
	uint16_t test[CAL_MSG_LEN_ELISA];
	int index;
	
	uint16_t IT_verify;

	int confA = 1300;
	int confC = 5600;
	int ItB = 5000;

	thd = Settings[CONFIDENCE_FACTOR];
	limit = Settings[DISTANCE_THD];
	thd_near_mm = Settings[NEAR_LIMIT];

	if(laura_t->elisa_fw19) {
		CCI_I2C_WrByte(dev_t, 0x18, 0x06);
		CCI_I2C_WrByte(dev_t, 0x19, 0x20);
		CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &IT_verify);
	} else {
		CCI_I2C_WrByte(dev_t, 0x18, 0x30);
		CCI_I2C_WrByte(dev_t, 0x19, 0x20);
		CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &IT_verify);
	}

	
	confA = Settings[CONFIDENCE10];
	confC = Settings[CONFIDENCE_THD];
	ItB =  Settings[IT];

	init_debug_raw_data();
	
      	status = CCI_I2C_RdWord(dev_t, RESULT_REGISTER, &RawRange);
	if (status < 0)
             	return status;

	debug_raw_range = RawRange;
	Range = (RawRange & DISTANCE_ELISA_MASK) >> 1;

	error_status = RawRange&ERROR_CODE_MASK;

	CCI_I2C_RdWord(dev_t, 0x0A, &RawConfidence);
	if (status < 0)
            	return status;
       
	debug_raw_confidence = RawConfidence;
	confidence_level = (RawConfidence&0x7ff0)>>4;

	if(dump0x09) {
		for(index = 0; index < CAL_MSG_LEN_ELISA; index++) {
			cal_data[index] = 0;
		}
		
		/* CMD_MBX to read data */
		status = Olivia_Mailbox_Command(dev_t, GET_ELISA_CALIBRATION, cal_data, CAL_MSG_LEN_ELISA);
		if(status < 0){
			LOG_Handler(LOG_ERR, "%s: MBX Command failed!!\n", __func__);
			return -EMODULE;
		}

		for(index = 0; index < CAL_MSG_LEN_ELISA; index++) {
			test[index] = (uint16_t)cal_data[index];
		}		
	}

	if(dev_t->enable_errortable){
		if(RawRange&VALID_DATA){
			if(!confirm){
				if(IT_verify < ItB){
					if(confidence_level < confA){
						confirm =1;
						errcode = RANGE_ERR_NOT_ADAPT;
						Range =	OUT_OF_RANGE;					
					}
				}
				else{
					if(( ItB*(confidence_level) < ItB*confA- (IT_verify - ItB)*(confC - confA) )){
						confirm =2;
						errcode = RANGE_ERR_NOT_ADAPT;
						Range =	OUT_OF_RANGE;										
					}
				}
			}
			if(!confirm && error_status==NO_ERROR && Range == 0){
				confirm=3;
				errcode = RANGE_ERR_NOT_ADAPT;
				Range =OUT_OF_RANGE;			
			}
			if(!confirm&& error_status==NO_ERROR && compareConfidence(confidence_level,Range,thd,limit)){
				confirm=4;
				errcode = RANGE_ERR_NOT_ADAPT;
				Range =OUT_OF_RANGE;	
			}
			if(!confirm&&error_status==GENERAL_ERROR){
				confirm=5;
				errcode = RANGE_ERR_NOT_ADAPT;
				Range =OUT_OF_RANGE;
			}
			if(!confirm&&error_status==NEAR_FIELD){
				if(confidence_level < 100) {
					confirm=11;
					errcode = RANGE_ERR_NOT_ADAPT;
					Range =OUT_OF_RANGE;
				} else {
					confirm=6;
					errcode = RANGE_ADAPT;
					Range =thd_near_mm;
				}
			}
			if(!confirm&&error_status==FAR_FIELD){
				confirm=7;
				errcode = RANGE_ADAPT;
				Range =OUT_OF_RANGE;
			}		
			if(!confirm&&Range==max_dis_bit_range){
				confirm=8;
				errcode = RANGE_ADAPT;
				Range =OUT_OF_RANGE;
			}
			if(dev_t->operation_mode != OP_MODE_LONG){
				if(!confirm&&Range>=DEFAULT_DISTANCE_THD){
					confirm=9;
					errcode = RANGE_ADAPT;
					Range =OUT_OF_RANGE;
				}
			}
			if(!confirm){
				confirm =0;
				if(Range <= 100 && !g_factory)
					errcode = RANGE_ADAPT_WITH_LESS_ACCURACY;
				else
					errcode = RANGE_ADAPT;
			}

		}
		else{
			confirm=10;
			Range=OUT_OF_RANGE;
			errcode = RANGE_ERR_NOT_ADAPT;	
		}
		ErrCode1 = errcode;
		Range1 = Range;	

	}else {
		ErrCode1 = error_status >> 13;
		Range1 = Range;
	}

	if((Laser_log_cnt==LOG_SAMPLE_RATE-1)||(Laser_log_cnt==LOG_SAMPLE_RATE-2)||proc_log_cnt){
		LOG_Handler(LOG_CDBG,"%s: conf(%d)  confA(%d) confC(%d) ItB(%d) IT_verify(%s:%d)\n", __func__, confidence_level, confA,confC,ItB,dev_t->elisa_fw19? "0x2006" : "0x2030", IT_verify);
		LOG_Handler(LOG_CDBG, "%s: status(%d) thd(%d) limit(%d) near(%d) Confidence(%d)\n", __func__,
			error_status>>13, thd, limit, thd_near_mm, confidence_level);
		LOG_Handler(LOG_CDBG, "%s: Range(%d) ErrCode(%d) RawRange(%x) RawDistance(%d) case(%d)\n", __func__,Range1,ErrCode1,RawRange, (RawRange & DISTANCE_ELISA_MASK) >> 1, confirm);	

		if((error_status>>13) == 3) {
			tof_get_extended_err_bitfield();
		}

		if(dump0x09) {
/*
			LOG_Handler(LOG_CDBG, "%s: Mailbox command 0x0009 payload(hex)\n", __func__);
			LOG_Handler(LOG_CDBG, "--------------------------------------------------------\n");
			LOG_Handler(LOG_CDBG, "%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n", 
				test[0], test[1], test[2], test[3], test[4], 
				test[5], test[6], test[7], test[8], test[9]);
			LOG_Handler(LOG_CDBG, "%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n", 
				test[10], test[11], test[12], test[13], test[14], 
				test[15], test[16], test[17], test[18], test[19]);
			LOG_Handler(LOG_CDBG, "%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n", 
				test[20], test[21], test[22], test[23], test[24], 
				test[25], test[26], test[27], test[28], test[29]);
			LOG_Handler(LOG_CDBG, "%04x %04x %04x\n", 
				test[30], test[31], test[32]);
*/
			LOG_Handler(LOG_CDBG, "%s: Mailbox command 0x0009 payload (Decimal)\n", __func__);
			LOG_Handler(LOG_CDBG, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", 
				test[0], test[1], test[2], test[3], test[4], test[5], test[6], test[7], test[8], test[9],
				test[10], test[11], test[12], test[13], test[14], test[15], test[16], test[17], test[18], test[19],
				test[20], test[21], test[22], test[23], test[24], test[25], test[26], test[27], test[28], test[29],
				test[30], test[31], test[32]);
			LOG_Handler(LOG_CDBG, "-----------------------------------------------------------------\n");			
		}
	}

	//reset cnt for case that use both proc and ioctrl
	proc_log_cnt=0;

	return Range;

}

int	Read_Range_Data_OldKdata(struct msm_laser_focus_ctrl_t *dev_t){
	
	uint16_t RawRange = 0, Range = 0, error_status =0;
	uint16_t RawConfidence = 0, confidence_level =0;
	int status;
	int errcode;
	int confirm=0;
	
	uint16_t IT_verify;

	int confA = 1300;
	int confC = 5600;
	int ItB = 5000;

	thd = Settings[CONFIDENCE_FACTOR];
	limit = Settings[DISTANCE_THD];
	thd_near_mm = Settings[NEAR_LIMIT];

	CCI_I2C_WrByte(dev_t, 0x18, 0x06);
	CCI_I2C_WrByte(dev_t, 0x19, 0x20);
	CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &IT_verify);
	
	confA = Settings[CONFIDENCE10];
	confC = Settings[CONFIDENCE_THD];
	ItB =  Settings[IT];

	init_debug_raw_data();
	
      	status = CCI_I2C_RdWord(dev_t, RESULT_REGISTER, &RawRange);
	if (status < 0)
             	return status;

	debug_raw_range = RawRange;
	Range = (RawRange & DISTANCE_ELISA_MASK) >> 1;

	error_status = RawRange&ERROR_CODE_MASK;

	CCI_I2C_RdWord(dev_t, 0x0A, &RawConfidence);
	if (status < 0)
            	return status;
       
	debug_raw_confidence = RawConfidence;
	confidence_level = (RawConfidence&0x7ff0)>>4;
	
		
	if(RawRange&VALID_DATA){
			
		if((error_status==NO_ERROR)){
			errcode = 0;
		
			if(IT_verify < ItB){
				if(!confirm && confidence_level < confA){
					errcode = RANGE_ERR_NOT_ADAPT;
					Range =	OUT_OF_RANGE;
					confirm = 15;
				}	
			}
			else{
				if(!confirm&&( ItB*(confidence_level) < ItB*confA- (IT_verify - ItB)*(confC - confA) )){
					errcode = RANGE_ERR_NOT_ADAPT;
					Range =	OUT_OF_RANGE;
					confirm = 16;
				}
			}

			if(!confirm && Range == 0){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 8;
			}

			if(!confirm && Range==2047){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 12;
			}
			
			
			if(!confirm && Range < limit && compareConfidence(confidence_level,Range,thd,limit)){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm =9;				
			}

			if(!confirm && Range >= limit && confidence_level >= thd){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;	
				confirm =10;				
			}

			if(!confirm && Range >= limit && confidence_level < thd){
				errcode = RANGE_ERR_NOT_ADAPT;
				Range =	OUT_OF_RANGE;	
				confirm =13;				
			}

			if(!confirm && Range<100 && !g_factory){
				errcode = RANGE_ADAPT_WITH_LESS_ACCURACY;
				confirm =11;					
			}

				
			if(!confirm){
				errcode = RANGE_ADAPT;
				confirm =11;					
			}
			
		}
		else{

			if(error_status==NEAR_FIELD){
				errcode = RANGE_ADAPT;
				Range = 0;
				confirm = 1; 	
			}
			
			if(error_status==FAR_FIELD && confidence_level==0){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 2;
			}
			
			if(error_status==FAR_FIELD && confidence_level > 0){
				errcode = RANGE_ERR_NOT_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 3;
			}
			
			if(error_status==GENERAL_ERROR){
				errcode = RANGE_ERR_NOT_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 4;
			}
			
		}
	} 
	else {
		if(error_status==NEAR_FIELD){
			Range = 0;
			confirm =5;
		}
		else if(error_status==NO_ERROR){
			Range = OUT_OF_RANGE;
			confirm =6;
		}
		else{
			Range = OUT_OF_RANGE;
			confirm =7;
		}
		errcode = RANGE_ERR_NOT_ADAPT;

	}



	ErrCode1 = errcode;
	Range1 = Range;

	if((Laser_log_cnt==LOG_SAMPLE_RATE-1)||(Laser_log_cnt==LOG_SAMPLE_RATE-2)||proc_log_cnt){
		LOG_Handler(LOG_CDBG,"%s: conf(%d)  confA(%d) confC(%d) ItB(%d) IT_verify(0x2006:%d)\n", __func__, confidence_level, confA,confC,ItB,IT_verify);
		LOG_Handler(LOG_CDBG, "%s: status(%d) thd(%d) limit(%d) near(%d) Confidence(%d)\n", __func__,
			error_status>>13, thd, limit, thd_near_mm, confidence_level);
		LOG_Handler(LOG_CDBG, "%s: Range(%d) ErrCode(%d) RawRange(%x) case(%d)\n", __func__, Range, errcode, RawRange, confirm);	
	}

	//reset cnt for case that use both proc and ioctrl
	proc_log_cnt=0;

	return Range;
	

}

int config_conditional_measurement(bool enable)
{
	uint16_t i2c_read_data;
	int status;

	LOG_Handler(LOG_DBG, "%s: %s Conditional Measurement\n", __func__, enable ? "Enable" : "Disable");

	status = CCI_I2C_RdWord(laura_t, CMD_HFCFG3, &i2c_read_data);
	if (status < 0)
   		return -EBUSY;	

	LOG_Handler(LOG_DBG, "%s: ADDR: 0x%02x, value: 0x%04x\n", __func__, CMD_HFCFG3, i2c_read_data);

	if(enable)
		i2c_read_data |= CONDITIONAL_MEASUREMENT_READY_EVT;
	else
		i2c_read_data &= ~CONDITIONAL_MEASUREMENT_READY_EVT;
	
	LOG_Handler(LOG_DBG, "%s: data to write: 0x%04x\n", __func__, i2c_read_data);
	status = CCI_I2C_WrWord(laura_t, CMD_HFCFG3, i2c_read_data);

	return status;

}

extern int Tof_configuration_interface(struct msm_laser_focus_ctrl_t *dev_t, int ctrl);
int Perform_measurement(struct msm_laser_focus_ctrl_t *dev_t)
{
	int status, Range=0;
	bool enable_continuous_measure = timedMeasure && laura_t->continuous_measurement;
	uint16_t sleep_time_ms = 40;
	uint16_t i2c_read_data;
	struct timeval conti_measure_start,now;	
	int elapse_time = 0;

	LOG_Handler(LOG_DBG,"timedMeasure flag = %d, continuous measurement flag = %d\n", timedMeasure, laura_t->continuous_measurement);

	/* Continuous measure */
	if(enable_continuous_measure) {

		/* Set sleep time based on refresh rate */
		sleep_time_ms = laura_t->u16RefreshRate_msec * 100 * 7 / 10;
		
		/* Refresh rate configuration */
		status = CCI_I2C_WrWord(dev_t, CMD_CFG_REGISTER_A, 0XE100 | (laura_t->u16RefreshRate_msec & 0x7F));
		if (status < 0){
		    return status;
		}

		/* Enable Conditional Measurement */
		if(laura_t->conditional_measurement) {
			status = CCI_I2C_RdWord(laura_t, CMD_HFCFG3, &i2c_read_data);
			if (status < 0)
		   		return -EBUSY;

			LOG_Handler(LOG_DBG, "%s: ADDR: 0x%02x, value: 0x%04x\n", __func__, CMD_HFCFG3, i2c_read_data);
			i2c_read_data |= CONDITIONAL_MEASUREMENT_READY_EVT;
			LOG_Handler(LOG_DBG, "%s: data to write: 0x%04x\n", __func__, i2c_read_data);
			status = CCI_I2C_WrWord(laura_t, CMD_HFCFG3, i2c_read_data);
			if (status < 0)
		   		return -EBUSY;
	   	}
		
		/* Continuous measure start */
		status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (CONTINUOUS_MEASURE_START | VALIDATE_CMD));
		if (status < 0){
		    return status;
		}

		O_get_current_time(&conti_measure_start);
		msleep(30);
	}

	do{
		if(!enable_continuous_measure) {			
			/* Trigger single measure */
			status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (SINGLE_MEASURE|VALIDATE_CMD));
			if (status < 0){
			    	return status;
			}
		}

		status = Verify_Range_Data_Ready(dev_t);
		if(status != 0) goto read_err;

		/* if laser has suspended, let it stop working as soon as possible */
		if(!laura_t->suspend) {
			/* Calculate continuous measurement time */
			if(enable_continuous_measure) {
				O_get_current_time(&now);
				DeltaTime_ms(conti_measure_start, now, &elapse_time);
				LOG_Handler(LOG_DBG, "%s: continuous measurement, elapse time = %dms\n", __func__, elapse_time);
				O_get_current_time(&conti_measure_start);
			}

			if(newKdata)
				Range = Read_Range_Data(dev_t);
			else
				Range = Read_Range_Data_OldKdata(dev_t);

			if(Range < 0){
				ErrCode1 = RANGE_ERR_NOT_ADAPT;
				Range1 = OUT_OF_RANGE;
				status = Range;
				goto read_err;
			}

			if(enable_continuous_measure)
				LOG_Handler(LOG_DBG,"continuous measure, distance = %d\n", Range);
		}

		repairing_state = false;		

		if(timedMeasure){
			mutex_ctrl(laura_t, MUTEX_LOCK);
			if(client<=0){
				/* Laser should stop running */
				LaserState = CLOSING_STATE;
				LOG_Handler(LOG_DBG,"ioctrl close, stop measuring\n");
				mutex_ctrl(laura_t, MUTEX_UNLOCK);

				if(enable_continuous_measure) {
					/* Continuous measure stop */
					status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (CONTINUOUS_MEASURE_STOP | VALIDATE_CMD));
					LOG_Handler(LOG_DBG, "Stop continuous measurement, sleep 50ms than let MCPU standby\n");
					msleep(50);
					/* Enable Conditional Measurement */
					if(laura_t->conditional_measurement) {
						status = config_conditional_measurement(false);
					}
				}

				return status;
			} else {
				mutex_ctrl(laura_t, MUTEX_UNLOCK);
				if(enable_continuous_measure) {
					/* Sleep, wait for next data ready */
					msleep(sleep_time_ms);
				} else {
					/* Sleep, wait for next run measurement */
					msleep(40);
				}
			}

			mutex_ctrl(laura_t, MUTEX_LOCK);
			if(laura_t->suspend) {
				mutex_ctrl(laura_t, MUTEX_UNLOCK);

				/* Enter suspend state */
				LOG_Handler(LOG_CDBG, "%s: Laser suspend +++\n", __func__);
				ErrCode1 = RANGE_WARNING_DATA_NOT_READY;
				wait_event(laura_t->suspend_wait_q, !laura_t->suspend);
				LOG_Handler(LOG_CDBG, "%s: Laser suspend ---\n", __func__);
			} else {
				mutex_ctrl(laura_t, MUTEX_UNLOCK);
			}

		} else {
			/* For single measurement, return value directly */
			return Range;
		}
	}while(timedMeasure);


read_err:
	if(enable_continuous_measure) {			
		/* Continuous measure stop */
		CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (CONTINUOUS_MEASURE_STOP | VALIDATE_CMD));

		if(laura_t->conditional_measurement) {
			config_conditional_measurement(false);
		}
		
		WaitMCPUStandby(dev_t);
	}

	LOG_Handler(LOG_ERR, "%s: Exit with Error: %d\n", __func__, status);	
	return status;

}



/** @brief MCPU Contorller
*
*	@param dev_t the laser focus controller
*	@param mode the MCPU go to status
*
*/
int MCPU_Controller(struct msm_laser_focus_ctrl_t *dev_t, int mode){
	int status;

	LOG_Handler(LOG_DBG, "%s: procdure (%d)\n", __func__, mode);
	switch(mode){
		case MCPU_ON:
			status = CCI_I2C_WrWord(dev_t, PMU_CONFIG, (PATCH_MEM_EN|MCPU_INIT_STATE));
			if(status >= 0)
				status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (MCPU_TO_ON|VALIDATE_CMD) /* 0x92 */);
			break;
			
		case MCPU_OFF:
			// Enable patch memory
			status = CCI_I2C_WrWord(dev_t, PMU_CONFIG, (PATCH_MEM_EN|PATCH_CODE_LD_EN));
			if(status >= 0)
				status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (GO_MCPU_OFF|VALIDATE_CMD) /* 0x91 */);
			break;
			
		case MCPU_STANDBY:
			status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (GO_STANDBY|VALIDATE_CMD) /* 0x90 */);
			break;
			
		default:
			LOG_Handler(LOG_ERR, "%s MCPU mode invalid (%d)\n", __func__, mode);
			break;
	}

	udelay(MCPU_DELAY_TIME);
	
	return status;
}

int getTOF(uint16_t* config_normal,uint16_t* config_K10,uint16_t* config_K40,int LAURA_CONFIG_SIZE){

	int buf[3*LAURA_CONFIG_SIZE];
	int i=0; 
	int status=0;

	for(i=0; i<3*LAURA_CONFIG_SIZE; i++)
		buf[i]=0;
	
	status = Sysfs_read_word_seq("/factory/Olivia_conf.txt",buf,3*LAURA_CONFIG_SIZE);
	if(status<0)
		return status;

	for(i=0; i< LAURA_CONFIG_SIZE; i++)
		config_normal[i]=buf[i]&0xffff;

	for(i=0; i< LAURA_CONFIG_SIZE; i++)
		config_K10[i]=buf[i+LAURA_CONFIG_SIZE]&0xffff;

	for(i=0; i< LAURA_CONFIG_SIZE; i++)
		config_K40[i]=buf[i+(2*LAURA_CONFIG_SIZE)]&0xffff;

	return 0;




}


/** @brief Initialize Laura tof configure
*
*	@param dev_t the laser focus controller
*	@param config the configuration param
*
*/
int settingTOF(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *config)
{
	int status = 0;
	uint16_t tempdata;
	char op_mode_str[30] = "";
	
	LOG_Handler(LOG_DBG, "%s: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n", __func__,
		config[0], config[1], config[2], config[3], config[4], config[5]);

	/* Change the default VCSEL threshold and VCSEL peak */
       	status = CCI_I2C_WrWord(dev_t, 0x0C, config[0]);

	if(status >= 0)  
       	status = CCI_I2C_WrWord(dev_t, 0x0E, config[1]);

	if(status >= 0) {
		switch(dev_t->operation_mode) {
			case OP_MODE_LONG:
				strncpy(op_mode_str, "Long Mode", sizeof(op_mode_str));
				break;
			case OP_MODE_DEFAULT:
				strncpy(op_mode_str, "Default Mode", sizeof(op_mode_str));
				break;
			case OP_MODE_HIGH_SPEED:
				strncpy(op_mode_str, "High Speed Mode", sizeof(op_mode_str));
				break;
			case OP_MODE_HIGH_ACCURACY:
				strncpy(op_mode_str, "High Accuracy Mode", sizeof(op_mode_str));
				break;
			case OP_MODE_CUSTOM:
				strncpy(op_mode_str, "Custom Mode", sizeof(op_mode_str));
				break;
			default:
				LOG_Handler(LOG_ERR, "%s: %d is not a valid number for operation mode, set it to Custom Mode\n", __func__, dev_t->operation_mode);
				dev_t->operation_mode = OP_MODE_CUSTOM;
				return -EFAULT;
		}

		LOG_Handler(LOG_DBG, "%s: TOF config will set Operation Mode: %s\n", __func__, op_mode_str);

		tempdata = config[2];
		tempdata &= ~OPERATION_MODE;
		LOG_Handler(LOG_DBG, "%s: after mask opration mode, ADDR: 0x%02x, value: 0x%04x\n", __func__, CMD_HFCFG0, tempdata);
		tempdata |= dev_t->operation_mode << OP_BIT_SHIFT;
		LOG_Handler(LOG_DBG, "%s: data to write: 0x%04x\n", __func__, tempdata);
		status = CCI_I2C_WrWord(dev_t, 0x20, tempdata);	
	}

	if(status >= 0)    
		status = CCI_I2C_WrWord(dev_t, 0x22, config[3]);
	
	if(status >= 0)  
		status = CCI_I2C_WrWord(dev_t, 0x24, config[4]);

	if(status >= 0)  
		status = CCI_I2C_WrWord(dev_t, 0x26, config[5]);
 
	return status;
} 


int WaitMCPUOn(struct msm_laser_focus_ctrl_t *dev_t){
	uint16_t i2c_read_data = 0;
	int status=0;	
	struct timeval start,now;
	O_get_current_time(&start);

	MCPU_Controller(dev_t, MCPU_ON);
	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0)
       		break;
       		
		//include MCPU_ON
		if(i2c_read_data == STATUS_MEASURE_ON){
			LOG_Handler(LOG_DBG, "%s: in MCPU_ON\n", __func__);		
			break;
		}

		 O_get_current_time(&now);
		if(is_timeout(start,now,MCPU_SWITCH_TIMEOUT_ms)){
			LOG_Handler(LOG_ERR, "%s:  time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
			status = -TIMEOUT_VAL;
			break;            	
		}             
	}	

	return status;
}

#define	MAX_RETRY_COUNT		3
int WaitMCPUStandby(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t i2c_read_data = 0;
	int cnt = 0;
	int elapse_time = 0;

	struct timeval start, now;
	O_get_current_time(&start);

	/* Wait MCPU standby */
	MCPU_Controller(dev_t, MCPU_STANDBY);
	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0)	break;

		i2c_read_data &= STATUS_MASK;
		if(i2c_read_data == STATUS_STANDBY){
			O_get_current_time(&now);
			DeltaTime_ms(start, now, &elapse_time);
			LOG_Handler(LOG_DBG, "%s: in STANDBY MODE, reg(0x06): 0x%x, retry count: %d, elapse time: %dms\n", 
								__func__, i2c_read_data, cnt, elapse_time);
			return status;
		}

		/* Check timeout and retry */
		O_get_current_time(&now);
		if(is_timeout(start,now,MCPU_SWITCH_TIMEOUT_ms)){
			LOG_Handler(LOG_ERR, "%s:  time out - register(0x06): 0x%x\n", __func__, i2c_read_data);

			cnt++;
			if(cnt >= MAX_RETRY_COUNT - 1) {
				if(cnt >= MAX_RETRY_COUNT)
					return -TIMEOUT_VAL;

				/* Retry failed, force MCPU to standby */
				LOG_Handler(LOG_ERR, "%s: force MCPU standby\n", __func__);

				status = CCI_I2C_WrWord(dev_t, 0x1E, BIT(1));
				if(status > 0)
					status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (GO_STANDBY|VALIDATE_CMD) /* 0x90 */);
			} else {
				MCPU_Controller(dev_t, MCPU_STANDBY);
			}
			LOG_Handler(LOG_ERR, "%s:  retry %d times, reg(0x06): 0x%x\n", __func__, cnt, i2c_read_data);
			O_get_current_time(&start);
		}
		usleep_range(1000, 2000);

	}

	return status;
}

int WaitMCPUOff(struct msm_laser_focus_ctrl_t *dev_t){

	int status = 0;
	uint16_t i2c_read_data = 0;
	struct timeval start, now;
	O_get_current_time(&start);

	// Set then Verify status is MCPU off 
	MCPU_Controller(dev_t, MCPU_OFF);
	while(1){		
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0)
			break;

		i2c_read_data &= STATUS_MASK;
		if(i2c_read_data == STATUS_MCPU_OFF){
			LOG_Handler(LOG_DBG, "%s: in OFF MODE, reg(0x06): 0x%x\n", __func__, i2c_read_data);					
			break;
		}

		if(is_timeout(start,now,MCPU_SWITCH_TIMEOUT_ms)){
			LOG_Handler(LOG_ERR, "%s: time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
			status = -TIMEOUT_VAL;
			break;
		}		
		udelay(500);
	}

	return status;
}


/** @brief Configure i2c interface
*
*	@param dev_t the laser focus controller
*
*/
int Config_I2C_Interface(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Configure I2C interface */
	//include enable auto-increment
	status = CCI_I2C_WrByte(dev_t, 0x1C, 0x65);
	if (status < 0){
	       return status;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return status;
}

/** @brief Power up initialization without applying calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Laura_No_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	status = MCPU_Controller(dev_t, MCPU_ON);
	return status;
}

/** @brief Power up initialization which apply calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Apply_Calibration_Data(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;

	status = WaitMCPUOff(dev_t);

	/* Load calibration data */
	Olivia_device_Load_Calibration_Value(dev_t);

	return status;
}

//update: Libra ER
#define VER_MAJOR	1
#define VER_MINOR	9

void Verify_FW_Version(void){

	LOG_Handler(LOG_CDBG,"FW version %d.%d\n",FW_version[0],FW_version[1]);
	if(FW_version[0] != VER_MAJOR || FW_version[1] != VER_MINOR)
		LOG_Handler(LOG_ERR,"FW vesrion not %d.%d\n",VER_MAJOR,VER_MINOR);

	laura_t->fw_version = (FW_version[0] * 1000) + FW_version[1];

	/* Set Elisa 1.9 flag */
	if(laura_t->fw_version == 1009)
		laura_t->elisa_fw19 = true;
	else
		laura_t->elisa_fw19 = false;
		
	LOG_Handler(LOG_CDBG,"FW vesrion = %d.%d, fw_version = %d, fw19 flag = %s\n", 
					FW_version[0], FW_version[1], laura_t->fw_version, laura_t->elisa_fw19 ? "true" : "false");

}


/** @brief Get module id from chip
*
*       @param dev_t the laser focus controller
*
*/
#define MODULE_ID_LEN	34
void Get_ModuleID(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0, i = 0;

	if(!module_id_flag){	
		MCPU_Controller(dev_t, MCPU_OFF);

		status = CCI_I2C_WrByte(dev_t, 0x18, 0xc0);
		status = CCI_I2C_WrByte(dev_t, 0x19, 0xff);
		//status = CCI_I2C_WrWord(dev_t, 0x18, 0xffc0);

		CCI_I2C_RdByte(dev_t, 0x1A, &FW_version[0]);
		CCI_I2C_RdByte(dev_t, 0x1A, &FW_version[1]);

		FW_version[0] &= 0x001f;
		FW_version[1] &= 0x000f;

		Verify_FW_Version();
		
		status = CCI_I2C_WrByte(dev_t, 0x18, 0x04);
		status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);
		//status = CCI_I2C_WrWord(dev_t, 0x18, 0xc804);

		for(i = 0; i < MODULE_ID_LEN; i++){
			CCI_I2C_RdByte(dev_t, 0x1A, &module_id[i]);
		}

		LOG_Handler(LOG_CDBG,"Module ID(Hex):%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x\n",
	                	module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
	                	module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
	                	module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
	                	module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
	                	module_id[32],module_id[33]);

		module_id_flag=true;
	}

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
}

/** @brief Get Chip from driver
*
*       @param dev_t the laser focus controller
*       @param vfile
*
*/
void Laura_Get_Module_ID(struct msm_laser_focus_ctrl_t *dev_t, struct seq_file *vfile){



	if(!module_id_flag){
		MCPU_Controller(dev_t, MCPU_OFF);
		Get_ModuleID(dev_t);	
	}
	else{
		LOG_Handler(LOG_DBG,"Module ID(Hex):%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x\n",
                	module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
                	module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
                	module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
                	module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
                	module_id[32],module_id[33]);
	}

	if(vfile!=NULL){
                seq_printf(vfile,"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
                        module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
                        module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
                        module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
                        module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
                        module_id[32],module_id[33]);

		seq_printf(vfile,"FW vesrion: %d.%d\n",FW_version[0],FW_version[1]);		
        }

}


int Olivia_DumpKdata(struct msm_laser_focus_ctrl_t *dev_t, int16_t* cal_data, uint16_t len ){
	int status = 0, i=0;
	uint16_t i2c_read_data;
	uint16_t len_byte = len*2;
	MCPU_Controller(dev_t, MCPU_STANDBY);
msleep(1);
	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }	
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);

	
	MCPU_Controller(dev_t, MCPU_OFF);	
	msleep(2);
	
	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }	
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);

	status = CCI_I2C_WrByte(dev_t, 0x18, 0x48);
	status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);
	
	for(i=0; i < len_byte; i++){
		CCI_I2C_RdByte(dev_t, 0x1A, &f0_data[i]);
	}

	for(i=0;i < len; i++){
		cal_data[i] = ((f0_data[2*i] & 0xFF) | (f0_data[2*i+1] & 0xFF) << 8);
	}
	
		LOG_Handler(LOG_DBG,"dump part f0_data:%04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  \n",
                	cal_data[0],cal_data[1],cal_data[2],cal_data[3],cal_data[4],cal_data[5],cal_data[6],cal_data[7],
                	cal_data[8],cal_data[9],cal_data[10]);

	return status;
}
