/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "laser_parser.h"
#include "laser_log.h"

#if 0
/** @brief Parse GPIO information from dtsi
*	
*	@param of_node the device node
*	@param sensordata the sensor board information
*
*/
int32_t dtsi_gpio_parser(struct device_node *of_node, struct msm_camera_sensor_board_info *sensordata)
{
	int i = 0;
	int32_t rc = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	power_info = &sensordata->power_info;
	
	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf), GFP_KERNEL);
	if(!power_info->gpio_conf){
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}

	gconf = power_info->gpio_conf;
	
	gpio_array_size = of_gpio_count(of_node);
	LOG_Handler(LOG_DBG, "%s: gpio count %d\n", __func__, gpio_array_size);

	if(gpio_array_size){
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size, GFP_KERNEL);
		if(!gpio_array){
			LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
			kfree(gconf);
			rc = -ENOMEM;
			return rc;
		}
		
		for(i=0; i < gpio_array_size; i++){
			gpio_array[i] = of_get_gpio(of_node, i);
			LOG_Handler(LOG_DBG, "%s: gpio_array[%d] = %d\n", __func__, i, gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
			kfree(gconf);
			return rc;
		}
//+++[Vincent][Remove]
#if 0
		rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
			kfree(gconf->cam_gpio_req_tbl);
			return rc;
		}
#endif
//---[Vincent][Remove]
		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
			//kfree(gconf->cam_gpio_set_tbl);
			return rc;
		}
	}
	kfree(gpio_array);

	////LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}
#endif /* Vincent Remove cci */



/** @brief Parse information from dtsi
*	
*	@param of_node the device node
*	@param dev_t the laser focus controller
*
*/
int32_t get_dtsi_data(struct device_node *of_node, struct msm_laser_focus_ctrl_t *dev_t)
{
	int32_t rc = 0;

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Check device node */
	if (!of_node) {
		LOG_Handler(LOG_ERR, "%s: of_node NULL\n", __func__);
		return -EINVAL;
	}

	/* Get subdev id information */
	rc = of_property_read_u32(of_node, "chipid", &dev_t->sensor_id);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: failed\n", __func__);
		return -EINVAL;
	}
	LOG_Handler(LOG_DBG, "%s: chip id =  0x%x\n", __func__, dev_t->sensor_id);

	/* Get label(sensor name) information */
	rc = of_property_read_string(of_node, "label", &dev_t->sensor_name);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		goto ERROR;
	}
	LOG_Handler(LOG_DBG, "%s: label = %s, rc = %d\n", __func__, dev_t->sensor_name, rc);

//+++[Vincent][Remove]
#if 0
	/* Handle GPIO (e.g. CAM_1V2_EN) */
	rc = dtsi_gpio_parser(of_node, sensordata);
	if(rc < 0){
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		goto ERROR;
	}

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
#endif
//---[Vincent][Remove]

ERROR:

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}
