#ifndef __LINUX_SHOW_SENSOR_PARSER_H
#define __LINUX_SHOW_SENSOR_PARSER_H

#include "msm_laser_focus.h"

/* Parse information from dtsi */
int32_t get_dtsi_data(struct device_node *of_node, struct msm_laser_focus_ctrl_t *fctrl);

#endif
