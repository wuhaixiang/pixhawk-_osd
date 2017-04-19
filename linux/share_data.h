/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include <mavlink.h>

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

/* mavlink_heartbeat_t         	heartbeat;			//飞行模式
mavlink_attitude_t          	attitude;			//横滚 俯仰 偏航
//mavlink_global_position_int_t 	position;
mavlink_sys_status_t         	sys_status;			//电压 电流
mavlink_vfr_hud_t            	vfr_hud;			//高度 地速 空速 下降速度率
//mavlink_statustext_t        	statustext_t;
mavlink_gps_input_t 			gps_input;			//卫星数量
mavlink_radio_status_t			radio_status;		//电台信号质量
mavlink_rc_channels_raw_t		rc_channels_raw;	//遥控信号
mavlink_local_position_ned_t	local_position_ned;	//离家距离 */


void copy_data_to_share(mavlink_message_t);
uint16_t get_voltage_battery(void);
int16_t get_current_battery(void);
float get_altitude(void);
float get_groundspeed(void);
float get_airspeed(void);
float get_distance_from_home(void);
uint8_t get_satellites_visible(void);
/* flight mode */
void get_chan_x_raw(uint16_t *channel_x);
uint8_t get_rssi(void);
float get_climb(void);
float get_roll(void);
float get_pitch(void);
float get_yaw(void);