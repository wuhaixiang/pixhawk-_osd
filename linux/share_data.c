#include "share_data.h"

mavlink_heartbeat_t         	heartbeat;			//飞行模式
mavlink_attitude_t          	attitude;			//横滚 俯仰 偏航
//mavlink_global_position_int_t 	position;
mavlink_sys_status_t         	sys_status;			//电压 电流
mavlink_vfr_hud_t            	vfr_hud;			//高度 地速 空速 下降速度率
//mavlink_statustext_t        	statustext_t;
mavlink_gps_input_t 			gps_input;			//卫星数量
mavlink_radio_status_t			radio_status;		//电台信号质量
mavlink_rc_channels_raw_t		rc_channels_raw;	//遥控信号
mavlink_local_position_ned_t	local_position_ned;	//离家距离

void copy_data_to_share(mavlink_message_t* msg)
{
	switch (msg->msgid) {
		/*飞行模式*/
        case MAVLINK_MSG_ID_HEARTBEAT: {
			mavlink_msg_heartbeat_decode(msg, &heartbeat);
            break;
        }
		/*电压 电流*/
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_msg_sys_status_decode(msg, &sys_status);
            break;
        }
		/*横滚 俯仰 偏航*/
		case MAVLINK_MSG_ID_ATTITUDE: {
			mavlink_msg_attitude_decode(msg, &attitude);
			break;
		}
		/*高度 地速 空速 下降速度率*/
	    case MAVLINK_MSG_ID_VFR_HUD: {
			mavlink_msg_vfr_hud_decode(msg, &vfr_hud);
			break;
		}
		/*卫星数量*/
		case MAVLINK_MSG_ID_GPS_INPUT: {
			mavlink_msg_gps_input_decode(msg, &gps_input);
			break;
		}
		/*电台信号质量*/
		case MAVLINK_MSG_ID_RADIO_STATUS: {
			mavlink_msg_radio_status_decode(msg, &radio_status);
			break;
		}
		/*遥控信号*/
		case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
			mavlink_msg_rc_channels_raw_decode(msg, &rc_channels_raw);
			break;
		}
		/*离家距离*/
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED {
			mavlink_msg_local_position_ned_decode(msg, &local_position_ned);
			break;
		}
		default:
			break;
    }
}

uint16_t get_voltage_battery(void) {
	return sys_status.voltage_battery;
}

int16_t get_current_battery(void) {
	return sys_status.current_battery;
}

float get_altitude(void) {
	return vfr_hud.alt;
}

float get_groundspeed(void) {
	return vfr_hud.groundspeed;
}

float get_airspeed(void) {
	return vfr_hud.airspeed;
}

float get_distance_from_home(void) {
	return sqrt((local_position_ned.x)*(local_position_ned.x)
			  + (local_position_ned.y)*(local_position_ned.y)
			  + (local_position_ned.z)*(local_position_ned.z));
}

uint8_t get_satellites_visible(void) {
	return gps_input.satellites_visible;
}

/* flight mode */

void get_chan_x_raw(uint16_t *channel_x) {
	uint16_t *channel_x_value = &(rc_channels_raw.chan1_raw);
	int i;
	for (i = 0; i < 8; i++) {
		*channel_x = *channel_x_value;
		channel_x_value++, channel_x++;
	}
	return;
}

uint8_t get_rssi(void) {
	return radio_status.rssi;
}

float get_climb(void) {
	return vfr_hud.climb;
}

float get_roll(void) {
	return attitude.roll;

}

float get_pitch(void) {
	return attitude.pitch;
}

float get_yaw(void) {
	return attitude.yaw;
}