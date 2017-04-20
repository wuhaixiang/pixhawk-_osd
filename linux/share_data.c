#include "share_data.h"

mavlink_heartbeat_t         	heartbeat;			
mavlink_attitude_t          	attitude;			
mavlink_sys_status_t         	sys_status;			
mavlink_vfr_hud_t            	vfr_hud;			
mavlink_gps_input_t 			gps_input;			
mavlink_radio_status_t			radio_status;		
mavlink_rc_channels_raw_t		rc_channels_raw;	
mavlink_local_position_ned_t	local_position_ned;	

void share_data(uint8_t c, mavlink_message_t* msg, mavlink_status_t* status) {
	if (mavlink_parse_char(MAVLINK_COMM_0, c, msg, status)) {
		// Packet received
		printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg->sysid, msg->compid, msg->len, msg->msgid);
		handleMessage(msg);
	}
}

void handleMessage(mavlink_message_t* msg)
{
	switch (msg->msgid) {
		
        case MAVLINK_MSG_ID_HEARTBEAT: {
			mavlink_msg_heartbeat_decode(msg, &heartbeat);
            break;
        }
		
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_msg_sys_status_decode(msg, &sys_status);
            break;
        }
		
		case MAVLINK_MSG_ID_ATTITUDE: {
			mavlink_msg_attitude_decode(msg, &attitude);
			break;
		}
		
	    case MAVLINK_MSG_ID_VFR_HUD: {
			mavlink_msg_vfr_hud_decode(msg, &vfr_hud);
			break;
		}
		
		case MAVLINK_MSG_ID_GPS_INPUT: {
			mavlink_msg_gps_input_decode(msg, &gps_input);
			break;
		}
		
		case MAVLINK_MSG_ID_RADIO_STATUS: {
			mavlink_msg_radio_status_decode(msg, &radio_status);
			break;
		}
		
		case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
			mavlink_msg_rc_channels_raw_decode(msg, &rc_channels_raw);
			break;
		}
		
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
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
