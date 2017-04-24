#include "share_data.h"

struct itimerval timeout;

mavlink_heartbeat_t         	heartbeat;			
mavlink_attitude_t          	attitude;			
mavlink_sys_status_t         	sys_status;			
mavlink_vfr_hud_t            	vfr_hud;			
mavlink_gps_raw_int_t			gps_raw_int;
mavlink_radio_status_t			radio_status;		
mavlink_rc_channels_raw_t		rc_channels_raw;	
mavlink_local_position_ned_t	local_position_ned;	

void updata(uint8_t c, mavlink_message_t* msg, mavlink_status_t* status)
{
	heartbeat_set_timeout();

	if (mavlink_parse_char(MAVLINK_COMM_0, c, msg, status)) {
		handleMessage(msg);
	}
	
	return ;
}

void heartbeat_set_timeout(void)
{
	signal(SIGALRM, heartbeat_timeout);
    memset(&timeout, 0, sizeof(timeout));

    //Timeout to run first time
    timeout.it_value.tv_sec = 1;
    timeout.it_value.tv_usec = 0;

    //After first, the Interval time for clock
    timeout.it_interval.tv_sec = 1;
    timeout.it_interval.tv_usec = 0;

    if(setitimer(ITIMER_REAL, &timeout, NULL) < 0) {
		exit(-1);
	}
	
	return ;
}

void heartbeat_timeout(int signo)
{
	memset(&heartbeat, 0, sizeof(heartbeat));
	memset(&attitude, 0, sizeof(attitude));
	memset(&sys_status, 0, sizeof(sys_status));
	memset(&vfr_hud, 0, sizeof(vfr_hud));
	memset(&gps_raw_int, 0, sizeof(gps_raw_int));
	memset(&radio_status, 0, sizeof(radio_status));
	memset(&rc_channels_raw, 0, sizeof(rc_channels_raw));
	memset(&local_position_ned, 0, sizeof(local_position_ned));
	
	return ;
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
		
		case MAVLINK_MSG_ID_GPS_RAW_INT: {
			mavlink_msg_gps_raw_int_decode(msg, &gps_raw_int);
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
	return ;
}

uint16_t get_voltage_battery(void)
{
	return sys_status.voltage_battery;
}

int8_t get_battery_remaining(void)
{
	return sys_status.battery_remaining;
}

int16_t get_current_battery(void)
{
	return sys_status.current_battery;
}

float get_altitude(void)
{
	return vfr_hud.alt;
}

float get_groundspeed(void)
{
	return vfr_hud.groundspeed;
}

float get_airspeed(void)
{
	return vfr_hud.airspeed;
}

float get_climb(void)
{
	return vfr_hud.climb;
}

float get_distance_from_home(void)
{
	return sqrt((local_position_ned.x)*(local_position_ned.x)
			  + (local_position_ned.y)*(local_position_ned.y)
			  + (local_position_ned.z)*(local_position_ned.z));
}

uint8_t get_satellites_visible(void)
{
	return gps_raw_int.satellites_visible;
}

void get_chan_x_raw(uint16_t *channel_x)
{
	uint16_t *channel_x_value = &(rc_channels_raw.chan1_raw);
	int i;
	
	for (i = 0; i < 8; i++) {
		*channel_x = *channel_x_value;
		channel_x_value++, channel_x++;
	}
	
	return ;
}

uint8_t get_rssi(void)
{
	return radio_status.rssi;
}

float get_roll(void)
{
	return (attitude.roll * deffer);

}

float get_pitch(void)
{
	return (attitude.pitch * deffer);
}

float get_yaw(void)
{
	if (attitude.yaw < 0) {
		return (attitude.yaw * deffer + 360);
	}
	return (attitude.yaw * deffer);
}

void get_flight_mode(char *base_mode_buf, char *custom_mode_buf)
{
	if ((heartbeat.type == 1) && (heartbeat.autopilot == 3)) {
		//printf("arduplane\n");
		get_ArduPlane_flight_mode(base_mode_buf, custom_mode_buf);
	}
	if ((heartbeat.type == 2) && (heartbeat.autopilot == 3)) {
		//printf("apm\n");
		get_APM_flight_mode(base_mode_buf, custom_mode_buf);
	}
	if (heartbeat.autopilot == 12) {
		//printf("PX4\n");
		get_PX4_flight_mode(base_mode_buf, custom_mode_buf);
	}
	
	return ;
}

void get_ArduPlane_flight_mode(char *base_mode_buf, char *custom_mode_buf)
{
	if (heartbeat.base_mode == 81) {
		
		strcpy(base_mode_buf, "81");
		
		switch (heartbeat.custom_mode) {
			case 0 : {
				//manual
				strcpy(custom_mode_buf, "manual");
				break;
			}
			case 2 : {
				//stabilized
				strcpy(custom_mode_buf, "stabilized");
				break;
			}
			case 3 : {
				//traning
				strcpy(custom_mode_buf, "traning");
				break;
			}
			case 4 : {
				//acro
				strcpy(custom_mode_buf, "acro");
				break;
			}
			case 5 : {
				//FWB A
				strcpy(custom_mode_buf, "FWB A");
				break;
			}
			case 6 : {
				//FWB B
				strcpy(custom_mode_buf, "FWB B");
				break;
			}
			case 7 : {
				//cruise
				strcpy(custom_mode_buf, "cruise");
				break;
			}
			case 8 : {
				//autotune
				strcpy(custom_mode_buf, "autotune");
				break;
			}
			default : {
				strcpy(custom_mode_buf, "unknow");
				break;
			}
		}
	}
	else if (heartbeat.base_mode == 89) {
		
		strcpy(base_mode_buf, "89");
		
		switch (heartbeat.custom_mode) {
			case 1 : {
				//circle
				strcpy(custom_mode_buf, "circle");
				break;
			}
			case 11 : {
				//RTL
				strcpy(custom_mode_buf, "RTL");
				break;
			}
			case 12 : {
				//loiter
				strcpy(custom_mode_buf, "loiter");
				break;
			}
			case 15 : {
				//guided
				strcpy(custom_mode_buf, "guided");
				break;
			}
			default : {
				strcpy(custom_mode_buf, "unknow");
				break;
			}
		}
	}
	else {
		strcpy(base_mode_buf, "unknow");
	}
	
	return ;
}

void get_APM_flight_mode(char *base_mode_buf, char *custom_mode_buf)
{
	if (heartbeat.base_mode == 81) {
		
		strcpy(base_mode_buf, "81");
		
		switch (heartbeat.custom_mode) {
			case 0 : {
				//stabilized
				strcpy(custom_mode_buf, "stabilized");
				break;
			}
			case 1 : {
				//acro
				strcpy(custom_mode_buf, "acro");
				break;
			}
			case 2 : {
				//alt hold
				strcpy(custom_mode_buf, "alt hold");
				break;
			}
			case 9 : {
				//land
				strcpy(custom_mode_buf, "land");
				break;
			}
			case 11 : {
				//drift
				strcpy(custom_mode_buf, "drift");
				break;
			}
			case 13 : {
				//sport
				strcpy(custom_mode_buf, "sport");
				break;
			}
			case 14 : {
				//flip
				strcpy(custom_mode_buf, "flip");
				break;
			}
			default : {
				strcpy(custom_mode_buf, "unknow");
				break;
			}
		}
	}
	else if (heartbeat.base_mode == 89) {
		
		strcpy(base_mode_buf, "89");
		
		switch (heartbeat.custom_mode) {
			case 14 : {
				//flip
				strcpy(custom_mode_buf, "flip");
				break;
			}
			case 3 : {
				//auto
				strcpy(custom_mode_buf, "auto");
				break;
			}
			case 4 : {
				//guided
				strcpy(custom_mode_buf, "guided");
				break;
			}
			case 5 : {
				//loiter
				strcpy(custom_mode_buf, "loiter");
				break;
			}
			case 6 : {
				//RTL
				strcpy(custom_mode_buf, "RTL");
				break;
			}
			case 7 : {
				//circle
				strcpy(custom_mode_buf, "circle");
				break;
			}
			case 16 : {
				//pos hold
				strcpy(custom_mode_buf, "pos hold");
				break;
			}
			case 17 : {
				//brake
				strcpy(custom_mode_buf, "brake");
				break;
			}
			default : {
				strcpy(custom_mode_buf, "unknow");
				break;
			}
		}
	}
	else {
		strcpy(base_mode_buf, "unknow");
	}
	
	return ;
}

void get_PX4_flight_mode(char *base_mode_buf, char *custom_mode_buf)
{
	sprintf(base_mode_buf, "%d", heartbeat.base_mode);
	
	switch (heartbeat.custom_mode) {
		case 65536 : {
			//manual
			strcpy(custom_mode_buf, "manual");
			break;
		}
		case 458752 : {
			//stabilized
			strcpy(custom_mode_buf, "stabilized");
			break;
		}
		case 327680 : {
			//acro
			strcpy(custom_mode_buf, "acro");
			break;
		}
		case 524288 : {
			//rattitude
			strcpy(custom_mode_buf, "rattitude");
			break;
		}
		case 131072 : {
			//altitude
			strcpy(custom_mode_buf, "altitude");
			break;
		}
		case 196608 : {
			//position
			strcpy(custom_mode_buf, "position");
			break;
		}
		case 117702656 : {
			//hold
			strcpy(custom_mode_buf, "hold");
			break;
		}
		case 67371008 : {
			//mission
			strcpy(custom_mode_buf, "mission");
			break;
		}
		case 84148224 : {
			//return
			strcpy(custom_mode_buf, "return");
			break;
		}
		case 134479872 : {
			//followme
			strcpy(custom_mode_buf, "followme");
			break;
		}
		default : {
			strcpy(custom_mode_buf, "unknow");
			break;
		}
	}
	return ;
}