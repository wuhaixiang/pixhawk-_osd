#ifndef _SHARE_DATA_H_
#define _SHARE_DATA_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#if (defined __QNX__) | (defined __QNXNTO__)
#include <unix.h>
#else
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif
#include <mavlink.h>

#define deffer 57.29576
 
void updata(uint8_t c, mavlink_message_t* msg, mavlink_status_t* status);
void heartbeat_set_timeout(void);
void heartbeat_timeout(int signo);
void handleMessage(mavlink_message_t* msg);

uint16_t get_voltage_battery(void);
int8_t get_battery_remaining(void);
int16_t get_current_battery(void);
float get_altitude(void);
float get_groundspeed(void);
float get_airspeed(void);
float get_climb(void);
float get_distance_from_home(void);
uint8_t get_satellites_visible(void);
void get_chan_x_raw(uint16_t *channel_x);
uint8_t get_rssi(void);
float get_roll(void);
float get_pitch(void);
float get_yaw(void);
void get_flight_mode(char *base_mode_buf, char *custom_mode_buf);

void get_ArduPlane_flight_mode(char *base_mode_buf, char *custom_mode_buf);
void get_APM_flight_mode(char *base_mode_buf, char *custom_mode_buf);
void get_PX4_flight_mode(char *base_mode_buf, char *custom_mode_buf);

#endif
