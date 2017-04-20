#ifndef _SHARE_DATA_H_
#define _SHARE_DATA_H_

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

void share_data(uint8_t c, mavlink_message_t* msg, mavlink_status_t* status);
void handleMessage(mavlink_message_t* msg);
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

#endif
