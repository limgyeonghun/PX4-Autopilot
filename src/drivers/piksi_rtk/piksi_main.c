/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file piksi.c
 * @author SungTae Moon <munhoey@gmail.com>
 *
 * Piksi RTK Driver
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
// #include <systemlib/systemlib.h>
#include <termios.h>
#include <drivers/drv_hrt.h>
#include <math.h>

// #include <px4_tasks.h>

#include <uORB/uORB.h>
#include <uORB/topics/piksi_rtk.h>
#include <uORB/topics/piksi_data.h>
//#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sensor_gps.h>
#include <libsbp/piksi.h>
#include <libsbp/edc.h>
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>


#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)

#define PERIOD_ARRAY_SIZE (10)			// ( 2000 ms (required logging time) / 200ms(default receive period from base) = 10)
#define MAX_RECV_TIMEOUT (3000000)		// 3 sec
#define MAX_OBS_PACKET_NUM      (10)

typedef struct {
	uint32_t tow;                           // TOW of obs
	uint8_t total;                          // the number of total packet
	uint8_t	obs[MAX_OBS_PACKET_NUM];        // the number of obs in packet
} obs_info_t;

/*
 * State of the SBP message parser.
 * Must be statically allocated.
 */
sbp_state_t _sbp_state;

/* SBP structs that messages from Piksi will feed. */
msg_baseline_ned_t	_baseline_ned;
msg_heartbeat_t		_heartbeat;
msg_iar_state_t		_iar_state;
msg_obs_t		_obs;
msg_gps_time_t		_gps_time;
msg_pos_llh_t       	_pos_llh;
msg_vel_ned_t       	_vel_ned;
msg_utc_time_t      	_utc_time;
msg_age_corrections_t	_age_corr;
msg_dops_t	            _dops;

hrt_abstime         	_pos_llh_time = 0;
hrt_abstime         	_vel_ned_time = 0;

sbp_msg_callbacks_node_t _heartbeat_callback_node;
sbp_msg_callbacks_node_t _baseline_ned_callback_node;
sbp_msg_callbacks_node_t _iar_state_callback_node;
sbp_msg_callbacks_node_t _obs_callback_node;
sbp_msg_callbacks_node_t _obs_dep_a_callback_node;
sbp_msg_callbacks_node_t _gps_time_callback_node;
sbp_msg_callbacks_node_t _pos_llh_callback_node;
sbp_msg_callbacks_node_t _vel_ned_callback_node;
sbp_msg_callbacks_node_t _utc_callback_node;
sbp_msg_callbacks_node_t _age_corr_callback_node;
sbp_msg_callbacks_node_t _dops_callback_node;

u16 _piksi_id = 0;
u8 _n_base_sat = 0;					// satellite number from base
u8 _n_base_sat_temp = 0;			// satellite number from base for temporary count

u8 _n_rover_sat = 0;				// satellite number from rover
u8 _n_rover_sat_temp = 0;			// satellite number from rover for temporary count

hrt_abstime _base_recv_time = 0;
hrt_abstime _period_for_base = 0;
hrt_abstime _arr_period_for_base[PERIOD_ARRAY_SIZE] = {0,};

hrt_abstime _rover_recv_time = 0;
hrt_abstime _period_for_rover = 0;

/* uart fd */
static int _sbp_uart = 0;

struct piksi_rtk_s	_piksi_rtk;
struct sensor_gps_s	_vehicle_gps_pos;
struct tm _timeinfo;

orb_advert_t		_piksi_rtk_pub = 0;
orb_advert_t		_vehicle_gps_pos_pub = 0;

obs_info_t          	_obs_info;

uint8_t             _fix_type = 0;
uint64_t            _utc_usec = 0;

/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int piksi_task;

/* functions */
void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void iar_state_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void obs_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void obs_dep_a_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void utc_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void age_corr_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void dops_callback(u16 sender_id, u8 len, u8 msg[], void *context);

int publish_vehicle_gps_pos(void);

// static s32 piksi_port_read(u8 *buff, u32 n, void *context);
static int open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original);
static void usage(void);
static void status(void);
static int piksi_thread_main(int argc, char *argv[]);

__EXPORT int piksi_rtk_main(int argc, char *argv[]);


/* TEST */
uint32_t _test_count = 0;
uint32_t _error_count = 0;

void dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    _dops = *(msg_dops_t *)msg;
}

void iar_state_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	_iar_state = *(msg_iar_state_t *)msg;
}

void age_corr_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	_age_corr = *(msg_age_corrections_t *)msg;
}

void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	_baseline_ned = *(msg_baseline_ned_t *)msg;
}

void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	if (_piksi_id == 0) { _piksi_id = sender_id; }

	_heartbeat = *(msg_heartbeat_t *)msg;
}

void obs_dep_a_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{

}

void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	_gps_time = *(msg_gps_time_t *)msg;
}

void utc_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	_utc_time = *(msg_utc_time_t *)msg;

	_timeinfo.tm_year	= _utc_time.year - 1900;
	_timeinfo.tm_mon	= _utc_time.month - 1;
	_timeinfo.tm_mday	= _utc_time.day;
	_timeinfo.tm_hour	= _utc_time.hours;
	_timeinfo.tm_min	= _utc_time.minutes;
	_timeinfo.tm_sec	= _utc_time.seconds;
	_timeinfo.tm_isdst	= 0;

    time_t epoch = mktime(&_timeinfo);
    _utc_usec = (uint64_t)epoch * 1000000ULL + (uint64_t)(_utc_time.ns/1000);
}

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	_pos_llh = *(msg_pos_llh_t *)msg;
	_pos_llh_time = hrt_absolute_time();
}

void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	_vel_ned = *(msg_vel_ned_t *)msg;
	_vel_ned_time = hrt_absolute_time();
}

void obs_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{

	_obs = *(msg_obs_t *)msg;
	u8 total = (_obs.header.n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
	u8 count = (_obs.header.n_obs & MSG_OBS_HEADER_SEQ_MASK);
	u8 obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);

	hrt_abstime curr_time = hrt_absolute_time();

	// in case of base
	if (sender_id == 0) {


		if (_obs_info.tow == 0) {
			// init
			memset(&_obs_info, 0, sizeof(_obs_info));
			_obs_info.tow = _obs.header.t.tow;
			_obs_info.total = total;
			_obs_info.obs[count]  = obs_in_msg;

		} else if (_obs_info.tow != _obs.header.t.tow) {
			_n_base_sat = 0;



			// calculate the number of satellite
			for (int i = 0 ; i <  MAX_OBS_PACKET_NUM ; i++) {
				_n_base_sat += _obs_info.obs[i];
			}

			// check period
			_period_for_base = _base_recv_time > 0 ? curr_time - _base_recv_time : 0;
			_base_recv_time = curr_time;

			// check max period
			for (int i = PERIOD_ARRAY_SIZE - 1 ; i > 0  ; i--) {
				_arr_period_for_base[i] = _arr_period_for_base[i - 1];
			}

			_arr_period_for_base[0] = _period_for_base;

			// reset
			memset(&_obs_info, 0, sizeof(_obs_info));
			_obs_info.tow = _obs.header.t.tow;
			_obs_info.total = total;
			_obs_info.obs[count]  = obs_in_msg;

		} else {
			_obs_info.obs[count]  = obs_in_msg;
		}
	}

	// in case of rover
	else {

		if (count == 0) {
			_period_for_rover = _rover_recv_time > 0 ? curr_time - _rover_recv_time : 0;
			_rover_recv_time = curr_time;

			_n_rover_sat = _n_rover_sat_temp;
			_n_rover_sat_temp = 0;
		}

		_n_rover_sat_temp += obs_in_msg;
	}

}

static s32 piksi_port_read(u8 *buff, u32 n, void *context)
{
	s32 result = 0;

	if (_sbp_uart > 0) {
		result = read(_sbp_uart, buff, n);
	}

	return result;
}

 static s32 piksi_port_write(u8 *buff, u32 n, void *context)
 {
 	s32 result = 0;

 	if (_sbp_uart > 0) {
 		result = write(_sbp_uart, buff, n);
 	}

 	return result;
 }

/**
 * Opens the UART device and sets all required serial parameters.
 */
static int open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original)
{
	/* Open UART */
	const int uart = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (uart < 0) {
		err(1, "Error opening port: %s", uart_name);
	}

	/* Back up the original UART configuration to restore it after exit */
	int termios_state;

	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		warnx("ERR: tcgetattr%s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, uart_config);

	/* Disable output post-processing */
	uart_config->c_oflag &= ~OPOST;

	uart_config->c_oflag &= ~ONLCR;

	/* Set baud rate */
	static const speed_t speed = B115200;



	if (cfsetispeed(uart_config, speed) < 0 || cfsetospeed(uart_config, speed) < 0) {
		warnx("ERR: %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, uart_config)) < 0) {
		warnx("ERR: %s (tcsetattr)\n", uart_name);
		close(uart);
		return -1;
	}

	uart_config->c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
	uart_config->c_cflag &= ~CSIZE;
	uart_config->c_cflag |= CS8;			// 8-bit characters
	uart_config->c_cflag &= ~PARENB;			// no parity bit
	uart_config->c_cflag &= ~CSTOPB;			// only need 1 stop bit
	uart_config->c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		// setup for non-canonical mode
	uart_config->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	uart_config->c_oflag &= ~OPOST;

	return uart;
}

/**
 * Print command usage information
 */
static void usage()
{
	fprintf(stderr,
		"usage: piksi start [-d <devicename>]\n"
		"       piksi stop\n"
		"       piksi status\n");
	exit(1);
}


static void status()
{
	hrt_abstime t = hrt_absolute_time();

	fprintf(stderr,
		"ID: %d\n", _piksi_id);
	fprintf(stderr,
		"TOW(%ld) : N[%f m], E[%f m], D[%f m] : IAR(%ld), Sat(%d), OBS(%d), flag(%d)\n",
		_piksi_rtk.tow,
		(double)_piksi_rtk.n,
		(double)_piksi_rtk.e,
		(double)_piksi_rtk.d,
		_piksi_rtk.iar,
		_piksi_rtk.nsats,
		_obs.header.n_obs,
        _baseline_ned.flags);
	fprintf(stderr, "period(rover): %llu ms \n", (t - _rover_recv_time) / 1000);
	fprintf(stderr, "period(base) : %llu ms \n", (t - _base_recv_time) / 1000);
	fprintf(stderr, "nobs_base(%d) / nobs_rover(%d) \n", _n_base_sat, _piksi_rtk.nobs_rover);
	fprintf(stderr,	"RECEVE OBS: %ld\n", _test_count);
	fprintf(stderr,	"RECEVE OBS ERROR COUNT: %ld\n", _error_count);
	fprintf(stderr, "obs tow: %ld, total: %d, obs: ", _obs_info.tow, _obs_info.total);
	for (int i = 0 ; i < MAX_OBS_PACKET_NUM ; i++ ) {
		fprintf(stderr, "%d ", _obs_info.obs[i]);
	}
	fprintf(stderr, "\n");

}


int publish_vehicle_gps_pos()
{

	_vehicle_gps_pos.timestamp = hrt_absolute_time();
	//_vehicle_gps_pos.timestamp = _pos_llh_time;
	_vehicle_gps_pos.lat = _pos_llh.lat * 1e7;
	_vehicle_gps_pos.lon = _pos_llh.lon * 1e7;
	_vehicle_gps_pos.alt = _pos_llh.height * 1e3;
	_vehicle_gps_pos.alt_ellipsoid = 0;                 // Not used
	_vehicle_gps_pos.s_variance_m_s = 0;                // Not used
	_vehicle_gps_pos.c_variance_rad = 0;                // Not used
	_vehicle_gps_pos.fix_type = _fix_type;
	_vehicle_gps_pos.eph = (float)_pos_llh.h_accuracy * 1e-3f;
	_vehicle_gps_pos.epv = (float)_pos_llh.v_accuracy * 1e-3f;
	_vehicle_gps_pos.hdop = (float)_dops.hdop/100;
	_vehicle_gps_pos.vdop = (float)_dops.vdop/100;
	_vehicle_gps_pos.noise_per_ms = 0;                  // Not supported
	_vehicle_gps_pos.jamming_indicator = 0;             // Not used
	_vehicle_gps_pos.vel_n_m_s = _vel_ned.n * 1e-3f;
	_vehicle_gps_pos.vel_e_m_s = _vel_ned.e * 1e-3f;
	_vehicle_gps_pos.vel_d_m_s = _vel_ned.d * 1e-3f;
	_vehicle_gps_pos.vel_m_s = sqrtf(_vel_ned.n*_vel_ned.n + _vel_ned.e*_vel_ned.e) * 1e-3f;
	_vehicle_gps_pos.cog_rad = 0.0f;                    // Not used
	_vehicle_gps_pos.vel_ned_valid = _vel_ned.flags == 0 ? false : true;
	time_t epoch = mktime(&_timeinfo);
	_vehicle_gps_pos.time_utc_usec = (uint64_t)epoch * 1000000ULL;
	_vehicle_gps_pos.satellites_used = _pos_llh.n_sats;


	if (_pos_llh_time > 0 && _pos_llh.lat > 0.0 && _pos_llh.lon > 0.0) {
		int _gps_orb_instance = 0;
		orb_publish_auto(ORB_ID(sensor_gps)
				 , &_vehicle_gps_pos_pub
				 , &_vehicle_gps_pos
				 , &_gps_orb_instance
				 );
	}

	return 0;
}

/**
 * The daemon thread.
 */
static int piksi_thread_main(int argc, char *argv[])
{
	int status = 0;

	/* SBP parser state must be initialized before sbp_process is called. */
	sbp_state_init(&_sbp_state);

	/* register callback function */
	sbp_register_callback(&_sbp_state, SBP_MSG_OBS, &obs_callback, NULL, &_obs_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_OBS_DEP_A, &obs_dep_a_callback, NULL, &_obs_dep_a_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL, &_heartbeat_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_IAR_STATE, &iar_state_callback, NULL, &_iar_state_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_BASELINE_NED, &baseline_ned_callback, NULL, &_baseline_ned_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_GPS_TIME, &gps_time_callback, NULL, &_gps_time_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, NULL, &_pos_llh_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_VEL_NED, &vel_ned_callback, NULL, &_vel_ned_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_UTC_TIME, &utc_callback, NULL, &_utc_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_AGE_CORRECTIONS, &age_corr_callback, NULL, &_age_corr_callback_node);
	sbp_register_callback(&_sbp_state, SBP_MSG_DOPS, &dops_callback, NULL, &_dops_callback_node);

	/* advertise */
	memset(&_piksi_rtk, 0, sizeof(_piksi_rtk));
	_piksi_rtk_pub = orb_advertise(ORB_ID(piksi_rtk), &_piksi_rtk);

	memset(&_vehicle_gps_pos, 0, sizeof(_vehicle_gps_pos));
	memset(&_timeinfo, 0, sizeof(_timeinfo));
	memset(&_obs_info, 0, sizeof(_obs_info));

	/* subscribe */
	struct piksi_data_s	piksi_data;
	memset(&piksi_data, 0, sizeof(piksi_data));
	int piksi_data_sub = orb_subscribe(ORB_ID(piksi_data));

	/* Default values for arguments */
	char *device_name = "/dev/ttyS4"; /* USART8 */

	/* Work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	int ch;

	while ((ch = getopt(argc, argv, "d:")) != EOF) {
		switch (ch) {
		case 'd':
			device_name = optarg;
			break;

		default:
			usage();
			break;
		}
	}

	/* Open UART assuming SmartPort telemetry */
	warnx("opening uart");
	struct termios uart_config_original;
	struct termios uart_config;

	_sbp_uart = open_uart(device_name, &uart_config, &uart_config_original);

	if (_sbp_uart < 0) {
		warnx("could not open %s", device_name);
		err(1, "could not open %s", device_name);
	}

	/* poll descriptor */
	struct pollfd fds[2];
	fds[0].fd = _sbp_uart;
	fds[0].events = POLLIN;
	fds[1].fd = piksi_data_sub;
	fds[1].events = POLLIN;

	thread_running = true;

	/* Main thread loop */
	while (thread_running) {

		status = poll(fds, sizeof(fds) / sizeof(fds[0]), 3000);

		if (status == 0) {		// TIMEOUT

		}
		else if (status > 0) { //if in
			//_test_count++;
			if (fds[1].revents & POLLIN) {


				//_test_count++;


				/* update obs data */
				orb_copy(ORB_ID(piksi_data), piksi_data_sub, &piksi_data);

				char ret = sbp_send_message(&_sbp_state,
							    piksi_data.msg_type,
						    	piksi_data.sender_id,
				 			    piksi_data.len,
				 			    piksi_data.data,
				 			    &piksi_port_write);
				if (ret != 0) { _error_count++;}

			}
			else if (fds[0].revents & POLLIN) {

				_test_count++;
				s8 status_sbp = sbp_process(&_sbp_state, &piksi_port_read);


				if (status_sbp == SBP_OK_CALLBACK_EXECUTED) {
					hrt_abstime t = hrt_absolute_time();

					_piksi_rtk.timestamp = t;
					_piksi_rtk.utc_usec  = _utc_usec;
					_piksi_rtk.tow    = _gps_time.tow;
					_piksi_rtk.n	  = (float)_baseline_ned.n / 1000.0f;
					_piksi_rtk.e	  = (float)_baseline_ned.e / 1000.0f;
					_piksi_rtk.d	  = (float)_baseline_ned.d / 1000.0f;
					_piksi_rtk.vn	  = (float)_vel_ned.n / 1000.0f;
					_piksi_rtk.ve	  = (float)_vel_ned.e / 1000.0f;
					_piksi_rtk.vd	  = (float)_vel_ned.d / 1000.0f;
					_piksi_rtk.nsats  =        _baseline_ned.n_sats;
					_piksi_rtk.iar    =        _iar_state.num_hyps;
					_piksi_rtk.age_corr =	   _age_corr.age;
					//_piksi_rtk.flag	  =	   _baseline_ned.flags;

					_piksi_rtk.flag	  =        _pos_llh.flags;


					if (t - _base_recv_time < MAX_RECV_TIMEOUT) {
						_piksi_rtk.nobs_base    =  _n_base_sat;

					} else {
						_piksi_rtk.nobs_base    =  0;
					}

					_piksi_rtk.period_base  =  _period_for_base / 1000;		// ms

					if (t - _rover_recv_time < MAX_RECV_TIMEOUT) {
						_piksi_rtk.nobs_rover   =  _n_rover_sat;

					} else {
						_piksi_rtk.nobs_rover   =  0;
					}

					_piksi_rtk.period_rover =  _period_for_rover / 1000;		// ms


					uint32_t max_period_from_base = 0;

					for (int i = 0 ; i < PERIOD_ARRAY_SIZE ; i++) {
						if (max_period_from_base < _arr_period_for_base[i]) {
							max_period_from_base = _arr_period_for_base[i];
						}
					}

					_piksi_rtk.max_time_base = max_period_from_base / 1000;	// ms

                    switch(_baseline_ned.flags) {
                    case 0: // invalid
                        _fix_type = 3;
                        break;
                    case 1: // reserved
                        _fix_type = 3;
                        break;
                    case 2: // DGNSS
                        _fix_type = 3;
                        break;
                    case 3: // Float RTK
                        _fix_type = 5;
                        break;
                    case 4: // Fixed RTK
                        _fix_type = 6;
                        break;
                    default:
                        break;
                    };

					// publish orb message
					orb_publish(ORB_ID(piksi_rtk), _piksi_rtk_pub, &_piksi_rtk);


					// publish vehicle gps position message
					publish_vehicle_gps_pos();

				}
			}

		} else {

			// poll error
		}

		if (thread_should_exit == true ) {
			PX4_ERR("STOP.....");
			thread_running = false;
		}
	}

	/* Reset the UART flags to original state */
	tcsetattr(_sbp_uart, TCSANOW, &uart_config_original);
	close(_sbp_uart);

	thread_running = false;
	return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int piksi_rtk_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("missing command");
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		/* this is not an error */
		if (thread_running) {
			errx(0, "piksi already running");
		}

		thread_should_exit = false;
		piksi_task = px4_task_spawn_cmd("piksi",
						SCHED_DEFAULT,
						200,
						2200,
						piksi_thread_main,
						(char *const *)argv);

		while (!thread_running) {
			usleep(200);
		}

		exit(0);

	} else if (!strcmp(argv[1], "stop")) {

		/* this is not an error */
		if (!thread_running) {
			errx(0, "piksi already stopped");
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(200000);
			warnx(".");
		}

		warnx("terminated.");
		exit(0);

	} else if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			status();

		} else {
			errx(1, "not running");
		}

	} else {
		errx(0, "unrecognize parameter");
		usage();
	}

	/* not getting here */
	return 0;
}
