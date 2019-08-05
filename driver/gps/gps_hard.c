/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : notify.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
	* BEEP TIM3 CHANNEL1 PWM Gerente
	* usart is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "usart.h"
#include "f_shell.h"
#include "fs_config.h"
#include "string.h"
#include "gd32f30x.h"
#include "gps.h"
#include "math.h"
#include "f_ops.h"
/*
*****************************************************************************
* Filename:			
* Description:		
* Change History:
*			xxxx	- 20xx-xx-xx - Ver 0.1
*					- created
*			xxxx		- 20xx-xx-xx - Ver
*					- change code
******************************************************************************
*/
extern struct file * usart0_p;
//=======================================================================================================================
/* statis delay for gps */
void gps_delay_ms(unsigned long long msCnt)
{
	unsigned long long i,j,ms;
	/*--------------------*/
	for( ms = 0 ; ms < msCnt ; ms++ )
	{
		for( i = 0 ; i < 250 ; i++ )
		{
			for( j = 0 ; j < 280 ; j++ );
		}
	}
}

//extern volatile unsigned int gps_pps_time; 
//extern volatile unsigned char pps_flags;
//extern unsigned int gps_time_ms_s;
//extern unsigned char ppsnum ;
volatile unsigned int dt = 0;

//========================================================================
extern void gps_delay_ms(unsigned long long msCnt);

/* Define --------------------------------------------------------------------*/
#define UBX_CONFIG_TIMEOUT	200				// ms, timeout for waiting ACK
#define UBX_PACKET_TIMEOUT	2				// ms, if now data during this delay assume that full update received
#define UBX_WAIT_BEFORE_READ	20			// ms, wait before reading to save read() calls
#define DISABLE_MSG_INTERVAL	1000000		// us, try to disable message with this interval
#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define SWAP16(X)	((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))
#define FNV1_32_INIT	((unsigned int)0x811c9dc5)	// init value for FNV1 hash algorithm
#define FNV1_32_PRIME	((unsigned int)0x01000193)	// magic prime for FNV1 hash algorithm

#define TRUE 1
#define FALSE 0

/* Variables -----------------------------------------------------------------*/
struct vehicle_gps_position_s _gps_position;
struct satellite_info_s _sat_info;
struct satellite_info_s *_satellite_info = &_sat_info;
struct tm timeinfo;
unsigned char _configured;
unsigned char _enable_sat_info;
unsigned char _got_posllh;
unsigned char _got_velned;
unsigned char _gps_data_ready;
unsigned char _rate_count_lat_lon;
unsigned char _rate_count_vel;
unsigned char _rx_ck_a;
unsigned char _rx_ck_b;
unsigned char _timeplus_flag;
unsigned char _use_nav_pvt;
short _ack_waiting_msg;
short _rx_msg;
short _rx_payload_length;
short _rx_payload_index;
unsigned int _ubx_version;
static volatile unsigned short _last_ndtr = 512;
unsigned int _timeplus_flag_time;
unsigned int _disable_cmd_last;

float _rate_lat_lon = 0.0f;
float _rate_vel = 0.0f;
ubx_buf_t _receive_buf;
ubx_ack_state_t _ack_state;
ubx_decode_state_t _decode_state;
ubx_rxmsg_state_t _rx_state;
ubx_ack_state_t _ack_state;

gps_signal_quality_def gps_signal_quality_curr;
gps_signal_quality_def gps_signal_quality;
unsigned char Sat_used_index = 0;
unsigned int hrt_absolute_time()
{
	return 0;
}

/*******************************************************************************
//????:	gps_decode_init
//????:	??????
//????:	?
//????:	?
*******************************************************************************/
void gps_decode_init(void)
{
	_decode_state = UBX_DECODE_SYNC1;
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_payload_length = 0;
	_rx_payload_index = 0;
	if(_rx_msg == UBX_MSG_MON_HW)
	{
		memset(&gps_signal_quality_curr.hw_status, 0, sizeof(gps_signal_quality_curr.hw_status));
	}
	if(_rx_msg == UBX_MSG_NAV_SVINFO)
	{
		Sat_used_index = 0;
		memset(&gps_signal_quality_curr.sv_info, 0, sizeof(gps_signal_quality_curr.sv_info));	
	}
}

/*******************************************************************************
//????:	gps_add_byte_to_checksum
//????:	?????
//????:	???
//????:	?
*******************************************************************************/
void gps_add_byte_to_checksum(const unsigned char b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

/*******************************************************************************
//????:	gps_calc_checksum
//????:	?????
//????:	buffer		??
			length		????
			checksum	???
//????:	?
*******************************************************************************/
void gps_calc_checksum(const unsigned char *buffer, const unsigned short length, ubx_checksum_t *checksum)
{
	int i;
	for (i = 0; i < length; i++) {
		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}
}

//static void Delay_ms(unsigned long long msCnt)
//{
//	unsigned long long i,j,ms;
//	for(ms=0;ms<msCnt;ms++)
//	{
//		for(i=0;i<250;i++)
//		{
//			for(j=0;j<280;j++);
//		}
//	}
//}
/*******************************************************************************
//????:	gps_uart_send
//????:	gps????
//????:	buffer	????
			size	????
//????:	?
*******************************************************************************/
/* some common functions */
void gps_set_baudrate(unsigned int baudrate, unsigned int dma_deepth)
{
	fs_ioctl(usart0_p,1,baudrate,0);
}
/* gps send data */
void gps_uart_send(unsigned char *buffer, unsigned char size)
{
	fs_write(usart0_p,buffer,size);
}

/*******************************************************************************
//????:	gps_send_message
//????:	?????
//????:	msg		class&ID
			payload	????
			length	????
//????:	?
*******************************************************************************/
void gps_send_message(const unsigned short msg, const unsigned char *payload, const unsigned short length)
{
	ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2};
	ubx_checksum_t checksum = {0, 0};

	// Populate header
	header.msg	= msg;
	header.length	= length;

	// Calculate checksum
	gps_calc_checksum(((unsigned char*)&header) + 2, sizeof(header) - 2, &checksum);  // skip 2 sync bytes
	if (payload != 0)
		gps_calc_checksum(payload, length, &checksum);

	// Send message
	gps_uart_send((unsigned char*)&header, sizeof(header));
	if (payload != 0)
		gps_uart_send((unsigned char*)payload, length);
	gps_uart_send((unsigned char*)&checksum, sizeof(checksum));
}

/*******************************************************************************
//????:	gps_configure_message_rate
//????:	??gps????
//????:	msg		class&ID
			payload	????
			length	????
//????:	?
*******************************************************************************/
void gps_configure_message_rate(const unsigned short msg, const unsigned char rate)
{
	ubx_payload_tx_cfg_msg_t cfg_msg;	// don't use _buf (allow interleaved operation)

	cfg_msg.msg_union.msg	= msg;
	cfg_msg.rate	= rate;

	gps_send_message(UBX_MSG_CFG_MSG, (unsigned char *)&cfg_msg, sizeof(cfg_msg));
}

/*******************************************************************************
//????:	gps_payload_rx_init
//????:	??????
//????:	?
//????:	-1 = abort, 0 = continue
*******************************************************************************/
int gps_payload_rx_init(void)
{
	int ret = 0;
	unsigned int t;
	_rx_state = UBX_RXMSG_HANDLE;	// handle by default

	switch (_rx_msg) {
	case UBX_MSG_NAV_PVT:
		if (_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8)	/* u-blox 8+ msg format */
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (!_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if not using NAV-PVT
		break;

	case UBX_MSG_NAV_POSLLH:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_posllh_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_NAV_SOL:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_sol_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_NAV_TIMEUTC:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_timeutc_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_NAV_SVINFO:
		if (_satellite_info == 0)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if sat info not requested
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else
			memset(_satellite_info, 0, sizeof(*_satellite_info));	// initialize sat info
		break;

	case UBX_MSG_NAV_VELNED:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_velned_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		else if (_use_nav_pvt)
			_rx_state = UBX_RXMSG_DISABLE;	// disable if using NAV-PVT instead
		break;

	case UBX_MSG_MON_VER:
		break;		// unconditionally handle this message

	case UBX_MSG_MON_HW:
		if (   (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx6_t))	/* u-blox 6 msg format */
		    && (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx7_t)))	/* u-blox 7+ msg format */
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (!_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if not _configured
		break;

	case UBX_MSG_ACK_ACK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if _configured
		break;

	case UBX_MSG_ACK_NAK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_nak_t))
			_rx_state = UBX_RXMSG_ERROR_LENGTH;
		else if (_configured)
			_rx_state = UBX_RXMSG_IGNORE;	// ignore if _configured
		break;

	default:
		_rx_state = UBX_RXMSG_DISABLE;	// disable all other messages
		break;
	}

	switch (_rx_state) {
	case UBX_RXMSG_HANDLE:	// handle message
	case UBX_RXMSG_IGNORE:	// ignore message but don't report error
		ret = 0;
		break;

	case UBX_RXMSG_DISABLE:	// disable unexpected messages
		//printf("ubx msg 0x%04x len %u unexpected", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);

		{
			t = hrt_absolute_time();

			if (t > _disable_cmd_last + DISABLE_MSG_INTERVAL) {
				/* don't attempt for every message to disable, some might not be disabled */
				_disable_cmd_last = t;
				//printf("ubx disabling msg 0x%04x", SWAP16((unsigned)_rx_msg));
				gps_configure_message_rate(_rx_msg, 0);
			}
		}

		ret = -1;	// return error, abort handling this message
		break;

	case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
		//printf("ubx msg 0x%04x invalid len %u", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);
		ret = -1;	// return error, abort handling this message
		break;

	default:	// invalid message state
		//printf("ubx internal err1");
		ret = -1;	// return error, abort handling this message
		break;
	}

	return ret;
}

/*******************************************************************************
//????:	gps_payload_rx_add_nav_svinfo
//????:	Add NAV-SVINFO payload rx byte
//????:	?
//????:	// -1 = error, 0 = ok, 1 = payload completed
*******************************************************************************/
int gps_payload_rx_add_nav_svinfo(const unsigned char b)
{
	int ret = 0;

	if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
		// Fill Part 1 buffer
		_receive_buf.raw[_rx_payload_index] = b;
	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
//			_satellite_info->count = MIN(_receive_buf.payload_rx_nav_svinfo_part1.numCh, SAT_INFO_MAX_SATELLITES);
			_satellite_info->count = (_receive_buf.payload_rx_nav_svinfo_part1.numCh);
			static unsigned char max_numCh = 0;
			if(max_numCh < _receive_buf.payload_rx_nav_svinfo_part1.numCh)
			{
				max_numCh = _receive_buf.payload_rx_nav_svinfo_part1.numCh;
			}
		}
		if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t) + _satellite_info->count * sizeof(ubx_payload_rx_nav_svinfo_part2_t)) {
			// Still room in _satellite_info: fill Part 2 buffer
			unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) % sizeof(ubx_payload_rx_nav_svinfo_part2_t);
			_receive_buf.raw[buf_index] = b;
			if (buf_index == sizeof(ubx_payload_rx_nav_svinfo_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) / sizeof(ubx_payload_rx_nav_svinfo_part2_t);
				sat_index %= SAT_INFO_MAX_SATELLITES;
				_satellite_info->used[sat_index]	= (unsigned char)(_receive_buf.payload_rx_nav_svinfo_part2.flags & 0x01);
				_satellite_info->snr[sat_index]		= (unsigned char)(_receive_buf.payload_rx_nav_svinfo_part2.cno);
				_satellite_info->elevation[sat_index]	= (unsigned char)(_receive_buf.payload_rx_nav_svinfo_part2.elev);
				_satellite_info->azimuth[sat_index]	= (unsigned char)((float)_receive_buf.payload_rx_nav_svinfo_part2.azim * 255.0f / 360.0f);
				_satellite_info->svid[sat_index]	= (unsigned char)(_receive_buf.payload_rx_nav_svinfo_part2.svid);

//				if(_satellite_info->used[sat_index])
				{
					gps_signal_quality_curr.sv_info.snr[Sat_used_index] = _satellite_info->snr[sat_index];
					gps_signal_quality_curr.sv_info.svid[Sat_used_index] = _satellite_info->svid[sat_index];
					gps_signal_quality_curr.sv_info.used[Sat_used_index] = _satellite_info->used[sat_index];
					Sat_used_index++;
					Sat_used_index %= SAT_INFO_MAX_SATELLITES;
				}
			}
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/*******************************************************************************
//????:	gps_fnv1_32_str
//????:
//????:
//????:	?
*******************************************************************************/
unsigned int gps_fnv1_32_str(unsigned char *str, unsigned int hval)
{
    unsigned char *s = str;

    /*
     * FNV-1 hash each octet in the buffer
     */
    while (*s) {

	/* multiply by the 32 bit FNV magic prime mod 2^32 */
#if defined(NO_FNV_GCC_OPTIMIZATION)
	hval *= FNV1_32_PRIME;
#else
	hval += (hval<<1) + (hval<<4) + (hval<<7) + (hval<<8) + (hval<<24);
#endif

	/* xor the bottom with the current octet */
	hval ^= (unsigned int)*s++;
    }

    /* return our new hash value */
    return hval;
}

/*******************************************************************************
//????:	gps_payload_rx_add_mon_ver
//????:	Add MON-VER payload rx byte
//????:	?
//????:	-1 = error, 0 = ok, 1 = payload completed
*******************************************************************************/
int gps_payload_rx_add_mon_ver(const unsigned char b)
{
	int ret = 0;
	unsigned buf_index;
	if (_rx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t)) {
		// Fill Part 1 buffer
		_receive_buf.raw[_rx_payload_index] = b;
	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t)) {
			// Part 1 complete: decode Part 1 buffer and calculate hash for SW&HW version strings
			_ubx_version = gps_fnv1_32_str(_receive_buf.payload_rx_mon_ver_part1.swVersion, FNV1_32_INIT);
			_ubx_version = gps_fnv1_32_str(_receive_buf.payload_rx_mon_ver_part1.hwVersion, _ubx_version);
			//printf("VER hash 0x%08x", _ubx_version);
			//printf("VER hw  \"%10s\"", _receive_buf.payload_rx_mon_ver_part1.hwVersion);
			//printf("VER sw  \"%30s\"", _receive_buf.payload_rx_mon_ver_part1.swVersion);
		}
		// fill Part 2 buffer
		buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(ubx_payload_rx_mon_ver_part2_t);
		_receive_buf.raw[buf_index] = b;
		if (buf_index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1) {
			// Part 2 complete: decode Part 2 buffer
			//printf("VER ext \" %30s\"", _receive_buf.payload_rx_mon_ver_part2.extension);
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/*******************************************************************************
//????:	gps_payload_rx_add
//????:	Add payload rx byte
//????:	?
//????:	// -1 = error, 0 = ok, 1 = payload completed
*******************************************************************************/
int gps_payload_rx_add(const unsigned char b)
{
	int ret = 0;

	_receive_buf.raw[_rx_payload_index] = b;

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/*******************************************************************************
//????:	gps_payload_rx_done
//????:	????
//????:	?
//????:	0 = no message handled, 1 = message handled, 2 = sat info message handled
*******************************************************************************/
extern unsigned long long read_sys_time_us(void);

unsigned int time_slot[50] = {0};
unsigned int time_sat_slot[50] = {0};
unsigned int time_hw_slot[50] = {0};

int gps_payload_rx_done(void)
{
	int ret = 0;
	unsigned int time_now = 0;
	static unsigned char cnt = 0;
	static unsigned char cnt_sat = 0;
	static unsigned char cnt_hw = 0;
	static unsigned int time_old = 0, time_slot_max = 0;
	static unsigned int time_sat_old = 0, time_sat_slot_max = 0;
	static unsigned int time_hw_old = 0, time_hw_slot_max = 0;
	// return if no message handled
	if (_rx_state != UBX_RXMSG_HANDLE) {
		return ret;
	}

	// handle message
	switch (_rx_msg) {

	case UBX_MSG_NAV_PVT:
		//Check if position fix flag is good
		if ((_receive_buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1)
		{
			_gps_position.fix_type		 = _receive_buf.payload_rx_nav_pvt.fixType;
			_gps_position.vel_ned_valid = TRUE;
		}
		else
		{
			_gps_position.fix_type		 = 0;
			_gps_position.vel_ned_valid = FALSE;
		}

		_gps_position.satellites_used	= _receive_buf.payload_rx_nav_pvt.numSV;

		_gps_position.lat		= _receive_buf.payload_rx_nav_pvt.lat;
		_gps_position.lon		= _receive_buf.payload_rx_nav_pvt.lon;
		_gps_position.alt		= _receive_buf.payload_rx_nav_pvt.hMSL;

		_gps_position.eph		= (float)_receive_buf.payload_rx_nav_pvt.hAcc * 1e-3f;
		_gps_position.epv		= (float)_receive_buf.payload_rx_nav_pvt.vAcc * 1e-3f;
		_gps_position.s_variance_m_s	= (float)_receive_buf.payload_rx_nav_pvt.sAcc * 1e-3f;

		_gps_position.vel_m_s		= (float)_receive_buf.payload_rx_nav_pvt.gSpeed * 1e-3f;

		_gps_position.vel_n_m_s	= (float)_receive_buf.payload_rx_nav_pvt.velN * 1e-3f;
		_gps_position.vel_e_m_s	= (float)_receive_buf.payload_rx_nav_pvt.velE * 1e-3f;
		_gps_position.vel_d_m_s	= (float)_receive_buf.payload_rx_nav_pvt.velD * 1e-3f;

		_gps_position.cog_rad		= (float)_receive_buf.payload_rx_nav_pvt.headMot * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position.c_variance_rad	= (float)_receive_buf.payload_rx_nav_pvt.headAcc * M_DEG_TO_RAD_F * 1e-5f;

		//Check if time and date fix flags are good
		if( (_receive_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDDATE)
		 && (_receive_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDTIME)
		 && (_receive_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_FULLYRESOLVED))
		{
//			/* convert to unix timestamp */
//
//			timeinfo.tm_year	= _receive_buf.payload_rx_nav_pvt.year - 1900;
//			timeinfo.tm_mon		= _receive_buf.payload_rx_nav_pvt.month - 1;
//			timeinfo.tm_mday	= _receive_buf.payload_rx_nav_pvt.day;
//			timeinfo.tm_hour	= _receive_buf.payload_rx_nav_pvt.hour;
//			timeinfo.tm_min		= _receive_buf.payload_rx_nav_pvt.min;
//			timeinfo.tm_sec		= _receive_buf.payload_rx_nav_pvt.sec;
//			epoch = mktime(&timeinfo);
//
//			if (epoch > GPS_EPOCH_SECS) {
//				// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
//				// and control its drift. Since we rely on the HRT for our monotonic
//				// clock, updating it from time to time is safe.
//
//
//				ts.tv_sec = epoch;
//				ts.tv_nsec = _receive_buf.payload_rx_nav_pvt.nano;
//				if (clock_settime(CLOCK_REALTIME, &ts)) {
//					//printf("failed setting clock");
//				}
//
//				_gps_position.time_utc_usec = (uint64_t)(epoch) * 1000000ULL;
//				_gps_position.time_utc_usec += _receive_buf.payload_rx_nav_timeutc.nano / 1000;
//			} else {
//				_gps_position.time_utc_usec = 0;
//			}
		}

		_gps_position.timestamp_time		= hrt_absolute_time();
		_gps_position.timestamp_velocity 	= hrt_absolute_time();
		_gps_position.timestamp_variance 	= hrt_absolute_time();
		_gps_position.timestamp_position	= hrt_absolute_time();
		_gps_position.timestamp_error		= _gps_position.timestamp_position - _gps_position.timestamp_timeplus;

		_rate_count_vel++;
		_rate_count_lat_lon++;

		_got_posllh = TRUE;
		_got_velned = TRUE;
		_gps_data_ready = TRUE;
		ret = 1;

		time_now = read_sys_time_us();


		time_slot[cnt] = time_now - time_old;
		if(time_slot[cnt] > time_slot_max && time_slot[cnt] < 150000)
		{
			time_slot_max = time_slot[cnt];
		}
		++cnt;
		cnt %= 50;
		time_old = time_now;
		break;

	case UBX_MSG_NAV_POSLLH:

		_gps_position.lat	= _receive_buf.payload_rx_nav_posllh.lat;
		_gps_position.lon	= _receive_buf.payload_rx_nav_posllh.lon;
		_gps_position.alt	= _receive_buf.payload_rx_nav_posllh.hMSL;
		_gps_position.eph	= (float)_receive_buf.payload_rx_nav_posllh.hAcc * 1e-3f; // from mm to m
		_gps_position.epv	= (float)_receive_buf.payload_rx_nav_posllh.vAcc * 1e-3f; // from mm to m

		_gps_position.timestamp_position = hrt_absolute_time();

		_rate_count_lat_lon++;
		_got_posllh = TRUE;

		ret = 1;
		break;

	case UBX_MSG_NAV_SOL:

		_gps_position.fix_type		= _receive_buf.payload_rx_nav_sol.gpsFix;
		_gps_position.s_variance_m_s	= (float)_receive_buf.payload_rx_nav_sol.sAcc * 1e-2f;	// from cm to m
		_gps_position.satellites_used	= _receive_buf.payload_rx_nav_sol.numSV;

		_gps_position.timestamp_variance = hrt_absolute_time();

		ret = 1;
		break;

	case UBX_MSG_NAV_TIMEUTC:

		ret = 1;
		break;

	case UBX_MSG_NAV_SVINFO:

		// _satellite_info already populated by payload_rx_add_svinfo(), just add a timestamp
		_satellite_info->timestamp = hrt_absolute_time();
		time_now = read_sys_time_us();


		time_sat_slot[cnt_sat] = time_now - time_sat_old;
		if(time_sat_slot[cnt_sat] > time_sat_slot_max && time_sat_slot[cnt_sat] < 1500000)
		{
			time_sat_slot_max = time_sat_slot[cnt_sat];
		}
		++cnt_sat;
		cnt_sat %= 50;
		time_sat_old = time_now;
		memcpy(&gps_signal_quality.sv_info, &gps_signal_quality_curr.sv_info, sizeof(gps_signal_quality_curr.sv_info));
		memset(&gps_signal_quality_curr.sv_info, 0, sizeof(gps_signal_quality_curr.sv_info));
		gps_signal_quality.flag |= 0x02;//sate info is updata.
		Sat_used_index = 0;
		ret = 2;
		break;

	case UBX_MSG_NAV_VELNED:

		_gps_position.vel_m_s		= (float)_receive_buf.payload_rx_nav_velned.speed * 1e-2f;
		_gps_position.vel_n_m_s	= (float)_receive_buf.payload_rx_nav_velned.velN * 1e-2f; /* NED NORTH velocity */
		_gps_position.vel_e_m_s	= (float)_receive_buf.payload_rx_nav_velned.velE * 1e-2f; /* NED EAST velocity */
		_gps_position.vel_d_m_s	= (float)_receive_buf.payload_rx_nav_velned.velD * 1e-2f; /* NED DOWN velocity */
		_gps_position.cog_rad		= (float)_receive_buf.payload_rx_nav_velned.heading * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position.c_variance_rad	= (float)_receive_buf.payload_rx_nav_velned.cAcc * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position.vel_ned_valid	= TRUE;

		_gps_position.timestamp_velocity = hrt_absolute_time();

		_rate_count_vel++;
		_got_velned = TRUE;

		ret = 1;
		break;

	case UBX_MSG_MON_VER:

		ret = 1;
		break;

	case UBX_MSG_MON_HW:

		switch (_rx_payload_length) {

		case sizeof(ubx_payload_rx_mon_hw_ubx6_t):	/* u-blox 6 msg format */
			_gps_position.noise_per_ms		= _receive_buf.payload_rx_mon_hw_ubx6.noisePerMS;
			_gps_position.jamming_indicator	= _receive_buf.payload_rx_mon_hw_ubx6.jamInd;

			ret = 1;
			break;

		case sizeof(ubx_payload_rx_mon_hw_ubx7_t):	/* u-blox 7+ msg format */
			_gps_position.noise_per_ms		= _receive_buf.payload_rx_mon_hw_ubx7.noisePerMS;
			_gps_position.jamming_indicator	= _receive_buf.payload_rx_mon_hw_ubx7.jamInd;

			gps_signal_quality_curr.hw_status.agcCnt = _receive_buf.payload_rx_mon_hw_ubx7.agcCnt;
			gps_signal_quality_curr.hw_status.noisePerMS = _receive_buf.payload_rx_mon_hw_ubx7.noisePerMS;
			gps_signal_quality_curr.hw_status.jamInd = _receive_buf.payload_rx_mon_hw_ubx7.jamInd;
			gps_signal_quality_curr.hw_status.flags = _receive_buf.payload_rx_mon_hw_ubx7.flags;
			gps_signal_quality_curr.flag |= 0x01;//hardware status is updata.
			memcpy(&gps_signal_quality.hw_status, &gps_signal_quality_curr.hw_status, sizeof(gps_signal_quality_curr.hw_status));
			memset(&gps_signal_quality_curr.hw_status, 0, sizeof(gps_signal_quality_curr.hw_status));
			gps_signal_quality.flag |= 0x01;//hardware status is updata.
			time_now = read_sys_time_us();


		time_hw_slot[cnt_hw] = time_now - time_hw_old;
		if(time_hw_slot[cnt_hw] > time_hw_slot_max && time_hw_slot[cnt_hw] < 1500000)
		{
			time_hw_slot_max = time_hw_slot[cnt_hw];
		}
		++cnt_hw;
		cnt_hw %= 50;
		time_hw_old = time_now;
			ret = 1;
			break;

		default:		// unexpected payload size:
			ret = 0;	// don't handle message
			break;
		}
		break;

	case UBX_MSG_ACK_ACK:

		if ((_ack_state == UBX_ACK_WAITING) && (_receive_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
			_ack_state = UBX_ACK_GOT_ACK;
		}

		ret = 1;
		break;

	case UBX_MSG_ACK_NAK:

		if ((_ack_state == UBX_ACK_WAITING) && (_receive_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
			_ack_state = UBX_ACK_GOT_NAK;
		}

		ret = 1;
		break;

	default:
		break;
	}

	return ret;
}

/*******************************************************************************
//????:	gps_parse_char
//????:	????
//????:	b	??
//????:	0 = decoding, 1 = message handled, 2 = sat info message handled
*******************************************************************************/
int gps_parse_char(const unsigned char b)
{
	int ret = 0;
	extern gps_data ublox_gps;

	switch (_decode_state) {

	/* Expecting Sync1 */
	case UBX_DECODE_SYNC1:
		if (b == UBX_SYNC1) {	// Sync1 found --> expecting Sync2
			_decode_state = UBX_DECODE_SYNC2;
		}
		break;

	/* Expecting Sync2 */
	case UBX_DECODE_SYNC2:
		if (b == UBX_SYNC2) {	// Sync2 found --> expecting Class
			_decode_state = UBX_DECODE_CLASS;

		} else {		// Sync1 not followed by Sync2: reset parser
			gps_decode_init();
		}
		break;

	/* Expecting Class */
	case UBX_DECODE_CLASS:
		gps_add_byte_to_checksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
		_rx_msg = b;
		_decode_state = UBX_DECODE_ID;
		break;

	/* Expecting ID */
	case UBX_DECODE_ID:
		gps_add_byte_to_checksum(b);
		_rx_msg |= b << 8;
		_decode_state = UBX_DECODE_LENGTH1;
		break;

	/* Expecting first length byte */
	case UBX_DECODE_LENGTH1:
		gps_add_byte_to_checksum(b);
		_rx_payload_length = b;
		_decode_state = UBX_DECODE_LENGTH2;
		break;

	/* Expecting second length byte */
	case UBX_DECODE_LENGTH2:
		gps_add_byte_to_checksum(b);
		_rx_payload_length |= b << 8;	// calculate payload size
		if (gps_payload_rx_init() != 0) {	// start payload reception
			// payload will not be handled, discard message
			gps_decode_init();
		} else {
			_decode_state = (_rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
		}
		break;

	/* Expecting payload */
	case UBX_DECODE_PAYLOAD:
		gps_add_byte_to_checksum(b);
		switch (_rx_msg) {
		case UBX_MSG_NAV_SVINFO:
			ret = gps_payload_rx_add_nav_svinfo(b);	// add a NAV-SVINFO payload byte
			break;
		case UBX_MSG_MON_VER:
			ret = gps_payload_rx_add_mon_ver(b);	// add a MON-VER payload byte
			break;
		default:
			ret = gps_payload_rx_add(b);		// add a payload byte
			break;
		}
		if (ret < 0) {
			// payload not handled, discard message
			gps_decode_init();
		} else if (ret > 0) {
			// payload complete, expecting checksum
			_decode_state = UBX_DECODE_CHKSUM1;
		} else {
			// expecting more payload, stay in state UBX_DECODE_PAYLOAD
		}
		ret = 0;
		break;

	/* Expecting first checksum byte */
	case UBX_DECODE_CHKSUM1:
		if (_rx_ck_a != b) {
			//printf("ubx checksum1 err");
			gps_decode_init();
		} else {
			_decode_state = UBX_DECODE_CHKSUM2;
		}
		break;

	/* Expecting second checksum byte */
	case UBX_DECODE_CHKSUM2:
		if (_rx_ck_b != b) {
			//printf("ubx checksum2 err");
			gps_decode_init();
		} else {
			ret = gps_payload_rx_done();	// finish payload processing
		}
		gps_decode_init();
		break;

	default:
		break;
	}
	if(gps_transfer(&ublox_gps))
	{ //gps_pps_time; pps_flags
//		if( pps_flags==1  )
//		{
//			float tow;
//			tow=(float)(ublox_gps.iTOW);
//			tow=1000.0f*roundf(tow/1000.0f);
//			if(((unsigned int)tow)>=gps_pps_time)
//			{
//				gps_time_ms_s = gps_time_ms_s + (unsigned int)tow- gps_pps_time;
//			}
//			else
//			{
//				gps_time_ms_s = gps_time_ms_s - (gps_pps_time-(unsigned int)tow);     
//			}
//			pps_flags = 0;    
//		}
//		if(ppsnum<5)
//		{
//			gps_time_ms_s=ublox_gps.iTOW+50;
//		}
//		/* send gps nav to pos */
//		send_gps_nav();
	}
	return ret;
}

void put_to_buf()
{

	
}
/*******************************************************************************
//????:	gps_receive
//????:	gps????
//????:	timeout	??
//????:	-1 = error
 			0 = no message handled
 			1 = message handled
 			2 = sat info message handled
*******************************************************************************/
int gps_receive(void)
{
	unsigned char buf[256] = {0};
	unsigned char count;
	int i;
	int handled = 0;
	
	memset(buf, 0, sizeof(buf));
	
	count = fs_read(usart0_p, (char *)buf, 256);

	/* pass received bytes to the packet decoder */
	for (i = 0; i < count; i++) {
		handled |= gps_parse_char(buf[i]);
	}
	return 1;
}

unsigned int UBX_find_baudrates()
{
	#define BAUD_RATE_TRY_TIMES  	30
	#define BAUD_RATE_TYPE 		 		6
	/* try different baudrates */
	extern void gps_set_baudrate(unsigned int baudrate, unsigned int dma_deepth);
	const unsigned int baudrates[BAUD_RATE_TYPE] = {9600, 115200, 9600, 115200, 9600,115200};
	unsigned int baudrate;
	unsigned int baud_i=0;
	ubx_buf_t _buf;
	static int i = 0;

	baud_i = i / BAUD_RATE_TRY_TIMES;//every baud rate try BAUD_RATE_TRY_TIMES
	if(i++ >= BAUD_RATE_TYPE*BAUD_RATE_TRY_TIMES)
	{
		i = 0;
		return 0;
	}

	baudrate = baudrates[baud_i];
	gps_set_baudrate(baudrate, UBX_UART_MAX_DMA_SIZE);
	/* flush input and wait for at least 20 ms silence */
	gps_decode_init();
	gps_receive();
	gps_decode_init();
	/* Send a CFG-PRT message to set the UBX protocol for in and out
	 * and leave the baudrate as it is, we just want an ACK-ACK for this */
	memset(&_buf.payload_tx_cfg_prt, 0, sizeof(_buf.payload_tx_cfg_prt));
	_buf.payload_tx_cfg_prt.portID		= UBX_TX_CFG_PRT_PORTID;
	_buf.payload_tx_cfg_prt.mode		= UBX_TX_CFG_PRT_MODE;
	_buf.payload_tx_cfg_prt.baudRate	= baudrate;
	_buf.payload_tx_cfg_prt.inProtoMask	= UBX_TX_CFG_PRT_INPROTOMASK;
	_buf.payload_tx_cfg_prt.outProtoMask	= UBX_TX_CFG_PRT_OUTPROTOMASK;

	gps_send_message(UBX_MSG_CFG_PRT, _buf.raw, sizeof(_buf.payload_tx_cfg_prt));
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_PRT;	// memorize sent msg class&ID for ACK check
	return baudrate;
}

int UBX_set_baudrates(unsigned int baudrate)
{
	extern void gps_set_baudrate(unsigned int baudrate, unsigned int dma_deepth);
	ubx_buf_t _buf;

	if(UBX_TX_CFG_PRT_BAUDRATE != baudrate)
	{
		return 1;
	}
	/* Send a CFG-PRT message again, this time change the baudrate */
	memset(&_buf.payload_tx_cfg_prt, 0, sizeof(_buf.payload_tx_cfg_prt));
	_buf.payload_tx_cfg_prt.portID		= UBX_TX_CFG_PRT_PORTID;
	_buf.payload_tx_cfg_prt.mode		= UBX_TX_CFG_PRT_MODE;
	_buf.payload_tx_cfg_prt.baudRate	= UBX_TX_CFG_PRT_BAUDRATE;
	_buf.payload_tx_cfg_prt.inProtoMask	= UBX_TX_CFG_PRT_INPROTOMASK;
	_buf.payload_tx_cfg_prt.outProtoMask	= UBX_TX_CFG_PRT_OUTPROTOMASK;

	gps_send_message(UBX_MSG_CFG_PRT, _buf.raw, sizeof(_buf.payload_tx_cfg_prt));
	gps_send_message(UBX_MSG_CFG_PRT, _buf.raw, sizeof(_buf.payload_tx_cfg_prt));
	/* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
	gps_set_baudrate(UBX_TX_CFG_PRT_BAUDRATE, UBX_UART_MAX_DMA_SIZE);
	/* get buffer */
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_PRT;	// memorize sent msg class&ID for ACK check
	return 0;
	/* at this point we have correct baudrate on both ends */
}

int UBX_define_baudrates()
{
	ubx_buf_t _buf;
	/* Send a CFG-RATE message to define update rate */
	memset(&_buf.payload_tx_cfg_rate, 0, sizeof(_buf.payload_tx_cfg_rate));
	_buf.payload_tx_cfg_rate.measRate	= UBX_TX_CFG_RATE_MEASINTERVAL;
	_buf.payload_tx_cfg_rate.navRate	= UBX_TX_CFG_RATE_NAVRATE;
	_buf.payload_tx_cfg_rate.timeRef	= UBX_TX_CFG_RATE_TIMEREF;

	gps_send_message(UBX_MSG_CFG_RATE, _buf.raw, sizeof(_buf.payload_tx_cfg_rate));
	gps_send_message(UBX_MSG_CFG_RATE, _buf.raw, sizeof(_buf.payload_tx_cfg_rate));
	gps_send_message(UBX_MSG_CFG_RATE, _buf.raw, sizeof(_buf.payload_tx_cfg_rate));
	/*-------------------*/
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_RATE;	// memorize sent msg class&ID for ACK check
	return 0;
}	

int UBX_set_nav5()
{
	ubx_buf_t _buf;
	/* send a NAV5 message to set the options for the internal filter */
	memset(&_buf.payload_tx_cfg_nav5, 0, sizeof(_buf.payload_tx_cfg_nav5));
	_buf.payload_tx_cfg_nav5.mask		= UBX_TX_CFG_NAV5_MASK;
	_buf.payload_tx_cfg_nav5.dynModel	= UBX_TX_CFG_NAV5_DYNMODEL;
	_buf.payload_tx_cfg_nav5.fixMode	= UBX_TX_CFG_NAV5_FIXMODE;

	gps_send_message(UBX_MSG_CFG_NAV5, _buf.raw, sizeof(_buf.payload_tx_cfg_nav5));
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_NAV5;	// memorize sent msg class&ID for ACK check	
	return 0;
}


int UBX_set_sbas()
{
	ubx_buf_t _buf;
	/* send a SBAS message to set the SBAS options */
	memset(&_buf.payload_tx_cfg_sbas, 0, sizeof(_buf.payload_tx_cfg_sbas));
	_buf.payload_tx_cfg_sbas.mode		= UBX_TX_CFG_SBAS_MODE;
	_buf.payload_tx_cfg_sbas.usage		= UBX_TX_CFG_SBAS_USAGE;
	_buf.payload_tx_cfg_sbas.maxSBAS	= UBX_TX_CFG_SBAS_MAXSBAS;
	_buf.payload_tx_cfg_sbas.scanmode2	= UBX_TX_CFG_SBAS_SCANMODE2;
	_buf.payload_tx_cfg_sbas.scanmode1	= UBX_TX_CFG_SBAS_SCANMODE1;

	gps_send_message(UBX_MSG_CFG_SBAS, _buf.raw, sizeof(_buf.payload_tx_cfg_sbas));

	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_SBAS;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_set_nav_pvt_rate()
{//TODO: check the org code "_ack_waiting_msg = UBX_MSG_CFG_MSG" if correc
	gps_configure_message_rate(UBX_MSG_NAV_PVT, 1);
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_MSG;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_set_NAV_TIMEUTC()
{
	gps_configure_message_rate(UBX_MSG_NAV_TIMEUTC, 5);
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_MSG;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_set_NAV_POSLLH()
{
	gps_configure_message_rate(UBX_MSG_NAV_POSLLH, 2);
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_MSG;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_set_NAV_SOL()
{
	gps_configure_message_rate(UBX_MSG_NAV_SOL, 2);
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_MSG;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_set_NAV_VELNED()
{
	gps_configure_message_rate(UBX_MSG_NAV_VELNED, 2);
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_MSG;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_set_CFG_TP5()
{
	ubx_buf_t _buf;
	/* request module version information by sending an empty MON-VER message */
	memset(&_buf.payload_tx_cfg_tp5, 0, sizeof(_buf.payload_tx_cfg_tp5));
	_buf.payload_tx_cfg_tp5.portID	= UBX_TX_CFG_TP5_DATA0;
	_buf.payload_tx_cfg_tp5.version	= UBX_TX_CFG_TP5_DATA1;
	_buf.payload_tx_cfg_tp5.reserved1	= UBX_TX_CFG_TP5_DATA2;
	_buf.payload_tx_cfg_tp5.antCableDelay	= UBX_TX_CFG_TP5_DATA3;
	_buf.payload_tx_cfg_tp5.rfGroupDelay	= UBX_TX_CFG_TP5_DATA4;
	_buf.payload_tx_cfg_tp5.freqPeriod	= UBX_TX_CFG_TP5_DATA5;
	_buf.payload_tx_cfg_tp5.freqPeriodLock	= UBX_TX_CFG_TP5_DATA6;
	_buf.payload_tx_cfg_tp5.pulseLenRatio	= UBX_TX_CFG_TP5_DATA7;
	_buf.payload_tx_cfg_tp5.pulseLenRatioLock	= UBX_TX_CFG_TP5_DATA8;
	_buf.payload_tx_cfg_tp5.userConfigDelay	= UBX_TX_CFG_TP5_DATA9;
	_buf.payload_tx_cfg_tp5.flag	= UBX_TX_CFG_TP5_DATA10;

	gps_send_message(UBX_MSG_CFG_TP5, _buf.raw, sizeof(_buf.payload_tx_cfg_tp5));
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_TP5;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_set_CFG_ITFM()
{
	ubx_buf_t _buf;
	/* request module version information by sending an empty MON-VER message */
	memset(&_buf.payload_tx_cfg_itfm, 0, sizeof(_buf.payload_tx_cfg_itfm));
	_buf.payload_tx_cfg_itfm.config	= UBX_TX_CFG_ITFM_CONFIG;
	_buf.payload_tx_cfg_itfm.config2	= UBX_TX_CFG_ITFM_CONFIG2;

	gps_send_message(UBX_MSG_CFG_ITFM, _buf.raw, sizeof(_buf.payload_tx_cfg_itfm));
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_ITFM;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_HW_status(unsigned char rate)
{
	gps_configure_message_rate(UBX_MSG_MON_HW, rate);
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_MSG;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_set_SVINFO_OUT_RATE(unsigned char rate)
{
	gps_configure_message_rate(UBX_MSG_NAV_SVINFO, rate);
	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = UBX_MSG_CFG_MSG;	// memorize sent msg class&ID for ACK check	
	return 0;
}

int UBX_get_MON_VER()
{
	gps_send_message(UBX_MSG_MON_VER, (void *)0, 0);
	return 0;
}

int UBX_cfg_success()
{
	extern void gps_set_baudrate(unsigned int baudrate, unsigned int dma_deepth);
	unsigned char tmp_buf2[28]={0xB5,0x62,0x06,0x17,0x14,0x00,0x00,0x40,
								0x00,0x02,0x00,0x00,0x00,0x00,0x01,0x00,
								0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
								0x00,0x00,0x75,0x50};
	gps_uart_send(tmp_buf2,sizeof(tmp_buf2));
	/* config success, change the DMA buf to UBX_UART_MAX_DMA_SIZE , don't need now*/
//	gps_set_baudrate(UBX_TX_CFG_PRT_BAUDRATE, UBX_UART_MAX_DMA_SIZE);
	return 0;
}

/*******************************************************************************
//????:	gps_transfer
//????:	gps????
//????:	?
//????:	0 = data no ready,
 			1 = data OK
*******************************************************************************/
unsigned char gps_transfer(gps_data* buf)
{
	unsigned char ret=0;

	unsigned int time = hrt_absolute_time();
	#if CFG_GPS_TEST_SUPPORT
	drv_ops * led_ops;
  static unsigned char flag = 0;
	led_ops = gpio_import_ops();
	#endif
	if(_gps_data_ready)
	{
		buf->iTOW = _receive_buf.payload_rx_nav_pvt.iTOW;
		buf->year = _receive_buf.payload_rx_nav_pvt.year;
		buf->month = _receive_buf.payload_rx_nav_pvt.month;
		buf->day = _receive_buf.payload_rx_nav_pvt.day;
		buf->hour = _receive_buf.payload_rx_nav_pvt.hour;
		buf->min = _receive_buf.payload_rx_nav_pvt.min;
		buf->sec = _receive_buf.payload_rx_nav_pvt.sec;
		buf->valid = _receive_buf.payload_rx_nav_pvt.valid;
		buf->tAcc = _receive_buf.payload_rx_nav_pvt.tAcc;
		buf->nano = _receive_buf.payload_rx_nav_pvt.nano;
		buf->fixType = _receive_buf.payload_rx_nav_pvt.fixType;//
		buf->flags = _receive_buf.payload_rx_nav_pvt.flags;
		buf->reserved1 = _receive_buf.payload_rx_nav_pvt.reserved1;
		buf->numSV = _receive_buf.payload_rx_nav_pvt.numSV;//
		buf->lon = _receive_buf.payload_rx_nav_pvt.lon;//
		buf->lat = _receive_buf.payload_rx_nav_pvt.lat;//
		buf->height = _receive_buf.payload_rx_nav_pvt.height;//
		buf->hMSL = _receive_buf.payload_rx_nav_pvt.hMSL;
		buf->hAcc = _receive_buf.payload_rx_nav_pvt.hAcc;
		buf->vAcc = _receive_buf.payload_rx_nav_pvt.vAcc;
		buf->velN = _receive_buf.payload_rx_nav_pvt.velN;//
		buf->valid = _receive_buf.payload_rx_nav_pvt.valid;
		buf->velE = _receive_buf.payload_rx_nav_pvt.velE;//
		buf->velD = _receive_buf.payload_rx_nav_pvt.velD;//
		buf->gSpeed = _receive_buf.payload_rx_nav_pvt.gSpeed;
		buf->headMot = _receive_buf.payload_rx_nav_pvt.headMot;
		buf->sAcc = _receive_buf.payload_rx_nav_pvt.sAcc;
		buf->headAcc = _receive_buf.payload_rx_nav_pvt.headAcc;
		buf->pDOP = _receive_buf.payload_rx_nav_pvt.pDOP;//
		buf->reserved2 = _receive_buf.payload_rx_nav_pvt.reserved2;
		buf->reserved3 = _receive_buf.payload_rx_nav_pvt.reserved3;
		buf->headVeh = _receive_buf.payload_rx_nav_pvt.headVeh;
		buf->reserved4 = _receive_buf.payload_rx_nav_pvt.reserved4;
		buf->timeplus_timestamp = read_sys_time_us()/1000;//ms
		buf->position_timestamp = read_sys_time_us();
		buf->error_timestamp = _gps_position.timestamp_error;
		_gps_data_ready = FALSE;
//		get_gps_speed(((int16_t)buf->velN/10),((int16_t)buf->velE/10),((int16_t)buf->velD/10));
//		ground_gps(buf);
//		ground_rc();
		ret = 1;
		#if 1
		static unsigned int led_ctrl = 0;
		
		if( led_ctrl++ % 2 )
		{
			gpio_bit_set(GPIOB , GPIO_PIN_14);
		}
		else
		{
			gpio_bit_reset(GPIOB , GPIO_PIN_14);
		}		
		#endif

	}
	return ret;
}

/*	
 * 	
 *	function : config ublox
 *	return   : 0 success, 1 fail
 */
Enum_UBX_STATE 		fail_state = UBX_STATE_CFG_SUCCESS;
int gps_configure()
{
	Enum_UBX_STATE 		ubx_cfg_state = UBX_STATE_GET_BAUD_RATE;
	unsigned int 		baudrate = 0;
	unsigned char 		buf[UBX_UART_MAX_DMA_SIZE] = {0};
	unsigned int 		count = 0, retry = 0, cfg_end_flag = 0;
	
	fail_state = UBX_STATE_CFG_SUCCESS;
	while(1)
	{
		/* send message to config UBlox, the send state machine and
		read state machine is is different task is much better*/
		switch(ubx_cfg_state)
		{
			case UBX_STATE_GET_BAUD_RATE :
				baudrate = UBX_find_baudrates();
				if(baudrate == 0)
				{
					fail_state = ubx_cfg_state;
					ubx_cfg_state = UBX_STATE_CFG_FAIL;
				}
			break;
				
			case UBX_STATE_SET_BAUD_RATE :
				UBX_set_baudrates(UBX_TX_CFG_PRT_BAUDRATE);
				ubx_cfg_state++;
			break;

			case UBX_STATE_DEFINE_BAUD_RATE :
				UBX_define_baudrates();
			break;

			case UBX_STATE_SET_NAV5 :
				UBX_set_nav5();
			break;

			case UBX_STATE_SET_SBAS :
				UBX_set_sbas();
			break;

			case UBX_STATE_SET_NAV_PVT :
				UBX_set_nav_pvt_rate();
			break;

			case UBX_STATE_SET_NAV_TIMEUTC :
				UBX_set_NAV_TIMEUTC();			
			break;

			case UBX_STATE_SET_NAV_POSLLH :
				UBX_set_NAV_POSLLH();
			break;

			case UBX_STATE_SET_NAV_SOL :
				UBX_set_NAV_SOL();
			break;

			case UBX_STATE_SET_NAV_VELNED :
				UBX_set_NAV_VELNED();
			break;

			case UBX_STATE_CFG_TP5 :
				UBX_set_CFG_TP5();
			break;

			case UBX_STATE_MON_VER :
				UBX_get_MON_VER();
				ubx_cfg_state++;
			break;

			case UBX_STATE_CFG_ITFM :
				UBX_set_CFG_ITFM();
			break;

			case UBX_STATE_CFG_HW_STATUS_OUT_RATE :
				UBX_HW_status(1);
			break;

			case UBX_STATE_CFG_SVINFO_OUT_RATE :
				UBX_set_SVINFO_OUT_RATE(1);
			break;

			case UBX_STATE_CFG_SUCCESS :
				UBX_cfg_success();
				_configured = TRUE;
				cfg_end_flag = 1;
			break;

			case UBX_STATE_CFG_FAIL :
			default :
				_configured = FALSE;
				cfg_end_flag = 1;
			break;
		}
		if(cfg_end_flag)
		{
			return (!_configured);
		}
		gps_delay_ms(10);/* wait 20ms after send message */

		memset(buf, 0, sizeof(buf));
		
		count = fs_read(usart0_p, (char *)buf, sizeof(buf));
		
		if (count > 0)
		{
			/* pass received bytes to the packet decoder */
			for (int i = 0; i < count; i++) {
				gps_parse_char(buf[i]);
			}
			switch(ubx_cfg_state)
			{
				case UBX_STATE_GET_BAUD_RATE :
				case UBX_STATE_SET_BAUD_RATE :
				case UBX_STATE_DEFINE_BAUD_RATE :
				case UBX_STATE_SET_NAV5 :
				case UBX_STATE_SET_SBAS :
				case UBX_STATE_SET_NAV_TIMEUTC :
				case UBX_STATE_SET_NAV_POSLLH :
				case UBX_STATE_SET_NAV_SOL :
				case UBX_STATE_SET_NAV_VELNED :
				case UBX_STATE_CFG_TP5 :
				case UBX_STATE_CFG_ITFM :
				case UBX_STATE_CFG_HW_STATUS_OUT_RATE :
				case UBX_STATE_CFG_SVINFO_OUT_RATE :
					if (_ack_state == UBX_ACK_GOT_ACK)
					{
						ubx_cfg_state++;
						retry = 0;
					}
					else
					{
						if(retry++ > 40)
						{
							fail_state = ubx_cfg_state;
							ubx_cfg_state = UBX_STATE_CFG_FAIL;
						}
					}
					_ack_state = UBX_ACK_IDLE;
				break;

				case UBX_STATE_SET_NAV_PVT :
					if(_ack_state == UBX_ACK_GOT_ACK)
					{
						_use_nav_pvt = TRUE;
						ubx_cfg_state = UBX_STATE_CFG_TP5;
						retry = 0;
					}
					else
					{
						if(retry++ > 40)
						{
							fail_state = ubx_cfg_state;
							ubx_cfg_state = UBX_STATE_CFG_FAIL;
						}
					}
				break;

				case UBX_STATE_MON_VER :
				case UBX_STATE_CFG_SUCCESS :
				case UBX_STATE_CFG_FAIL :
				default:
					;
				break;
			}
		}
		else
		{
			switch(ubx_cfg_state)
			{
//				case UBX_STATE_SET_BAUD_RATE :
				case UBX_STATE_DEFINE_BAUD_RATE :
				case UBX_STATE_SET_NAV5 :
				case UBX_STATE_SET_SBAS :
				case UBX_STATE_SET_NAV_TIMEUTC :
				case UBX_STATE_SET_NAV_POSLLH :
				case UBX_STATE_SET_NAV_SOL :
				case UBX_STATE_SET_NAV_VELNED :
				case UBX_STATE_CFG_TP5 :
				case UBX_STATE_CFG_ITFM :
				case UBX_STATE_CFG_HW_STATUS_OUT_RATE :
				case UBX_STATE_CFG_SVINFO_OUT_RATE :
					{
						if(retry++ > 40)
						{
							fail_state = ubx_cfg_state;
							ubx_cfg_state = UBX_STATE_CFG_FAIL;
						}
					}
				break;
			}
		}
	}
}


//=======================================================================================================================

















