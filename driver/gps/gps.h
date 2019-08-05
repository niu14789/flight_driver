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
	* LED is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#ifndef	__GPS_H__
#define	__GPS_H__

/* Define --------------------------------------------------------------------*/

#define UBX_PACKET_HEAD_LEN         6
#define UBX_CKECK_SUM_LEN           2
/* take care off DMA_SIZE, should be <= 60, if use 64 
	will miss 4byte in driver uart.c -> u_s_art_read when DMA
	is copying
*/
//#define UBX_CFG_DMA_SIZE			60
#define UBX_UART_MAX_DMA_SIZE		100
#define UBX_USART1_MAX_DMA_SIZE		(128*4)
#define UBX_MAX_PACKET_SIZE			(UBX_UART_MAX_DMA_SIZE * 3)//max packet already know is 5c = 100 in org data(find 264), max read is 52
#define UBX_RAW_DATA_BUF_SIZE		(UBX_MAX_PACKET_SIZE * 2)
#define UBX_SYNC1 0xB5

#define M_DEG_TO_RAD_F 	0.01745329251994f
#define GPS_EPOCH_SECS 1234567890ULL

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

//offset
#define UBX_SYNC1_OFFSET				0x00
#define UBX_SYNC2_OFFSET				0x01
#define UBX_MSGID1_OFFSET				0x02
#define UBX_MSGID2_OFFSET				0x03
#define UBX_LEN1_OFFSET					0x04
#define UBX_LEN2_OFFSET					0x05
#define UBX_PAYLOAD_OFFSET				0x06

/* Message Classes */
#define UBX_CLASS_NAV		0x01
#define UBX_CLASS_ACK		0x05
#define UBX_CLASS_CFG		0x06
#define UBX_CLASS_MON		0x0A

/* Message IDs */
#define UBX_ID_NAV_POSLLH	0x02
#define UBX_ID_NAV_SOL		0x06
#define UBX_ID_NAV_PVT		0x07
#define UBX_ID_NAV_VELNED	0x12
#define UBX_ID_NAV_TIMEUTC	0x21
#define UBX_ID_NAV_SVINFO	0x30
#define UBX_ID_NAV_STATUS	0x35
#define UBX_ID_ACK_NAK		0x00
#define UBX_ID_ACK_ACK		0x01
#define UBX_ID_CFG_PRT		0x00
#define UBX_ID_CFG_MSG		0x01
#define UBX_ID_CFG_RATE		0x08
#define UBX_ID_CFG_NAV5		0x24
#define UBX_ID_CFG_SBAS		0x16
#define UBX_ID_CFG_TP5		0x31
#define UBX_ID_CFG_ITFM		0x39
#define UBX_ID_MON_VER		0x04
#define UBX_ID_MON_HW		0x09

/* Message Classes & IDs */
#define UBX_MSG_NAV_POSLLH	((UBX_CLASS_NAV) | UBX_ID_NAV_POSLLH << 8)
#define UBX_MSG_NAV_SOL		((UBX_CLASS_NAV) | UBX_ID_NAV_SOL << 8)
#define UBX_MSG_NAV_PVT		((UBX_CLASS_NAV) | UBX_ID_NAV_PVT << 8)
#define UBX_MSG_NAV_VELNED	((UBX_CLASS_NAV) | UBX_ID_NAV_VELNED << 8)
#define UBX_MSG_NAV_TIMEUTC	((UBX_CLASS_NAV) | UBX_ID_NAV_TIMEUTC << 8)
#define UBX_MSG_NAV_SVINFO	((UBX_CLASS_NAV) | UBX_ID_NAV_SVINFO << 8)
#define UBX_MSG_NAV_STATUS	((UBX_CLASS_NAV) | UBX_ID_NAV_STATUS << 8)
#define UBX_MSG_ACK_NAK		((UBX_CLASS_ACK) | UBX_ID_ACK_NAK << 8)
#define UBX_MSG_ACK_ACK		((UBX_CLASS_ACK) | UBX_ID_ACK_ACK << 8)
#define UBX_MSG_CFG_PRT		((UBX_CLASS_CFG) | UBX_ID_CFG_PRT << 8)
#define UBX_MSG_CFG_MSG		((UBX_CLASS_CFG) | UBX_ID_CFG_MSG << 8)
#define UBX_MSG_CFG_RATE	((UBX_CLASS_CFG) | UBX_ID_CFG_RATE << 8)
#define UBX_MSG_CFG_NAV5	((UBX_CLASS_CFG) | UBX_ID_CFG_NAV5 << 8)
#define UBX_MSG_CFG_SBAS	((UBX_CLASS_CFG) | UBX_ID_CFG_SBAS << 8)
#define UBX_MSG_CFG_TP5		((UBX_CLASS_CFG) | UBX_ID_CFG_TP5 << 8)
#define UBX_MSG_CFG_ITFM	((UBX_CLASS_CFG) | UBX_ID_CFG_ITFM << 8)
#define UBX_MSG_MON_HW		((UBX_CLASS_MON) | UBX_ID_MON_HW << 8)
#define UBX_MSG_MON_VER		((UBX_CLASS_MON) | UBX_ID_MON_VER << 8)

/* RX NAV-PVT message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_PVT_VALID_VALIDDATE		0x01	/**< validDate (Valid UTC Date) */
#define UBX_RX_NAV_PVT_VALID_VALIDTIME		0x02	/**< validTime (Valid UTC Time) */
#define UBX_RX_NAV_PVT_VALID_FULLYRESOLVED	0x04	/**< fullyResolved (1 = UTC Time of Day has been fully resolved (no seconds uncertainty)) */

/*   Bitfield "flags" masks */
#define UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK		0x01	/**< gnssFixOK (A valid fix (i.e within DOP & accuracy masks)) */
#define UBX_RX_NAV_PVT_FLAGS_DIFFSOLN		0x02	/**< diffSoln (1 if differential corrections were applied) */
#define UBX_RX_NAV_PVT_FLAGS_PSMSTATE		0x1C	/**< psmState (Power Save Mode state (see Power Management)) */
#define UBX_RX_NAV_PVT_FLAGS_HEADVEHVALID	0x20	/**< headVehValid (Heading of vehicle is valid) */

/* RX NAV-TIMEUTC message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDTOW	0x01	/**< validTOW (1 = Valid Time of Week) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDKWN	0x02	/**< validWKN (1 = Valid Week Number) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC	0x04	/**< validUTC (1 = Valid UTC Time) */
#define UBX_RX_NAV_TIMEUTC_VALID_UTCSTANDARD	0xF0	/**< utcStandard (0..15 = UTC standard identifier) */

/* TX CFG-PRT message contents */
#define UBX_TX_CFG_PRT_PORTID		0x01		/**< UART1 */
#define UBX_TX_CFG_PRT_MODE		0x000008D0	/**< 0b0000100011010000: 8N1 */
#define UBX_TX_CFG_PRT_BAUDRATE		115200		/**< choose 57600 as GPS baudrate */
#define UBX_TX_CFG_PRT_INPROTOMASK	0x01		/**< UBX in */
#define UBX_TX_CFG_PRT_OUTPROTOMASK	0x01		/**< UBX out */

/* TX CFG-RATE message contents */
#define UBX_TX_CFG_RATE_MEASINTERVAL	100		/**< 100ms for 10Hz */
#define UBX_TX_CFG_RATE_NAVRATE		1		/**< cannot be changed */
#define UBX_TX_CFG_RATE_TIMEREF		0		/**< 0: UTC, 1: GPS time */

/* TX CFG-NAV5 message contents */
#define UBX_TX_CFG_NAV5_MASK		0x0005		/**< Only update dynamic model and fix mode */
#define UBX_TX_CFG_NAV5_DYNMODEL	7		/**< 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
#define UBX_TX_CFG_NAV5_FIXMODE		2		/**< 1 2D only, 2 3D only, 3 Auto 2D/3D */

/* TX CFG-SBAS message contents */
#define UBX_TX_CFG_SBAS_MODE_ENABLED	1				/**< SBAS enabled */
#define UBX_TX_CFG_SBAS_MODE_DISABLED	0				/**< SBAS disabled */
#define UBX_TX_CFG_SBAS_MODE		UBX_TX_CFG_SBAS_MODE_DISABLED	/**< SBAS enabled or disabled */
#define UBX_TX_CFG_SBAS_USAGE		0x07
#define UBX_TX_CFG_SBAS_MAXSBAS		0x01
#define UBX_TX_CFG_SBAS_SCANMODE2	0x00
#define UBX_TX_CFG_SBAS_SCANMODE1	0x00000000

/* TX CFG-MSG message contents */
#define UBX_TX_CFG_MSG_RATE1_5HZ	0x01 		/**< {0x00, 0x01, 0x00, 0x00, 0x00, 0x00} the second entry is for UART1 */
#define UBX_TX_CFG_MSG_RATE1_1HZ	0x05		/**< {0x00, 0x05, 0x00, 0x00, 0x00, 0x00} the second entry is for UART1 */
#define UBX_TX_CFG_MSG_RATE1_05HZ	10

/* TX CFG-TP5 message contents */
#define UBX_TX_CFG_TP5_DATA0		0x00
#define UBX_TX_CFG_TP5_DATA1		0x01
#define UBX_TX_CFG_TP5_DATA2		0x0000
#define UBX_TX_CFG_TP5_DATA3		0x0032
#define UBX_TX_CFG_TP5_DATA4		0x0000
#define UBX_TX_CFG_TP5_DATA5		0x0000000A
#define UBX_TX_CFG_TP5_DATA6		0x0000000A
#define UBX_TX_CFG_TP5_DATA7		0x19999999
#define UBX_TX_CFG_TP5_DATA8		0x19999999
#define UBX_TX_CFG_TP5_DATA9		0x00000000
#define UBX_TX_CFG_TP5_DATA10		0x0000086F
/* TX CFG-ITFM message contents */
#define UBX_TX_CFG_ITFM_CONFIG		0xAD62ACF3
#define UBX_TX_CFG_ITFM_CONFIG2		0x0000431E

/*** u-blox protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* General: Header */
typedef struct {
	unsigned char		sync1;
	unsigned char		sync2;
	unsigned short	msg;
	unsigned short	length;
} ubx_header_t;

/* General: Checksum */
typedef struct {
	unsigned char		ck_a;
	unsigned char		ck_b;
} ubx_checksum_t ;

/* Rx NAV-POSLLH */
typedef struct {
	unsigned int	iTOW;		/**< GPS Time of Week [ms] */
	int		lon;		/**< Longitude [1e-7 deg] */
	int		lat;		/**< Latitude [1e-7 deg] */
	int		height;		/**< Height above ellipsoid [mm] */
	int		hMSL;		/**< Height above mean sea level [mm] */
	unsigned int	hAcc;  		/**< Horizontal accuracy estimate [mm] */
	unsigned int	vAcc;  		/**< Vertical accuracy estimate [mm] */
} ubx_payload_rx_nav_posllh_t;

/* Rx NAV-SOL */
typedef struct {
	unsigned int	iTOW;		/**< GPS Time of Week [ms] */
	int		fTOW;		/**< Fractional part of iTOW (range: +/-500000) [ns] */
	short		week;		/**< GPS week */
	unsigned char		gpsFix;		/**< GPSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GPS + dead reckoning, 5 = time only fix */
	unsigned char		flags;
	int		ecefX;
	int		ecefY;
	int		ecefZ;
	unsigned int	pAcc;
	int		ecefVX;
	int		ecefVY;
	int		ecefVZ;
	unsigned int	sAcc;
	unsigned short	pDOP;
	unsigned char		reserved1;
	unsigned char		numSV;		/**< Number of SVs used in Nav Solution */
	unsigned int	reserved2;
} ubx_payload_rx_nav_sol_t;

/* Rx NAV-PVT (ubx8) */
typedef struct {
	unsigned int	iTOW;		/**< GPS Time of Week [ms] */
	unsigned short	year; 		/**< Year (UTC)*/
	unsigned char		month; 		/**< Month, range 1..12 (UTC) */
	unsigned char		day; 		/**< Day of month, range 1..31 (UTC) */
	unsigned char		hour; 		/**< Hour of day, range 0..23 (UTC) */
	unsigned char		min; 		/**< Minute of hour, range 0..59 (UTC) */
	unsigned char		sec;		/**< Seconds of minute, range 0..60 (UTC) */
	unsigned char		valid; 		/**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
	unsigned int	tAcc; 		/**< Time accuracy estimate (UTC) [ns] */
	int		nano;		/**< Fraction of second (UTC) [-1e9...1e9 ns] */
	unsigned char		fixType;	/**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	unsigned char		flags;		/**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
	unsigned char		reserved1;
	unsigned char		numSV;		/**< Number of SVs used in Nav Solution */
	int		lon;		/**< Longitude [1e-7 deg] */
	int		lat;		/**< Latitude [1e-7 deg] */
	int		height;		/**< Height above ellipsoid [mm] */
	int		hMSL;		/**< Height above mean sea level [mm] */
	unsigned int	hAcc;  		/**< Horizontal accuracy estimate [mm] */
	unsigned int	vAcc;  		/**< Vertical accuracy estimate [mm] */
	int		velN;		/**< NED north velocity [mm/s]*/
	int		velE;		/**< NED east velocity [mm/s]*/
	int		velD;		/**< NED down velocity [mm/s]*/
	int		gSpeed;		/**< Ground Speed (2-D) [mm/s] */
	int		headMot;	/**< Heading of motion (2-D) [1e-5 deg] */
	unsigned int	sAcc;		/**< Speed accuracy estimate [mm/s] */
	unsigned int	headAcc;	/**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
	unsigned short	pDOP;		/**< Position DOP [0.01] */
	unsigned short	reserved2;
	unsigned int	reserved3;
	int		headVeh;	/**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
	unsigned int	reserved4;	/**< (ubx8+ only) */
} ubx_payload_rx_nav_pvt_t;

#define UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8	(sizeof(ubx_payload_rx_nav_pvt_t))

/* Rx NAV-TIMEUTC */
typedef struct {
	unsigned int	iTOW;		/**< GPS Time of Week [ms] */
	unsigned int	tAcc; 		/**< Time accuracy estimate (UTC) [ns] */
	int		nano;		/**< Fraction of second, range -1e9 .. 1e9 (UTC) [ns] */
	unsigned short	year; 		/**< Year, range 1999..2099 (UTC) */
	unsigned char		month; 		/**< Month, range 1..12 (UTC) */
	unsigned char		day; 		/**< Day of month, range 1..31 (UTC) */
	unsigned char		hour; 		/**< Hour of day, range 0..23 (UTC) */
	unsigned char		min; 		/**< Minute of hour, range 0..59 (UTC) */
	unsigned char		sec;		/**< Seconds of minute, range 0..60 (UTC) */
	unsigned char		valid; 		/**< Validity Flags (see UBX_RX_NAV_TIMEUTC_VALID_...) */
} ubx_payload_rx_nav_timeutc_t;

/* Rx NAV-SVINFO Part 1 */
typedef struct {
	unsigned int	iTOW;		/**< GPS Time of Week [ms] */
	unsigned char		numCh; 		/**< Number of channels */
	unsigned char		globalFlags;
	unsigned short	reserved2;
} ubx_payload_rx_nav_svinfo_part1_t;

/* Rx NAV-SVINFO Part 2 (repeated) */
typedef struct {
	unsigned char		chn; 		/**< Channel number, 255 for SVs not assigned to a channel */
	unsigned char		svid; 		/**< Satellite ID */
	unsigned char		flags;
	unsigned char		quality;
	unsigned char		cno;		/**< Carrier to Noise Ratio (Signal Strength) [dbHz] */
	char		elev; 		/**< Elevation [deg] */
	short		azim; 		/**< Azimuth [deg] */
	int		prRes; 		/**< Pseudo range residual [cm] */
} ubx_payload_rx_nav_svinfo_part2_t;

/* Rx NAV-VELNED */
typedef struct {
	unsigned int	iTOW;		/**< GPS Time of Week [ms] */
	int		velN;		/**< North velocity component [cm/s]*/
	int		velE;		/**< East velocity component [cm/s]*/
	int		velD;		/**< Down velocity component [cm/s]*/
	unsigned int	speed;		/**< Speed (3-D) [cm/s] */
	unsigned int	gSpeed;		/**< Ground speed (2-D) [cm/s] */
	int		heading;	/**< Heading of motion 2-D [1e-5 deg] */
	unsigned int	sAcc;		/**< Speed accuracy estimate [cm/s] */
	unsigned int	cAcc;		/**< Course / Heading accuracy estimate [1e-5 deg] */
} ubx_payload_rx_nav_velned_t;

typedef struct 
{
	unsigned char 	gnssId;
	unsigned char 	svId;
	unsigned char 	cno;
	char 			elev;
	short 			azim;
	short 			prRes;
	unsigned int 	flags;
} ubx_payload_rx_nav_sta_block_t;

typedef struct 
{
	unsigned int	iTOW;		/**< GPS Time of Week [ms] */
	unsigned char 	version;
	unsigned char 	numSVs;
	unsigned short 	reserved1;
	ubx_payload_rx_nav_sta_block_t 	*nav_sta_block;
} ubx_payload_rx_nav_sat_t;

/* Rx MON-HW (ubx6) */
typedef struct {
	unsigned int	pinSel;
	unsigned int	pinBank;
	unsigned int	pinDir;
	unsigned int	pinVal;
	unsigned short	noisePerMS;
	unsigned short	agcCnt;
	unsigned char		aStatus;
	unsigned char		aPower;
	unsigned char		flags;
	unsigned char		reserved1;
	unsigned int	usedMask;
	unsigned char		VP[25];
	unsigned char		jamInd;
	unsigned short	reserved3;
	unsigned int	pinIrq;
	unsigned int	pullH;
	unsigned int	pullL;
} ubx_payload_rx_mon_hw_ubx6_t;

/* Rx MON-HW (ubx7+) */
typedef struct {
	unsigned int	pinSel;
	unsigned int	pinBank;
	unsigned int	pinDir;
	unsigned int	pinVal;
	unsigned short	noisePerMS;
	unsigned short	agcCnt;
	unsigned char		aStatus;
	unsigned char		aPower;
	unsigned char		flags;
	unsigned char		reserved1;
	unsigned int	usedMask;
	unsigned char		VP[17];
	unsigned char		jamInd;
	unsigned short	reserved3;
	unsigned int	pinIrq;
	unsigned int	pullH;
	unsigned int	pullL;
} ubx_payload_rx_mon_hw_ubx7_t;

/* Rx MON-VER Part 1 */
typedef struct {
	unsigned char		swVersion[30];
	unsigned char		hwVersion[10];
} ubx_payload_rx_mon_ver_part1_t;

/* Rx MON-VER Part 2 (repeated) */
typedef struct {
	unsigned char		extension[30];
} ubx_payload_rx_mon_ver_part2_t;

/* Rx ACK-ACK */
typedef	union {
	unsigned short	msg;
	struct {
		unsigned char	clsID;
		unsigned char	msgID;
	}id;
} ubx_payload_rx_ack_ack_t;

/* Rx ACK-NAK */
typedef	union {
	unsigned short	msg;
	struct {
		unsigned char	clsID;
		unsigned char	msgID;
	}id;
} ubx_payload_rx_ack_nak_t;

/* Tx CFG-PRT */
typedef struct {
	unsigned char		portID;
	unsigned char		reserved0;
	unsigned short	txReady;
	unsigned int	mode;
	unsigned int	baudRate;
	unsigned short	inProtoMask;
	unsigned short	outProtoMask;
	unsigned short	flags;
	unsigned short	reserved5;
} ubx_payload_tx_cfg_prt_t;
/* Tx CFG-TP5 */
typedef struct {
	unsigned char		portID;
	unsigned char		version;
	unsigned short	reserved1;
	short		antCableDelay;
	short		rfGroupDelay;
	unsigned int	freqPeriod;
	unsigned int	freqPeriodLock;
	unsigned int	pulseLenRatio;
	unsigned int	pulseLenRatioLock;
	int		userConfigDelay;
	unsigned int	flag;
} ubx_payload_tx_cfg_tp5_t;
/* Tx CFG-ITFM */
typedef struct {
	unsigned int 	config;
	unsigned int 	config2;
} ubx_payload_tx_cfg_itfm_t;
/* Tx CFG-RATE */
typedef struct {
	unsigned short	measRate;	/**< Measurement Rate, GPS measurements are taken every measRate milliseconds */
	unsigned short	navRate;	/**< Navigation Rate, in number of measurement cycles. This parameter cannot be changed, and must be set to 1 */
	unsigned short	timeRef;	/**< Alignment to reference time: 0 = UTC time, 1 = GPS time */
} ubx_payload_tx_cfg_rate_t;

/* Tx CFG-NAV5 */
typedef struct {
	unsigned short	mask;
	unsigned char		dynModel;	/**< Dynamic Platform model: 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
	unsigned char		fixMode;	/**< Position Fixing Mode: 1 2D only, 2 3D only, 3 Auto 2D/3D */
	int		fixedAlt;
	unsigned int	fixedAltVar;
	char		minElev;
	unsigned char		drLimit;
	unsigned short	pDop;
	unsigned short	tDop;
	unsigned short	pAcc;
	unsigned short	tAcc;
	unsigned char		staticHoldThresh;
	unsigned char		dgpsTimeOut;
	unsigned char		cnoThreshNumSVs;	/**< (ubx7+ only, else 0) */
	unsigned char		cnoThresh;		/**< (ubx7+ only, else 0) */
	unsigned short	reserved;
	unsigned short	staticHoldMaxDist;	/**< (ubx8+ only, else 0) */
	unsigned char		utcStandard;		/**< (ubx8+ only, else 0) */
	unsigned char		reserved3;
	unsigned int	reserved4;
} ubx_payload_tx_cfg_nav5_t;

/* tx cfg-sbas */
typedef struct {
	unsigned char		mode;
	unsigned char		usage;
	unsigned char		maxSBAS;
	unsigned char		scanmode2;
	unsigned int	scanmode1;
} ubx_payload_tx_cfg_sbas_t;

/* Tx CFG-MSG */
typedef struct {
	union {
		unsigned short	msg;
		struct {
			unsigned char	msgClass;
			unsigned char	msgID;
		}id;
	}msg_union;
	unsigned char rate;
} ubx_payload_tx_cfg_msg_t;

/* General message and payload buffer union */
typedef union {
	ubx_payload_rx_nav_pvt_t		payload_rx_nav_pvt;
	ubx_payload_rx_nav_posllh_t		payload_rx_nav_posllh;
	ubx_payload_rx_nav_sol_t		payload_rx_nav_sol;
	ubx_payload_rx_nav_timeutc_t		payload_rx_nav_timeutc;
	ubx_payload_rx_nav_svinfo_part1_t	payload_rx_nav_svinfo_part1;
	ubx_payload_rx_nav_svinfo_part2_t	payload_rx_nav_svinfo_part2;
	ubx_payload_rx_nav_velned_t		payload_rx_nav_velned;
	ubx_payload_rx_nav_sat_t		payload_rx_nav_sat;
	ubx_payload_rx_mon_hw_ubx6_t		payload_rx_mon_hw_ubx6;
	ubx_payload_rx_mon_hw_ubx7_t		payload_rx_mon_hw_ubx7;
	ubx_payload_rx_mon_ver_part1_t		payload_rx_mon_ver_part1;
	ubx_payload_rx_mon_ver_part2_t		payload_rx_mon_ver_part2;
	ubx_payload_rx_ack_ack_t		payload_rx_ack_ack;
	ubx_payload_rx_ack_nak_t		payload_rx_ack_nak;
	ubx_payload_tx_cfg_prt_t		payload_tx_cfg_prt;
	ubx_payload_tx_cfg_rate_t		payload_tx_cfg_rate;
	ubx_payload_tx_cfg_nav5_t		payload_tx_cfg_nav5;
	ubx_payload_tx_cfg_sbas_t		payload_tx_cfg_sbas;
	ubx_payload_tx_cfg_msg_t		payload_tx_cfg_msg;
	ubx_payload_tx_cfg_tp5_t		payload_tx_cfg_tp5;
	ubx_payload_tx_cfg_itfm_t		payload_tx_cfg_itfm;
	unsigned char					raw[128];
} ubx_buf_t;

#pragma pack(pop)
/*** END OF u-blox protocol binary message and payload definitions ***/

/* Decoder state */
typedef enum {
	UBX_DECODE_SYNC1 = 0,
	UBX_DECODE_SYNC2,
	UBX_DECODE_CLASS,
	UBX_DECODE_ID,
	UBX_DECODE_LENGTH1,
	UBX_DECODE_LENGTH2,
	UBX_DECODE_PAYLOAD,
	UBX_DECODE_CHKSUM1,
	UBX_DECODE_CHKSUM2
} ubx_decode_state_t;

/* Rx message state */
typedef enum {
	UBX_RXMSG_IGNORE = 0,
	UBX_RXMSG_HANDLE,
	UBX_RXMSG_DISABLE,
	UBX_RXMSG_ERROR_LENGTH
} ubx_rxmsg_state_t;

/* ACK state */
typedef enum {
	UBX_ACK_IDLE = 0,
	UBX_ACK_WAITING,
	UBX_ACK_GOT_ACK,
	UBX_ACK_GOT_NAK
} ubx_ack_state_t;



struct vehicle_gps_position_s {
	unsigned long long timestamp_position;
	unsigned long long timestamp_timeplus;
	int	timestamp_error;
	int lat;
	int lon;
	int alt;
	unsigned long long timestamp_variance;
	float s_variance_m_s;
	float c_variance_rad;
	unsigned char fix_type;
	float eph;
	float epv;
	int noise_per_ms;
	int jamming_indicator;
	unsigned long long timestamp_velocity;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	float cog_rad;
	unsigned char vel_ned_valid;
	unsigned long long timestamp_time;

	unsigned long long time_utc_usec;
	unsigned char satellites_used;
};


//time.h

#define SAT_INFO_MAX_SATELLITES  32

struct satellite_info_s {
	unsigned long long timestamp;				/**< Timestamp of satellite info */
	unsigned char count;					/**< Number of satellites in satellite info */
	unsigned char svid[SAT_INFO_MAX_SATELLITES]; 		/**< Space vehicle ID [1..255], see scheme below  */
	unsigned char used[SAT_INFO_MAX_SATELLITES];		/**< 0: Satellite not used, 1: used for navigation */
	unsigned char elevation[SAT_INFO_MAX_SATELLITES];	/**< Elevation (0: right on top of receiver, 90: on the horizon) of satellite */
	unsigned char azimuth[SAT_INFO_MAX_SATELLITES];	/**< Direction of satellite, 0: 0 deg, 255: 360 deg. */
	unsigned char snr[SAT_INFO_MAX_SATELLITES];		/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite. */
};

#define CLOCK_REALTIME     0
struct tm
{
	unsigned int tm_sec;     /* second (0-61, allows for leap seconds) */
	unsigned int tm_min;     /* minute (0-59) */
	unsigned int tm_hour;    /* hour (0-23) */
	unsigned int tm_mday;    /* day of the month (1-31) */
	unsigned int tm_mon;     /* month (0-11) */
	unsigned int tm_year;    /* years since 1900 */
};

struct timespec
{
	unsigned int tv_sec;                   /* Seconds */
	unsigned int   tv_nsec;                  /* Nanoseconds */
};

struct timeval
{
	unsigned int tv_sec;                   /* Seconds */
	unsigned int tv_usec;                    /* Microseconds */
};
//end time.h

typedef struct{
	unsigned int	    iTOW;		/**< GPS Time of Week [ms] */
	unsigned short	    year; 		/**< Year (UTC)*/
	unsigned char		month; 		/**< Month, range 1..12 (UTC) */
	unsigned char		day; 		/**< Day of month, range 1..31 (UTC) */
	unsigned char		hour; 		/**< Hour of day, range 0..23 (UTC) */
	unsigned char		min; 		/**< Minute of hour, range 0..59 (UTC) */
	unsigned char		sec;		/**< Seconds of minute, range 0..60 (UTC) */
	unsigned char		valid; 		/**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
	unsigned int	    tAcc; 		/**< Time accuracy estimate (UTC) [ns] */
	int		            nano;		/**< Fraction of second (UTC) [-1e9...1e9 ns] */
	unsigned char		fixType;	/**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	unsigned char		flags;		/**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
	unsigned char		reserved1;
	unsigned char		numSV;		/**< Number of SVs used in Nav Solution */
	int		        lon;		/**< Longitude [1e-7 deg] */
	int		        lat;		/**< Latitude [1e-7 deg] */
	int		        height;		/**< Height above ellipsoid [mm] */
	int		        hMSL;		/**< Height above mean sea level [mm] */
	unsigned int	hAcc;  		/**< Horizontal accuracy estimate [mm] */
	unsigned int	vAcc;  		/**< Vertical accuracy estimate [mm] */
	int		        velN;		/**< NED north velocity [mm/s]*/
	int		        velE;		/**< NED east velocity [mm/s]*/
	int		        velD;		/**< NED down velocity [mm/s]*/
	int		        gSpeed;		/**< Ground Speed (2-D) [mm/s] */
	int		        headMot;	/**< Heading of motion (2-D) [1e-5 deg] */
	unsigned int	sAcc;		/**< Speed accuracy estimate [mm/s] */
	unsigned int	headAcc;	/**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
	unsigned short	pDOP;		/**< Position DOP [0.01] */
	unsigned short	reserved2;
	unsigned int	reserved3;
	int		        headVeh;    	/**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
	unsigned int	reserved4;	/**< (ubx8+ only) */
	unsigned int	timeplus_timestamp;
	unsigned int	position_timestamp;
	unsigned int	error_timestamp;
}gps_data;

typedef enum UBX_STATE_MACHINE{
	UBX_STATE_GET_BAUD_RATE,
	UBX_STATE_SET_BAUD_RATE,
	UBX_STATE_DEFINE_BAUD_RATE,
	UBX_STATE_SET_NAV5,
	UBX_STATE_SET_SBAS,
	UBX_STATE_SET_NAV_PVT,
	UBX_STATE_SET_NAV_TIMEUTC,
	UBX_STATE_SET_NAV_POSLLH,
	UBX_STATE_SET_NAV_SOL,
	UBX_STATE_SET_NAV_VELNED,
	UBX_STATE_CFG_TP5,
	UBX_STATE_MON_VER,
	UBX_STATE_CFG_ITFM,	
	UBX_STATE_CFG_HW_STATUS_OUT_RATE,
	UBX_STATE_CFG_SVINFO_OUT_RATE,
	UBX_STATE_CFG_SUCCESS,
	UBX_STATE_CFG_FAIL,
}Enum_UBX_STATE;

typedef	struct 
{
	/* dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite. */
	unsigned char snr[SAT_INFO_MAX_SATELLITES];
	/* svid */
	unsigned char svid[SAT_INFO_MAX_SATELLITES];
	/* SV is used for navigation */
	unsigned char used[SAT_INFO_MAX_SATELLITES];
}gps_sv_info_def;

typedef struct 
{
	unsigned short agcCnt;
	unsigned short noisePerMS;
	unsigned char jamInd;
	/*
		bit2-3:
		jammingState output from Jamming/Interference Monitor 
		(0 = unknown or feature disabled, 1 = ok - no significant jamming, 2
		= warning - interference visible but fix OK, 3 = critical - interference visible and no fix)
	*/
	unsigned char flags;
}gps_hw_status_def;

typedef struct 
{
	gps_hw_status_def hw_status;
	gps_sv_info_def sv_info;
	/*
		bit0:is hardware status updata.
		bit1:is sate info updata.

		note:after use, please set to 0;
	*/
	unsigned char flag;
}gps_signal_quality_def;

int gps_receive(void);
void gps_uart_init(void);
int gps_configure(void);
unsigned short gpio_input(unsigned char gpio,unsigned short pin);
extern int gps_parse_char(const unsigned char b);
unsigned char gps_transfer(gps_data* buf);
unsigned char ublox_status(void);
void gps_uart_send(unsigned char *buffer, unsigned char size);
//------------------------------End of File-------------------------------------
/* functions declare */
static int gps_heap_init(void);
static int gps_default_config(void);
static void gps_init_thread(void);
static void gps_task_thread( void );
/*----------------*/


#endif

/* end of file */


