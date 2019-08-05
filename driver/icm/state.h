
#ifndef __IMU_H__
#define __IMU_H__

#define DEG2RAD		(0.017453293f)	/* deg to rad */
#define RAD2DEG		(57.29578f)		  /* rad to deg */

/* mpu9250 ins sensor default */
typedef struct{
	float accel[3];
	float gyro[3];
	float icm206_temperature;
}ICM206_INS_DEF;
/* mpu9250 mag def */
typedef struct{
	float mag[3];
}MPU9250_MAG_DEF;
/* attitude define */
typedef struct{
	float roll;
	float pitch;
	float yaw;
}ATTITUDE_DEF;
/* redef */
typedef struct
{
	/* analog channel */
  unsigned short channel[8];
  /* digital channel */
  unsigned short  channel567;
  /* unique id */
  unsigned short unique_id;
  /* crc */
  unsigned short crc ;
  /*-------------------------*/
}rcs_HandleTypeDef;
/* gps def */
typedef struct
{
	/* position */
	double		lon;			/**&lt; Longitude [deg] */
	double		lat;			/**&lt; Latitude [deg] */
	float		  height;		/**&lt; Height above mean sea level [m] */
	/* time stamp */
	unsigned int	position_timestamp;
  /* velocity */
	float		velN;		/**&lt; NED north velocity [m/s]*/
	float		velE;		/**&lt; NED east velocity [m/s]*/
	float		velU;		/**&lt; NED up velocity [m/s]*/
  /* pos dop */
	float		pDOP;		/**&lt; Position DOP [0.01] */
	/* rev */
	unsigned int rev1;
	unsigned int rev2;
	unsigned int rev3;
	unsigned int rev4;	
  /* fixtype */
	unsigned char		fixType;		/**&lt; GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	unsigned char		numSV;			/**&lt; Number of SVs used in Nav Solution */
	/* end of data */
}GPS_User_t;
/* system math def */
typedef struct
{
	ICM206_INS_DEF ins;
	GPS_User_t gps;
}state_def;

#endif
















