
#ifndef _PARAMETER_H_
#define _PARAMETER_H_


#ifndef U8
#define U8 unsigned char
#endif

#ifndef U16
#define U16 unsigned short
#endif


#ifndef U32
#define U32 unsigned int
#endif


#ifndef I8
#define I8 char
#endif

#ifndef I16
#define I16 short
#endif


#ifndef I32
#define I32 int
#endif







#define PARAMETER_MAGIC_NUMBER  0xf5760c43

#pragma pack(4)
typedef struct _PARAMETER_SAVE_ {
    U32 dwCrc32;
    U32 dwCrcComputeLen; // from dwMagicNum to last data of this struct
    U32 dwMagicNum;  // if the struct is valid, the magic number always is 0xf5760c43

    // Imu calibration parameter
    I16 asAccelOffset[3]; // x,y,z
    I16 asGyroOffset[3];  // x,y,z

    // MagnetoCalibration
    float afOffset[3];     // x,y,z
    float fSphereRadius;
    float afDiag[3];       // x,y,z
    float afOffsetDiag[3]; // x,y,z

    // other parameter to save here
    U8 abyRev[2048-64];
}PARAMETER_SAVE; // 2048 bytes
#pragma pack()





PARAMETER_SAVE *LoadParameter(void);
int SaveParameter(void);







#endif



