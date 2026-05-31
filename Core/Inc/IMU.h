#ifndef __IMU_H
#define __IMU_H

#include <math.h>
#define M_PI  (float)3.1415926535
typedef struct
{
    float x;
    float y;
    float z;
} xyz_f_t;
extern xyz_f_t north,west;
extern volatile float yaw[5];   //�����������ֵ
extern float motion6[7];
extern float imu_g_z;//z轴角速度
//Mini IMU AHRS �����API
void IMU_init(void);
void IMU_getYawPitchRoll(float * ypr); //获取姿态
void IMU_TT_getgyro(float * zsjganda);
size_t IMU_BuildStatus(char *buf, size_t buf_len);  /* IMU status for CLI */
int IMU_IsCalibrated(void);  /* 陀螺仪零偏校准完成返回非0 */
//uint32_t micros(void);	//获取系统上电以来时间  单位 us 
void MPU6050_InitAng_Offset(void);
#endif

//------------------End of File----------------------------
