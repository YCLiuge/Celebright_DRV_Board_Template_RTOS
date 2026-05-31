/* main.c file
Author: lisn3188
Website: www.chiplab7.com
Email: lisn3188@163.com
Development Environment: MDK-Lite  Version: 4.23
Date: 2012-04-25
Description: Implementation of mini IMU
Functionality: 
Static IMU
Calculates the static attitude based on the sensor values.
*/
#include "app.h"
#include "IMU.h"
#include "usart.h" 
#include "inv_imu_driver.h"
#include "config.h"
#include "car_attitude.h"
#include "read_aux_data_mode.h"
#include "sys_time.h"
#if APP_ENABLE_IMU
xyz_f_t north, west;
volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float q0, q1, q2, q3; // 四元数
volatile float integralFBhand, handdiff;
/* Timing now uses DWT->CYCCNT directly in IMU_AHRSupdate() */volatile float yaw[5] = {0, 0, 0, 0, 0};  // 偏航角值
int16_t Ax_offset = 0, Ay_offset = 0;
float TTangles_gyro[7]; // 陀螺仪角度
float Angle_Final[3]; // 最终角度
float imu_g_z;//z轴角速度

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


// 快速计算倒数平方根
float invSqrt1(float x) {
	float halfx = 0.5f * x;
	union { float f; long l; } y;
	y.f = x;
	y.l = 0x5f3759df - (y.l >> 1);
	y.f = y.f * (1.5f - (halfx * y.f * y.f));
	return y.f;
}


// 初始化IMU
void IMU_init(void) {	 
	if (0x00 == setup_imu(1, 1, 1)) {
		// 初始化四元数
		q0 = 1.0f;  // 初始化四元数
		q1 = 0.0f;
		q2 = 0.0f;
		q3 = 0.0f;
		exInt = 0.0;
		eyInt = 0.0;
		ezInt = 0.0;
		return;
	}
	printf("IMU ERROR!!\r\n");
}

static double Gyro_fill[3][300];
static double Gyro_total[3];
static double sqrGyro_total[3];
static int GyroinitFlag = 0;
static int GyroCount = 0;

// 计算方差
void calGyroVariance(float data[], int length, float sqrResult[], float avgResult[]) {
	int i;
	double tmplen;
	if (GyroinitFlag == 0) {
		for (i = 0; i < 3; i++) {
			Gyro_fill[i][GyroCount] = data[i];
			Gyro_total[i] += data[i];
			sqrGyro_total[i] += data[i] * data[i];
			sqrResult[i] = 100;
			avgResult[i] = 0;
		}
	} else {
		for (i = 0; i < 3; i++) {
			Gyro_total[i] -= Gyro_fill[i][GyroCount];
			sqrGyro_total[i] -= Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
			Gyro_fill[i][GyroCount] = data[i];
			Gyro_total[i] += Gyro_fill[i][GyroCount];
			sqrGyro_total[i] += Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
		}
	}
	GyroCount++;
	if (GyroCount >= length) {
		GyroCount = 0;
		GyroinitFlag = 1;
	}
	if (GyroinitFlag == 0) {
		return;
	}
	tmplen = length;
	for (i = 0; i < 3; i++) {
		avgResult[i] = (float)(Gyro_total[i] / tmplen);
		sqrResult[i] = (float)((sqrGyro_total[i] - Gyro_total[i] * Gyro_total[i] / tmplen) / tmplen);
	}
}
float gyro_offset[3] = {0};
int CalCount = 0;
/* 陀螺仪零偏自校准完成标志 (一次性 latch)。校准前 car_state.yaw 冻结,
 * 故任何依赖 yaw 的动作 (如 SPIN) 在校准完成前都不应启动。 */
static volatile _Bool s_imu_calibrated = 0;

int IMU_IsCalibrated(void) { return (int)s_imu_calibrated; }
/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 获取加速度、陀螺仪、磁力计的当前值  
输入参数:  存储数据的指针地址
输出参数:  无
*******************************************************************************/
void IMU_getValues(float * values) {
	float accgyroval[7];
	
	float sqrResult_gyro[3];
	float avgResult_gyro[3];
	// 获取加速度和陀螺仪的当前ADC值
	bsp_IcmGetRawData(accgyroval, &accgyroval[3], &accgyroval[6]);
	TTangles_gyro[0] =  accgyroval[0];
	TTangles_gyro[1] =  accgyroval[1];
	TTangles_gyro[2] =  accgyroval[2];
	TTangles_gyro[3] =  accgyroval[3];
	TTangles_gyro[4] =  accgyroval[4];
	TTangles_gyro[5] =  accgyroval[5];
	TTangles_gyro[6] =  accgyroval[6];
	
	calGyroVariance(&TTangles_gyro[3], 100, sqrResult_gyro, avgResult_gyro);
	if (sqrResult_gyro[0] < 0.02f && sqrResult_gyro[1] < 0.02f && sqrResult_gyro[2] < 0.02f && CalCount >= 99) {
		gyro_offset[0] = avgResult_gyro[0];
		gyro_offset[1] = avgResult_gyro[1];
		gyro_offset[2] = avgResult_gyro[2];
		exInt = 0;
		eyInt = 0;
		ezInt = 0;
		CalCount = 0;
		s_imu_calibrated = 1;
	} else if (CalCount < 100) {
		CalCount++;
	}
	values[0] =  accgyroval[0];
	values[1] =  accgyroval[1];
	values[2] =  accgyroval[2];
	values[3] =  accgyroval[3] - gyro_offset[0];
	values[4] =  accgyroval[4] - gyro_offset[1];
	values[5] =  accgyroval[5] - gyro_offset[2];
	imu_g_z=values[5];//z轴角速度
	if(s_imu_calibrated)
	{
		Car_Attitude_Yaw_Update(imu_g_z,0.001F*TASK_ITV_IMU);
	}
}


/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 四元数 
输入参数: 当前的传感器值
输出参数: 无
*******************************************************************************/
#define Kp 0.5f   // 比例增益，控制加速度计/磁力计的收敛速率
#define Ki 0.001f   // 积分增益，控制陀螺仪偏差的收敛速率

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float norm;
	float vx, vy, vz;
	float ex, ey, ez, halfT;
	float tempq0, tempq1, tempq2, tempq3;
	static uint32_t last_dwt = 0;
	uint32_t now_dwt = DWT->CYCCNT;
	uint32_t delta_dwt = now_dwt - last_dwt; /* unsigned subtraction handles wrap */
	float dt = (float)delta_dwt / 480000000.0f; /* seconds @ 480 MHz */
	halfT = dt * 0.5f;
	last_dwt = now_dwt;

	// 计算四元数的乘积
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;   
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	
	norm = invSqrt1(ax * ax + ay * ay + az * az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	// 将加速度计数据归一化

	norm = invSqrt1(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;

	// 计算重力方向
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	// 计算误差
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);

	if (ex != 0.0f || ey != 0.0f || ez != 0.0f) {
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;

		// PI控制器
		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;
	}

	// 四元数微分方程
	tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;  
	
	// 四元数归一化
	norm = invSqrt1(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 获取四元数 返回当前四元数的值
输入参数: 需要存储四元数的指针地址
输出参数: 无
*******************************************************************************/
float mygetqval[9];	// 用于存储传感器数据
void IMU_getQ(float * q) {
	IMU_getValues(mygetqval);	 
	// 将传感器的值转换为四元数
	IMU_AHRSupdate(mygetqval[3] * M_PI / 180, mygetqval[4] * M_PI / 180, mygetqval[5] * M_PI / 180,
								 mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

	q[0] = q0; // 返回当前四元数
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 获取四元数 返回当前姿态角
输入参数: 需要存储姿态角的指针地址
输出参数: 无
*******************************************************************************/
void IMU_getYawPitchRoll(float * angles) {
	float q[4]; // 四元数
	IMU_getQ(q); // 获取当前四元数
	
	angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) * 180 / M_PI; // 偏航角
	angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180 / M_PI; // 俯仰角
	angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 180 / M_PI; // 横滚角
}

void IMU_TT_getgyro(float * zsjganda) {
	zsjganda[0] = TTangles_gyro[0];
	zsjganda[1] = TTangles_gyro[1];
	zsjganda[2] = TTangles_gyro[2];
	zsjganda[3] = TTangles_gyro[3];
	zsjganda[4] = TTangles_gyro[4];
	zsjganda[5] = TTangles_gyro[5];
	zsjganda[6] = TTangles_gyro[6];
}

void MPU6050_InitAng_Offset(void) {
	// 初始化角度偏移
}

size_t IMU_BuildStatus(char *buf, size_t buf_len)
{
	size_t used;
	if ((buf == NULL) || (buf_len == 0U)) return 0U;

	used = (size_t)snprintf(buf, buf_len,
		"=== IMU Status ===\r\n"
		"Yaw:     %7.1f deg\r\n"
		"Pitch:   %7.1f deg\r\n"
		"Roll:    %7.1f deg\r\n"
		"Gyro Z:  %7.2f deg/s\r\n"
		"Accel:   %6.3f %6.3f %6.3f g\r\n"
		"Gyro:    %7.2f %7.2f %7.2f deg/s\r\n"
		"Quat:    %7.4f %7.4f %7.4f %7.4f\r\n"
		"GyroOff: %7.2f %7.2f %7.2f deg/s\r\n"
		"CalOK:   %s\r\n",
		(double)ypr[0], (double)ypr[1], (double)ypr[2],
		(double)imu_g_z,
		(double)motion6[0], (double)motion6[1], (double)motion6[2],
		(double)motion6[3], (double)motion6[4], (double)motion6[5],
		(double)q0, (double)q1, (double)q2, (double)q3,
		(double)gyro_offset[0], (double)gyro_offset[1], (double)gyro_offset[2],
		(gyro_offset[0] != 0.0f || gyro_offset[1] != 0.0f || gyro_offset[2] != 0.0f) ? "YES" : "NO");

	return (used < buf_len) ? used : (buf_len - 1U);
}
//------------------End of File----------------------------

#endif /* APP_ENABLE_IMU */
