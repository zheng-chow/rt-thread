#ifndef __AHRS_HELPER_H__
#define __AHRS_HELPER_H__
#include <rtdef.h>
#include <math.h>
//#define Pi		acos(-1);
//#define ToDeg(rad)	(rad*180/Pi)
//#define ToRad(deg)	(deg*Pi/180)
#define ToDeg(rad)	    (rad*57.29577951f)
#define ToRad(deg)	    (deg*0.017453293f)

typedef struct Quaternion{
	float q0;
	float q1;
	float q2;
	float q3;	
}Quaternion_t;

typedef struct Vector{
	float theta;
	float ax;
	float ay;
	float az;	
}Vector_t;

typedef struct Eular{
	float roll;
	float pitch;
	float yaw;
}Eular_t;

typedef struct Accel{
	float X;
	float Y;
	float Z;
}Accel_t;

typedef struct Gyro{
	float X;
	float Y;
	float Z;
}Gyro_t;

typedef struct ImuAHRS{
	Quaternion_t 	QS;
    
	Gyro_t			gyro;
	Accel_t			accel;
	Eular_t			eular;
    
	float			exInt;
	float			eyInt;
	float			ezInt;
	
	float			halfT;
}ImuAHRS_t;

void Q_Reset(Quaternion_t* q);
void Q_Norm(Quaternion_t* q);
void Q_FromVector(Vector_t* v, Quaternion_t* q);
void Q_Mul(Quaternion_t* qa,Quaternion_t* qb, Quaternion_t* q);
void Q_Conj(Quaternion_t* q,Quaternion_t* cq);
void Q_Inv(Quaternion_t* q,Quaternion_t* iq);
void Q_Div(Quaternion_t* qa,Quaternion_t* qb, Quaternion_t* q);
void Q_ToEularGeneral(Quaternion_t* q, Eular_t* e);
void Q_ToEular(Quaternion_t* q, Eular_t* e);
void IMUupdate(ImuAHRS_t* imu);
void Q_FromEularGeneral(Eular_t* e, Quaternion_t* q);
void Q_ToEularGeneral(Quaternion_t* q, Eular_t* e);

#endif
