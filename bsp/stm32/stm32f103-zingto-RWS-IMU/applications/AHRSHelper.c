#include "AHRSHelper.h"

/************************************************
四元数相关计算函数
************************************************/

void Q_Reset(Quaternion_t* q){
	q->q0 = 1;
	q->q1 = 0;
	q->q2 = 0;
	q->q3 = 0;
}

void Q_Norm(Quaternion_t* q){
	float norm = sqrt(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
	q->q0 /=  norm;
	q->q1 /=  norm;
	q->q2 /=  norm;
	q->q3 /=  norm;
}

void Q_FromVector(Vector_t* v, Quaternion_t* q){
	float halfTheta = v->theta/2;
	q->q0 = cosf(halfTheta);
	q->q1 = v->ax * sinf(halfTheta);
	q->q2 = v->ay * sinf(halfTheta);
	q->q3 = v->az * sinf(halfTheta);
}

void Q_Mul(Quaternion_t* qa,Quaternion_t* qb, Quaternion_t* q){
	q->q0 = qa->q0*qb->q0 - qa->q1*qb->q1 - qa->q2*qb->q2 - qa->q3*qb->q3;
	q->q1 = qa->q0*qb->q1 + qa->q1*qb->q0 + qa->q2*qb->q3 - qa->q3*qb->q2;
	q->q2 = qa->q0*qb->q2 - qa->q1*qb->q3 + qa->q2*qb->q0 + qa->q3*qb->q1;
	q->q3 = qa->q0*qb->q3 + qa->q1*qb->q2 - qa->q2*qb->q1 + qa->q3*qb->q0;
}

void Q_Conj(Quaternion_t* q,Quaternion_t* cq){
	cq->q0 = q->q0;
	cq->q1 = -q->q1;
	cq->q2 = -q->q2;
	cq->q3 = -q->q3;	
}

void Q_INV(Quaternion_t* q,Quaternion_t* iq){
	float norm2 = q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3;
	iq->q0 = q->q0/norm2;
	iq->q1 = -q->q1/norm2;
	iq->q2 = -q->q2/norm2;
	iq->q3 = -q->q3/norm2;	
}

void Q_Div(Quaternion_t* qa,Quaternion_t* qb, Quaternion_t* q){
	Quaternion_t qt;
	Q_INV(qb,&qt);
	Q_Mul(qa,&qt,q);
}

/*void Q_ToEularGeneral(Quaternion_t* q, Eular_t* e){
	//e->roll = atan2f(2*(q->q0*q->q1+q->q2*q->q3),1-2*(q->q1*q->q1+q->q2*q->q2));
	e->roll = atan2f(2*(q->q0*q->q1+q->q2*q->q3),(q->q0*q->q0+q->q3*q->q3)-(q->q1*q->q1+q->q2*q->q2));
	e->pitch = asinf(2*(q->q0*q->q2-q->q3*q->q1));
	//e->yaw = atan2f(2*(q->q0*q->q3+q->q1*q->q2),1-2*(q->q2*q->q2+q->q3+q->q3));
	e->yaw = atan2f(2*(q->q0*q->q3+q->q1*q->q2),(q->q0*q->q0+q->q1*q->q1)-(q->q2*q->q2+q->q3*q->q3));
}*/

/*void Q_ToEularGeneral(Quaternion_t* q, Eular_t* e){
	e->roll = atan2f(2*(q->q0*q->q1+q->q2*q->q3),(q->q0*q->q0+q->q3*q->q3)-(q->q1*q->q1+q->q2*q->q2));
	e->pitch = asinf(2*(q->q0*q->q2-q->q3*q->q1));
	e->yaw = atan2f(2*(q->q0*q->q3+q->q1*q->q2),(q->q0*q->q0+q->q1*q->q1)-(q->q2*q->q2+q->q3*q->q3));
}*/

static void Q_ToEularHelper(float* r, float* e){
	// r : r11 r12 r21 r31 r32
	e[0] = atan2f( r[0], r[1]);
	e[1] = asinf(r[2]);
	e[2] = atan2f( r[3], r[4]);
}

void Q_ToEularGeneralYXZ(Quaternion_t* q, Eular_t* e){//YXZ
	float r[5];
	float eu[3];
	r[0] = 2*(q->q1*q->q3+q->q0*q->q2);
	r[1] = (q->q0*q->q0+q->q3*q->q3)-(q->q1*q->q1+q->q2*q->q2);
	r[2] = -2*(q->q2*q->q3-q->q0*q->q1);
	r[3] = 2*(q->q1*q->q2+q->q0*q->q3);
	r[4] = (q->q0*q->q0+ q->q2*q->q2) -(q->q1*q->q1 + q->q3*q->q3);	
	Q_ToEularHelper(r,eu);
	e->yaw 		= eu[2];
	if (fabsf(eu[2]) >= cos(-1)/2){
		e->roll 	= -eu[1];
		e->pitch 	= -eu[0];
	}
	else{
		e->roll 	= eu[1];
		e->pitch 	= eu[0];
	}
}

void Q_ToEularGeneral(Quaternion_t* q, Eular_t* e){//ZXY
	float r[5];
	float eu[3];
	
	r[0] = 2*(q->q0*q->q3-q->q1*q->q2);
	r[1] = (q->q0*q->q0+q->q2*q->q2)-(q->q1*q->q1+q->q3*q->q3);
	r[2] = 2*(q->q2*q->q3+q->q0*q->q1);
	r[3] = 2*(q->q0*q->q2-q->q1*q->q3);
	r[4] = (q->q0*q->q0+ q->q3*q->q3) -(q->q1*q->q1 + q->q2*q->q2);	
	Q_ToEularHelper(r,eu);
	e->yaw 		= eu[0];
	e->roll 	= eu[1];
	e->pitch 	= eu[2];
}

void Q_FromEularGeneralYXZ(Eular_t* e, Quaternion_t* q){
	float hp = ToRad(e->pitch/2);
	float hr = ToRad(e->roll/2);
	float hy = ToRad(e->yaw/2);
	
	float sp = sinf(hp), cp = cosf(hp);
	float sr = sinf(hr), cr = cosf(hr);
	float sy = sinf(hy), cy = cosf(hy);
	q->q0 = cp * cr * cy + sp * sr * sy;
	q->q1 = cp * sr * cy + sp * cr * sy;
	q->q2 = sp * cr * cy - cp * sr * sy;
	q->q3 = cp * cr * sy - sp * sr * cy;
}

void Q_FromEularGeneral(Eular_t* e, Quaternion_t* q){
	float hp = ToRad(e->pitch/2);
	float hr = ToRad(e->roll/2);
	float hy = ToRad(e->yaw/2);
	
	float sp = sinf(hp), cp = cosf(hp);
	float sr = sinf(hr), cr = cosf(hr);
	float sy = sinf(hy), cy = cosf(hy);
	
	q->q0 = cy * cr * cp - sy * sr * sp;
	q->q1 = cy * sr * cp - sy * cr * sp;
	q->q2 = cy * cr * sp + sy * sr * cp;
	q->q3 = cy * sr * sp + sy * cr * cp;
}
/*
void Q_ToEularGeneral(Quaternion_t* q, Eular_t* e){
    const float rollthrshold = 3.14159265f /2 * 0.1;
	float r11 = 2*(q->q1*q->q3+q->q0*q->q2);
	float r12 = (q->q0*q->q0+q->q3*q->q3)-(q->q1*q->q1+q->q2*q->q2);
	float r21 = -2*(q->q2*q->q3-q->q0*q->q1);
	float r31 = 2*(q->q1*q->q2+q->q0*q->q3);
	float r32 = (q->q0*q->q0+ q->q2*q->q2) -(q->q1*q->q1 + q->q3*q->q3);	
	
	float roll = asin(r21);//acos( r21 );
	float pitch = atan2f( r11, r12 );
	float yaw = atan2( r31, r32 );
    
    if (fabsf(roll) < rollthrshold){
        e->roll = roll;
        e->pitch = pitch;
        e->yaw = yaw;
    }
    else{
        float pitch1 = asinf(2*(q->q0*q->q2-q->q3*q->q1));
        e->roll = roll;
        e->pitch = pitch1;
        e->yaw = yaw + pitch1 - pitch;
   }
}*/



void Q_ToEular(Quaternion_t* q, Eular_t* e){
	e->roll = atan2f(2*(q->q2*q->q3-q->q0*q->q1),(q->q0*q->q0+q->q3*q->q3)-(q->q1*q->q1+q->q2*q->q2));
	e->pitch = asinf(-2*(q->q0*q->q2+q->q3*q->q1));
	e->yaw = atan2f(-2*(q->q0*q->q3-q->q1*q->q2),(q->q0*q->q0+q->q1*q->q1)-(q->q2*q->q2+q->q3*q->q3));
}

/************************************************
传感器相关评价函数
************************************************/
static float AccelValid(Accel_t* accel){

#define ACCEL_WINDOW_SZ 6
#define ACCEL_STD_THRSHOLD 0.0005f
    
	static Accel_t accelHis[ACCEL_WINDOW_SZ];
	static rt_uint8_t winIdx = 0;
	Accel_t accelMean = {0.f, 0.f, 0.f};
	Accel_t accelStd = {0.f, 0.f, 0.f};	
	float   accelNorm = 0;
	//push data
	accelHis[winIdx++] = *accel;
    
	if (winIdx == ACCEL_WINDOW_SZ)
		winIdx = 0;
    
	accelNorm = sqrt(accel->X*accel->X + accel->Y*accel->Y + accel->Z*accel->Z);
    
	if ((accelNorm > 1.15f) || (accelNorm < 0.85f))
			return 0;
	
	for (rt_uint8_t n = 0; n < ACCEL_WINDOW_SZ; n++){
		accelMean.X += accelHis[n].X;
		accelMean.Y += accelHis[n].Y;
		accelMean.Z += accelHis[n].Z;		
	}
    
	accelMean.X /= ACCEL_WINDOW_SZ;
	accelMean.Y /= ACCEL_WINDOW_SZ;
	accelMean.Z /= ACCEL_WINDOW_SZ;
    
	for (rt_uint8_t n = 0; n < ACCEL_WINDOW_SZ; n++){
		accelStd.X += (accelHis[n].X - accelMean.X)*(accelHis[n].X - accelMean.X);
		accelStd.Y += (accelHis[n].Y - accelMean.Y)*(accelHis[n].Y - accelMean.Y);
		accelStd.Z += (accelHis[n].Z - accelMean.Z)*(accelHis[n].Z - accelMean.Z);
	}
    
	accelStd.X /= ACCEL_WINDOW_SZ;
	accelStd.Y /= ACCEL_WINDOW_SZ;
	accelStd.Z /= ACCEL_WINDOW_SZ;
    
	if ((accelStd.X > ACCEL_STD_THRSHOLD) || 
        (accelStd.Y > ACCEL_STD_THRSHOLD) || 
        (accelStd.Z > ACCEL_STD_THRSHOLD))
		return 0;
    
	return accelNorm;
}

void IMUupdate(ImuAHRS_t* imu){
#define KpAcc 0.707f// proportional gain governs rate of convergence to accelerometer/magnetometer  2.0f->
#define KiAcc 0.005f// integral gain governs rate of convergence of gyroscope biases  0.005f ->
    
	Accel_t     accelTmp = imu->accel;
	Gyro_t	        gyro = imu->gyro;
	Eular_t*        eular = &imu->eular;
	Quaternion_t*   Q = &imu->QS;
	
	float           vx=0.f, vy=0.f, vz=0.f;
	float           ex=0.f, ey=0.f, ez=0.f;  
	float           Kp = KpAcc;
	float           Ki = KiAcc;
	float           accelNorm = sqrt(accelTmp.X*accelTmp.X + accelTmp.Y*accelTmp.Y + accelTmp.Z*accelTmp.Z);//accelNorm = AccelValid(&imu->accel);
    
	if ((accelNorm > 0.8f) &&(accelNorm < 1.2f) )
  {//accel data is valid, use to eliminating static error
        
    //accelNorm = sqrt(accelTmp.X*accelTmp.X + accelTmp.Y*accelTmp.Y + accelTmp.Z*accelTmp.Z);
		accelTmp.X /= accelNorm;
		accelTmp.Y /= accelNorm;
		accelTmp.Z /= accelNorm;
		// estimated direction of gravity
		vx = 2*(Q->q1*Q->q3 - Q->q0*Q->q2);
		vy = 2*(Q->q0*Q->q1 + Q->q2*Q->q3);
		vz = Q->q0*Q->q0 - Q->q1*Q->q1 - Q->q2*Q->q2 + Q->q3*Q->q3;
		/************************************
		添加评价重力运动程度评价，避免重力误差过大
		************************************/
		ex = (accelTmp.Y*vz - accelTmp.Z*vy);
		ey = (accelTmp.Z*vx - accelTmp.X*vz);
		ez = (accelTmp.X*vy - accelTmp.Y*vx);
		// integral error scaled integral gain
		imu->exInt = imu->exInt + ex*Ki*imu->halfT;
		imu->eyInt = imu->eyInt + ey*Ki*imu->halfT;
		imu->ezInt = imu->ezInt + ez*Ki*imu->halfT;
		/************************************
		添加评价重力方向部分，避免因为重力导致的，沿重力轴向的旋转累积误差
		************************************/
		if (fabsf(vx) > 0.7f)
			gyro.X = gyro.X + Kp*ex;
		else
			gyro.X = gyro.X + Kp*ex + imu->exInt;
        
		if (fabsf(vy) > 0.7f)
			gyro.Y = gyro.Y + Kp*ey;
		else
			gyro.Y = gyro.Y + Kp*ey + imu->eyInt;
        
		if (fabsf(vz) > 0.7f)
			gyro.Z = gyro.Z + Kp*ez;
		else
			gyro.Z = gyro.Z + Kp*ez + imu->ezInt;
	}
    
	// integrate quaternion rate and normalise
	Q->q0 = Q->q0 + (-Q->q1*gyro.X - Q->q2*gyro.Y - Q->q3*gyro.Z)*imu->halfT;
	Q->q1 = Q->q1 + ( Q->q0*gyro.X + Q->q2*gyro.Z - Q->q3*gyro.Y)*imu->halfT;
	Q->q2 = Q->q2 + ( Q->q0*gyro.Y - Q->q1*gyro.Z + Q->q3*gyro.X)*imu->halfT;
	Q->q3 = Q->q3 + ( Q->q0*gyro.Z + Q->q1*gyro.Y - Q->q2*gyro.X)*imu->halfT;  	
	Q_Norm(Q);	
	Q_ToEularGeneral(Q, eular);//General
}
