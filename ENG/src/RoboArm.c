#include "RoboArm.h"
#include "LKMotoDriver.h"
#include "DJIMotoDriver.h"
#include "pid.h"
#include "arm_math.h"
#include "RcDriver.h"
#include "cmsis_os.h"
#include "JointReset.h"
#include "uart_zbw.h"


//pitch
#define ARM_ANGLE_MAX_1 45077
#define ARM_ANGLE_MIN_1 12650
#define ARM_ANGLE_STD_1 28492

//roll
#define ARM_ANGLE_MAX_2 42583
#define ARM_ANGLE_MIN_2 9858
#define ARM_ANGLE_STD_2 25823

//yall
#define ARM_ANGLE_MAX_3 40000
#define ARM_ANGLE_MIN_3 10000
#define ARM_ANGLE_STD_3 24370
/*
// �����÷�
// �������һά����
float32_t A[4] = {1,2,
									3,4};
float32_t B[4] = {4,3,
									2,1};
float32_t C[4];
	
//����ARM��������ָ��
arm_matrix_instance_f32 Matrix_A;
arm_matrix_instance_f32 Matrix_B;
arm_matrix_instance_f32 Matrix_C;
	

//��һά�����ַ�����ARM����ָ��
arm_mat_init_f32(&Matrix_A,2,2,(float32_t *)A);
arm_mat_init_f32(&Matrix_B,2,2,(float32_t *)B);
arm_mat_init_f32(&Matrix_C,2,2,(float32_t *)C);

//������� A*B = C
arm_mat_mult_f32(&Matrix_A,&Matrix_B,&Matrix_C);

//C���ӦΪ{8  5 
					 10 13}

*/

typedef struct
{
	float32_t x,y,z,pitch,raw,yaw;
}PoseParaTD;

PoseParaTD PosePara;

typedef struct
{
	float32_t theta1, theta2, theta3, a1, a2, a3;
}RoboArmParaTD;

typedef float32_t *DH_Matrix;

PidTD pid_lk_moto_spd[3];
PidTD pid_lk_moto_pos[3];
int16_t current_set[3];
int16_t dji_current_set[2];

float32_t
	EndToE_theta=0, EndToE_d=0, EndToE_a=0 ,EndToE_alpha=0,
	EToD_theta=0, EToD_d=0, EToD_a=0 ,EToD_alpha=0,
	DToC_theta=0, DToC_d=0, DToC_a=0,DToC_alpha=0,
	CToB_theta=0, CToB_d=0, CToB_a=0,CToB_alpha=0,
	BToA_theta=0, BToA_d=0, BToA_a=0,BToA_alpha=0,
	AToGround_theta=0, AToGround_d=0, AToGround_a=0,AToGround_alpha=0;



// ������
void RoboArm_Pid_Init(){
	for(int i=0;i<3;i++){
		pidInit(&pid_lk_moto_spd[i],10000,10000,0.3,0,0);
		pidInit(&pid_lk_moto_pos[i],10000,10000,0.8,0,0);
	}
}


void RoboArm_Pos_Init(){
	LKMotoState[0].angle_desired = ARM_ANGLE_STD_1;
	LKMotoState[1].angle_desired = ARM_ANGLE_STD_2;
	LKMotoState[2].angle_desired = ARM_ANGLE_STD_3;
}


extern PidTD pid_moto_pos[2];
extern PidTD pid_moto_spd[2];
// ���û�е��ĩ�˸�����Ƕȣ�ʹ�õ���ģʽ+˫��pidʵ�֣�
void Update_RoboArm_Pos(){
	// ǰ����
	for(int i=0;i<3;i++){
		pid_calculate(&pid_lk_moto_pos[i], (float)LKMotoState[i].angle_desired , (float)LKMotoState[i].encoder);
		LKMotoState[i].speed_desired = (int)pid_lk_moto_pos[i].outPID;
		pid_calculate(&pid_lk_moto_spd[i], (float)LKMotoState[i].speed_desired , (float)LKMotoState[i].speed);
		current_set[i] = (int)pid_lk_moto_spd[i].outPID;
	}
	
//	LKSetMotoCurrent(&hcan1,current_set[0],current_set[1],current_set[2],0); //�㲥ģʽroll����
	// ���������
	LKSetMotoCurrent_single(LK_Motor1_ID,current_set[0]);
	osDelay(1);
	LKSetMotoCurrent_single(LK_Motor2_ID,current_set[1]);
	osDelay(1);
	LKSetMotoCurrent_single(LK_Motor3_ID,current_set[2]);
	osDelay(1);
	
	if(qs_inited == true){
		// ǰ��
		pid_calculate(&pid_moto_pos[1], (float)MotoState[1].angle_desired , (float)MotoState[1].angle);
		MotoState[1].speed_desired = (int)pid_moto_pos[1].outPID;
		pid_calculate(&pid_moto_spd[1], (float)MotoState[1].speed_desired , (float)MotoState[1].speed_actual);
		dji_current_set[1] = (int)pid_moto_spd[1].outPID;
		SetMotoCurrent(&hcan1,Ahead,dji_current_set[0],dji_current_set[1],0,0);
	}
	
	if(hy_inited == true){
		// ����
		pid_calculate(&pid_moto_pos[0], (float)MotoState[0].angle_desired , (float)MotoState[0].angle);
		MotoState[0].speed_desired = (int)pid_moto_pos[0].outPID;
		pid_calculate(&pid_moto_spd[0], (float)MotoState[0].speed_desired , (float)MotoState[0].speed_actual);
		dji_current_set[0] = (int)pid_moto_spd[0].outPID;
		SetMotoCurrent(&hcan1,Ahead,dji_current_set[0],dji_current_set[1],0,0);
	}

}

// ң�ؿ���ǰ����
void RoboArm_RC_Ctrl(){
	LKMotoState[0].angle_desired += RC_CtrlData.rc.ch2 / 5;
	if(LKMotoState[0].angle_desired > ARM_ANGLE_MAX_1) LKMotoState[0].angle_desired = ARM_ANGLE_MAX_1;
	if(LKMotoState[0].angle_desired < ARM_ANGLE_MIN_1) LKMotoState[0].angle_desired = ARM_ANGLE_MIN_1;
	
	LKMotoState[1].angle_desired -= RC_CtrlData.rc.ch1 / 5;
	if(LKMotoState[1].angle_desired > ARM_ANGLE_MAX_2) LKMotoState[1].angle_desired = ARM_ANGLE_MAX_2;
	if(LKMotoState[1].angle_desired < ARM_ANGLE_MIN_2) LKMotoState[1].angle_desired = ARM_ANGLE_MIN_2;
	
	LKMotoState[2].angle_desired -= RC_CtrlData.rc.ch3 / 5;
	if(LKMotoState[2].angle_desired > ARM_ANGLE_MAX_3) LKMotoState[2].angle_desired = ARM_ANGLE_MAX_3;
	if(LKMotoState[2].angle_desired < ARM_ANGLE_MIN_3) LKMotoState[2].angle_desired = ARM_ANGLE_MIN_3;
	
	MotoState[1].angle_desired += RC_CtrlData.rc.ch4 * 2.5;
	if(MotoState[1].angle_desired > 780000) MotoState[1].angle_desired = 780000;
	if(MotoState[1].angle_desired < 10000) MotoState[1].angle_desired = 10000;
	
}


// ���ڿ���ǰ����
void RoboArm_UART_Ctrl(){
	LKMotoState[0].angle_desired =  sync_data_from_a.data.theta1;
	if(LKMotoState[0].angle_desired > ARM_ANGLE_MAX_1) LKMotoState[0].angle_desired = ARM_ANGLE_MAX_1;
	if(LKMotoState[0].angle_desired < ARM_ANGLE_MIN_1) LKMotoState[0].angle_desired = ARM_ANGLE_MIN_1;
	
	LKMotoState[1].angle_desired =  sync_data_from_a.data.theta2;
	if(LKMotoState[1].angle_desired > ARM_ANGLE_MAX_2) LKMotoState[1].angle_desired = ARM_ANGLE_MAX_2;
	if(LKMotoState[1].angle_desired < ARM_ANGLE_MIN_2) LKMotoState[1].angle_desired = ARM_ANGLE_MIN_2;
	
	LKMotoState[2].angle_desired =  sync_data_from_a.data.theta3;
	if(LKMotoState[2].angle_desired > ARM_ANGLE_MAX_3) LKMotoState[2].angle_desired = ARM_ANGLE_MAX_3;
	if(LKMotoState[2].angle_desired < ARM_ANGLE_MIN_3) LKMotoState[2].angle_desired = ARM_ANGLE_MIN_3;
	
	MotoState[1].angle_desired = sync_data_from_a.data.qs_pos;
	if(MotoState[1].angle_desired > 780000) MotoState[1].angle_desired = 780000;
	if(MotoState[1].angle_desired < 10000) MotoState[1].angle_desired = 10000;
	
	MotoState[0].angle_desired = sync_data_from_a.data.hy_pos;
	if(MotoState[0].angle_desired > 0) MotoState[0].angle_desired = 0;
	if(MotoState[0].angle_desired < -390000) MotoState[0].angle_desired = -390000;
}


// ��̬����� ԭʼ��̬6��x,y,z,pitch,raw,yaw Ŀ��6�� theta1 theta2 theta3 a1 a2 a3
void PoseCalculate_Converse(PoseParaTD* PosePara, RoboArmParaTD* RoboArmPara)
{
	



}


//����DH�任����theta,alpha��Ϊ�����ƣ�
void DH_Transform(DH_Matrix m, float32_t theta, float32_t d, float32_t a, float32_t alpha){
	float32_t (*cos)(float32_t x) = arm_cos_f32;
	float32_t (*sin)(float32_t x) = arm_sin_f32;
	
	float32_t m_temp[16] ={	cos(theta),							-sin(theta),						0,						a,
													sin(theta)*cos(alpha),	cos(theta)*cos(alpha),	-sin(alpha),	-sin(alpha)*d,
													sin(theta)*sin(alpha),	cos(theta)*sin(alpha),	cos(alpha),		cos(alpha)*d,
													0,											0,											0,												1};

	for(int i=0;i<16;i++) m[i] = m_temp[i];
}


//����ĩ������ϵ����������ϵ�ı任����
void Matrix_EndToGround_Calculate(arm_matrix_instance_f32* M_EndToGround){
	// ���ٿռ�
	float32_t end2e[16],e2d[16],d2c[16],c2b[16],b2a[16],a2g[16];
	
	DH_Matrix EndToE = end2e;
	DH_Matrix EToD = e2d;
	DH_Matrix DToC = d2c;
	DH_Matrix CToB = c2b;
	DH_Matrix BToA = b2a;
	DH_Matrix AToGround = a2g;
	
	arm_matrix_instance_f32 M_EndToE;
	arm_matrix_instance_f32 M_EToD;
	arm_matrix_instance_f32 M_DToC;
	arm_matrix_instance_f32 M_CToB;
	arm_matrix_instance_f32 M_BToA;
	arm_matrix_instance_f32 M_AToGround;
	
	arm_matrix_instance_f32 M_EndToD;
	arm_matrix_instance_f32 M_EndToC;
	arm_matrix_instance_f32 M_EndToB;
	arm_matrix_instance_f32 M_EndToA;
	
	
	// �����DH�任����
	DH_Transform(EndToE, EndToE_theta, EndToE_d, EndToE_a,EndToE_alpha);
	DH_Transform(EToD, EToD_theta, EToD_d, EToD_a, EToD_alpha);
	DH_Transform(DToC, DToC_theta, DToC_d, DToC_a, DToC_alpha);
	DH_Transform(CToB, CToB_theta, CToB_d, CToB_a, CToB_alpha);
	DH_Transform(BToA, BToA_theta, BToA_d, BToA_a, BToA_alpha);
	DH_Transform(AToGround, AToGround_theta, AToGround_d, AToGround_a, AToGround_alpha);
	
	
	// ת��Ϊarm�����ģ��
	arm_mat_init_f32(&M_EndToE,4,4,(float32_t *)EndToE);
	arm_mat_init_f32(&M_EToD,4,4,(float32_t *)EToD);
	arm_mat_init_f32(&M_DToC,4,4,(float32_t *)DToC);
	arm_mat_init_f32(&M_CToB,4,4,(float32_t *)CToB);
	arm_mat_init_f32(&M_BToA,4,4,(float32_t *)BToA);
	arm_mat_init_f32(&M_AToGround,4,4,(float32_t *)AToGround);
	
	// �����
	arm_mat_mult_f32(&M_EToD, &M_EndToE, &M_EndToD);
	arm_mat_mult_f32(&M_EToD, &M_EndToD, &M_EndToC);
	arm_mat_mult_f32(&M_EToD, &M_EndToC, &M_EndToB);
	arm_mat_mult_f32(&M_EToD, &M_EndToB, &M_EndToA);
	arm_mat_mult_f32(&M_EToD, &M_EndToA, M_EndToGround);
	
	

}
