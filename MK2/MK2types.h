#pragma once

#include <Servo.h>
/*
ʹ��˵��
1�����ؽ��˶����ؽ���+�Ƕȣ�����b45 �����ƶ���45��ǵ�λ��
2��ģʽ���ã�m+ģʽ+�ؽ���+[����]������m1b-1 �������ٶ�-1һֱ�˶�
3��a����Сѡ���ؽڵĽǶ�ֵ
4��d������ѡ���ؽڵĽǶ�ֵ
5�����������ٶȣ�v+�ٶ�ֵ������v10���·����10ms��
6�����е��ĸ��Ƕȣ���:j0, 12, 13, 14���ĸ����ֱ�����е�Ŀ��ֵ
7�����е��ռ�λ��x,y,z
*/
//const int baseLimtMin = -45;
//const int baseLimtMax = 45;
//const int rArmLimtMin = -45;
//const int rArmLimtMax = 70;
//const int fArmLimtMin = -30;
//const int fArmLimtMax = 50;
//const int clawLimtMin = 0;
//const int clawLimtMax = 180;

const float rad2deg = 57.196;
const float deg2rad = 0.0175;

const int baseLimtMin = 544;
const int baseLimtMax = 2400;
const int rArmLimtMin = 1008;
const int rArmLimtMax = 2194;
const int fArmLimtMin = 956;
const int fArmLimtMax = 1781;
const int clawLimtMin = 544;
const int clawLimtMax = 2400;

const float L0 = 100;
const float L1 = 135;
const float L2 = 147;
const float LTool = 84;   // ��������ϵ

/*
�˶���Ӧģʽ��
*/
enum MoveMode
{
	NEXT_MODE = 0,   // ѭ��MODE
	CONTIOUS_MODE,   // �Թ̶��ٶ���������
	POINT_MODE,      // ��������˶�һ��
	TARGET_MODE,     //  ���е�ָ��ָ���ĽǶȡ�λ��
	MODE_MAX,
};

// ����ϵ
enum CoordType
{
	JOINT_TYPE,
	CARTISIAN_TYPE,
	LINE_TYPE
};

enum RobotState
{
	Ready = 0, // ���˶�
	Moving,    // �˶���
	Idle       // ����
};

/*
����Ƕ�ֵ�ṹ��
*/
typedef struct JointValue
{
	int jVal[4];        // �ĸ�����ĽǶ�ֵ
}JointValue;

typedef struct CartisianTran
{
	float cVal[3];       
}CartisianTran;

/*
��е�����ݽṹ��
*/
typedef struct RobotJoint
{
	Servo servo;
	int setVal;           // �趨�ĽǶ�ֵ,�����ʵ��ת��
	const int *limitMax;
	const int *limitMin;  // ���ٿռ�ռ��
}RobotJoint;

typedef struct RobotData
{
	JointValue setJointVal;  //�趨�ĽǶ�ֵ, �����ʵ��ת��
	JointValue curJointVal;  //�趨�ĽǶ�ֵ,�����ʵ��ת��
	JointValue jointDir;
	CartisianTran curCartVal;  // ��ǰ�ռ�λ��
	CartisianTran setCartVal;  // Ŀ��ռ�λ��
	CartisianTran cartDir;            // �����������ָ��
	RobotState state;          // ��е�۵�ǰ����״̬
}RobotData;
