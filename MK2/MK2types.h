#pragma once

#include <Servo.h>
/*
使用说明
1、单关节运动：关节名+角度，例：b45 基座移动到45°角的位置
2、模式设置：m+模式+关节名+[方向]，例：m1b-1 基座以速度-1一直运动
3、a：减小选定关节的角度值
4、d：增加选定关节的角度值
5、设置运行速度：v+速度值；例，v10，下发间隔10ms；
6、运行到四个角度，例:j0, 12, 13, 14，四个电机直接运行到目标值
7、运行到空间位置x,y,z
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
const float LTool = 84;   // 工具坐标系

/*
运动响应模式，
*/
enum MoveMode
{
	NEXT_MODE = 0,   // 循环MODE
	CONTIOUS_MODE,   // 以固定速度连续运行
	POINT_MODE,      // 发送命令，运动一下
	TARGET_MODE,     //  运行到指令指定的角度、位置
	MODE_MAX,
};

// 坐标系
enum CoordType
{
	JOINT_TYPE,
	CARTISIAN_TYPE,
	LINE_TYPE
};

enum RobotState
{
	Ready = 0, // 无运动
	Moving,    // 运动中
	Idle       // 空闲
};

/*
电机角度值结构体
*/
typedef struct JointValue
{
	int jVal[4];        // 四个电机的角度值
}JointValue;

typedef struct CartisianTran
{
	float cVal[3];       
}CartisianTran;

/*
机械臂数据结构体
*/
typedef struct RobotJoint
{
	Servo servo;
	int setVal;           // 设定的角度值,电机的实际转数
	const int *limitMax;
	const int *limitMin;  // 减少空间占用
}RobotJoint;

typedef struct RobotData
{
	JointValue setJointVal;  //设定的角度值, 电机的实际转数
	JointValue curJointVal;  //设定的角度值,电机的实际转数
	JointValue jointDir;
	CartisianTran curCartVal;  // 当前空间位置
	CartisianTran setCartVal;  // 目标空间位置
	CartisianTran cartDir;            // 各方向的连续指令
	RobotState state;          // 机械臂当前所处状态
}RobotData;
