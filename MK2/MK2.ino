#include "MK2types.h"

#include <MatrixMath.h>
#include <string.h>

RobotJoint base, rArm, fArm, claw;  // 四个电机对象
RobotJoint * curServo;

int DSD = 5;                   // default servo delay,延时时长

MoveMode mode = TARGET_MODE;   // 默认为收取指令的模式
CoordType coordType = JOINT_TYPE;        // 默认为关节空间的运动

RobotData rbD;                 //   机械臂当前的参数

CartisianTran interpolationVal;

mtx_type Tbase_G[4][4];

void setup()
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			Tbase_G[i][j] = 0;

	base.servo.attach(11);      // base 伺服电机连接引脚11 电机代号'b'
	rArm.servo.attach(10);      // rArm 伺服电机连接引脚10 电机代号'r'
	fArm.servo.attach(9);       // fArm 伺服电机连接引脚9 电机代号'f'
	claw.servo.attach(6);       // claw 伺服电机连接引脚6 电机代号'c'

	// memset(&rbData, 1500, sizeof(RobotData));  // 角度值初始化
	base.setVal = 1472;
	rArm.setVal = 1472;
	fArm.setVal = 1472;
	claw.setVal = 1472;

	for (int i = 0; i < 4; i++) {
		rbD.setJointVal.jVal[i] = 1472;// 最终目标值
		rbD.curJointVal.jVal[i] = 1472;// 当前值，也就是下一时刻的目标值
	}

	base.servo.writeMicroseconds(base.setVal);
	rArm.servo.writeMicroseconds(rArm.setVal);
	fArm.servo.writeMicroseconds(fArm.setVal);
	claw.servo.writeMicroseconds(claw.setVal);   // 初始化引脚输出

	base.limitMax = &baseLimtMax;
	base.limitMin = &baseLimtMin;
	rArm.limitMax = &rArmLimtMax;
	rArm.limitMin = &rArmLimtMin;
	fArm.limitMax = &fArmLimtMax;
	fArm.limitMin = &fArmLimtMin;
	claw.limitMax = &clawLimtMax;
	claw.limitMin = &clawLimtMin;

	// 机械臂初始TCP位置 初始化
	rbD.curCartVal.cVal[0] = L2 + LTool;
	rbD.curCartVal.cVal[1] = 0;
	rbD.curCartVal.cVal[2] = L0 + L1;
	memcpy(&rbD.setCartVal, &rbD.curCartVal, sizeof(CartisianTran));

	// 注：如果不初始化输出，则默认输出为0，机械臂在上电、重启的时候，会有一段输出为0，
	// 也就是，pwm为0，此时舵机会转到0，而我们期望的是机械臂各个电机停在中位上
	Serial.begin(9600);
	Serial.setTimeout(10);  // 10ms的接收延时
	Serial.println("Please input serial data");
}

void loop()
{
	//接收数据
	if (Serial.available() > 0) {
		char serialCmd = Serial.read();  // 获取电机指令中电机编号信息
		commandPaser(serialCmd);         // 指令解析
	}

	// 处理数据
	switch (mode) // 不同模式有不同的处理方式
	{
	case CONTIOUS_MODE: // 计算下一个点
		switch (coordType)
		{
		case JOINT_TYPE:
			setJointVal(base, base.setVal + rbD.jointDir.jVal[0]);
			setJointVal(rArm, rArm.setVal + rbD.jointDir.jVal[1]);
			setJointVal(fArm, fArm.setVal + rbD.jointDir.jVal[2]);
			setJointVal(claw, claw.setVal + rbD.jointDir.jVal[3]);
			break;
		case CARTISIAN_TYPE:
			for (int i = 0; i < 3; i++) {
				rbD.setCartVal.cVal[i] = rbD.curCartVal.cVal[i] + rbD.cartDir.cVal[i] / 10.0;
			} // 在当前坐标基础上，计算下一个目标点

			if (InverseKinematics(rbD.setCartVal)) {
				// 如果执行逆解的目标点成功了，则修改当前值
				memcpy(&rbD.curCartVal,&rbD.setCartVal, sizeof(CartisianTran));
			}else {// 否则到达限位后，退回目标值，并不在运动
				coordType = JOINT_TYPE;
				memcpy(&rbD.setCartVal,&rbD.curCartVal,sizeof(CartisianTran));
				Serial.println("+INFO:cartisian is in limit!");
			}// 执行到目标点
			break;
		default:
			break;
		}
		break;
	case POINT_MODE:
		break;
	case TARGET_MODE:
		if (coordType == LINE_TYPE) {
			float posDiff = 0;  // 目标位置到当前位置的差值
			int inPlace = 0;    // 是否到位
			for (int i = 0; i < 3; i++) {
				posDiff = (rbD.setCartVal.cVal[i] - rbD.curCartVal.cVal[i]) * 10;
				if (abs(posDiff) < 1) {
					interpolationVal.cVal[i] = rbD.setCartVal.cVal[i];
					inPlace += 1;
				}
				else
					interpolationVal.cVal[i] = rbD.curCartVal.cVal[i] + (posDiff / abs(posDiff)) / 50;
			}

			if (inPlace < 3) {
				InverseKinematics(interpolationVal); // 执行到目标点
			}
			else {
				coordType = JOINT_TYPE;
				Serial.println("+INFO:Line in place!");
			}
		}// 计算下一个点
		break;
	default:
		break;
	}

	// 赋值当前设定的角度
	rbD.setJointVal.jVal[0] = base.setVal;
	rbD.setJointVal.jVal[1] = rArm.setVal;
	rbD.setJointVal.jVal[2] = fArm.setVal;
	rbD.setJointVal.jVal[3] = claw.setVal;        // 最终目标值
	rbD.state = Idle;                        // 默认是空闲

	for (int i = 0; i < 4; i++) {
		int inc = rbD.setJointVal.jVal[i] - rbD.curJointVal.jVal[i];
		if (inc != 0) {
			rbD.curJointVal.jVal[i] += inc / abs(inc);  // 计算下一个点位值
			rbD.state = Moving;  // 只要有一个变化，就是非空闲
		}
	}
	// 只有需要运动时，才会下发指令
	if (rbD.state == Moving) {
		base.servo.writeMicroseconds(rbD.curJointVal.jVal[0]);
		rArm.servo.writeMicroseconds(rbD.curJointVal.jVal[1]);
		fArm.servo.writeMicroseconds(rbD.curJointVal.jVal[2]);
		claw.servo.writeMicroseconds(rbD.curJointVal.jVal[3]);
		delay(DSD); // 控制运行速度
		//Serial.println(rbD.curVal.jVal[0]); // 发送数据耗时很严重
	}
}

/*
函数功能：指令解析
参数：指令名 serialCmd
*/
void commandPaser(char serialCmd) {
	float serialData;
	float serialDataFloat;
	if (serialCmd == 'b' || serialCmd == 'r' ||
		serialCmd == 'f' || serialCmd == 'c') {
		coordType = JOINT_TYPE;
		serialData = Serial.parseFloat();   // 获取指令信息
		switch (mode)
		{
		case CONTIOUS_MODE:
			for (int i = 0; i < 4; i++)
				rbD.jointDir.jVal[i] = .0;
			setDir(serialCmd, (int)serialData);  // 设定运动方向
			break;
		case POINT_MODE:
			switch (serialCmd)
			{
			case 'b':
				if (!setJointVal(base, base.setVal + (int)(serialData*10.31))) {
					Serial.println("+Warning:Base Servo Value Out Of Limit!");
				} // 执行指令信息
				break;
			case 'r':
				if (!setJointVal(rArm, rArm.setVal + (int)(serialData*10.31))) {
					Serial.println("+Warning:rArm Servo Value Out Of Limit!");
				} // 执行指令信息
				break;
			case 'f':
				if (!setJointVal(fArm, fArm.setVal + (int)(serialData*10.31))) {
					Serial.println("+Warning:fArm Servo Value Out Of Limit!");
				} // 执行指令信息
				break;
			case 'c':
				if (setJointVal(claw, claw.setVal + (int)(serialData*10.31))) {
					Serial.println("+Warning:claw Servo Value Out Of Limit!");
				} // 执行指令信息
				break;
			default:
				break;
			}
			break;
		case TARGET_MODE:
			servoPrint(serialCmd, (int)serialData);
			if (!servoCmd(serialCmd, serialData)) {
				Serial.print("+Warning:");
				Serial.print(serialCmd);
				Serial.print(" Value Out Of Limit!");
				Serial.println("");
			} // 执行指令信息
			break;
		default:
			break;
		}
		while (Serial.available()) { char wornStr = Serial.read(); };  // 清空缓冲区
		return;
	}

	if (serialCmd == 'x' || serialCmd == 'y' || serialCmd == 'z') {
		coordType = CARTISIAN_TYPE;
		switch (mode)
		{
		case CONTIOUS_MODE:   // 相当于沿 x、y、z ,一直走下去的speedl，要一直插值
			serialData = Serial.parseInt();   // 获取指令信息
			for (int i = 0; i < 3; i++)
				rbD.cartDir.cVal[i] = .0;
			// 需要计算一下当前的实时值，之后的值会在此基础上增加
			// rbD.curCartVal 的值
			forwardKinematics((45 - base.servo.read() / 2)* deg2rad, 
				(rArm.servo.read() - 90)* deg2rad, 
				(90 - fArm.servo.read())* deg2rad);
			setDir(serialCmd, serialData);    // 设定运动方向
			break;
		case TARGET_MODE:  // 相当于Ptp，到达空间中的某一个点
			serialDataFloat = Serial.parseFloat();   // 获取指令信息
			//servoCmd(serialCmd, serialData); ??     // 执行指令信息
			break;
		default:
			break;
		}
		while (Serial.available()) { char wornStr = Serial.read(); };  // 清空缓冲区
		return;
	}

	switch (serialCmd) {
	case 'm':  // 模式选择
		for (int i = 0; i < 3; i++) {
			rbD.jointDir.jVal[i] = .0;
			rbD.cartDir.cVal[i] = .0;
		}
		rbD.jointDir.jVal[3] = .0;
		// 切换模式的时候，一定要清零，否则再切换回来可能会继续运动
		serialData = Serial.parseFloat();   // 获取指令信息
		if ((int)serialData != NEXT_MODE) {
			setMode((int)serialData);
		}else {
			serialData = ((mode + 1) % (MODE_MAX - 1) + 1);
			setMode((int)serialData);
		}
		break;
	case 'v':  // 速度设置
		serialData = Serial.parseFloat();  // 获取电机指令中速度值
		Serial.println("");
		Serial.print("+Command:Set Velocity ");
		Serial.print(serialData);
		Serial.println("");
		if (serialData > 0) {
			DSD = (int)serialData;
		}
		break;
	case 'o':  // 机械臂信息输出
		reportStatus();
		break;
	case 'j':  // 关节角同时运动 joint：jb,r,f,c
		coordType = JOINT_TYPE;
		switch (mode)
		{
		case CONTIOUS_MODE:
			for (int i = 0; i < 4; i++)
				rbD.jointDir.jVal[i] = .0;
			setDir('b', Serial.parseInt());  // 设定运动方向
			serialData = Serial.read();  // 获取电机指令中电机编号信息
			setDir('r', Serial.parseInt());  // 设定运动方向
			serialData = Serial.read();  // 获取电机指令中电机编号信息
			setDir('f', Serial.parseInt());  // 设定运动方向
			serialData = Serial.read();  // 获取电机指令中电机编号信息
			setDir('c', Serial.parseInt());  // 设定运动方向
			break;
		case POINT_MODE:
			setJointVal(base, base.setVal + Serial.parseInt());
			serialData = Serial.read();  // 获取电机指令中电机编号信息
			setJointVal(rArm, rArm.setVal + Serial.parseInt());
			serialData = Serial.read();  // 获取电机指令中电机编号信息
			setJointVal(fArm, fArm.setVal + Serial.parseInt());
			serialData = Serial.read();  // 获取电机指令中电机编号信息
			setJointVal(claw, claw.setVal + Serial.parseInt());
			break;
		case TARGET_MODE:
			Serial.println("joint move");
			servoCmd('b', Serial.parseInt()); // 获取电机指令中速度值
			serialData = Serial.read();  // 获取电机指令中电机编号信息
			servoCmd('r', Serial.parseInt()); // 获取电机指令中速度值
			serialData = Serial.read();  // 获取电机指令中电机编号信息
			servoCmd('f', Serial.parseInt()); // 获取电机指令中速度值
			serialData = Serial.read();  // 获取电机指令中电机编号信息
			servoCmd('c', Serial.parseInt()); // 获取电机指令中速度值
			break;
		default:
			break;
		}
		break;
	case 'p':  // 末端位置模式，Ptp：px,y,z,c
		coordType = CARTISIAN_TYPE;
		switch (mode)
		{
		case CONTIOUS_MODE:  // 相当于 speedl
			break;
		case POINT_MODE:     // 相当于随便移动一点
			break;
		case TARGET_MODE:    // 相当于Ptp
			// 接收位置值
			for (int i = 0; i < 3; i++) {
				rbD.setCartVal.cVal[i] = Serial.parseFloat();
				serialData = Serial.read();
			}
			// 计算角度值逆解
			InverseKinematics(rbD.setCartVal); // 执行到目标点
			break;
		default:
			break;
		}
		break;
	case 'l': // 沿直线走到空间某点 line:lx,y,z
		for (int i = 0; i < 3; i++) {
			rbD.setCartVal.cVal[i] = Serial.parseFloat();
			serialData = Serial.read();
		}
		coordType = LINE_TYPE;
		break;
	default:  // 非已有模式，不再执行
		if (serialCmd == ' ') {
			while (Serial.available()) { char wornStr = Serial.read(); };  // 清空缓冲区
			return;
		}
		Serial.print("Unknow Command");
		Serial.print(serialCmd);
		Serial.println("");
		break;
	}
	while (Serial.available()) { char wornStr = Serial.read(); };  // 清空缓冲区
}

void setMode(int serialData) {
	switch (serialData) {
	case CONTIOUS_MODE:  // contious 连续运动模式
		mode = CONTIOUS_MODE;
		Serial.println("+Command:Set Move Mode: CONTIOUS_MODE");
		break;
	case POINT_MODE:  // 点动模式
		mode = POINT_MODE;
		Serial.println("+Command:Set Move Mode: POINT_MODE");
		break;
	case TARGET_MODE:  // target 目标点模式
		mode = TARGET_MODE;
		Serial.println("+Command:Set Move Mode: TARGET_MODE");
		break;
	default:
		Serial.print("Unknow Mode");
		break;
	}
}

/*机械臂正解*/
bool forwardKinematics(float j1, float j2, float j3) {
	Tbase_G[0][1] = sin(j1);
	Tbase_G[0][2] = cos(j1);
	Tbase_G[0][3] = cos(j1)*(sin(j2)*L1 + cos(j3)* L2 + LTool);
	Tbase_G[1][1] = -cos(j1);
	Tbase_G[1][2] = sin(j1);
	Tbase_G[1][3] = sin(j1)*(sin(j2)*L1 + cos(j3)* L2 + LTool);
	Tbase_G[2][3] = L0 + cos(j2)*L1 - sin(j3)*L2;

	rbD.curCartVal.cVal[0] = Tbase_G[0][3];
	rbD.curCartVal.cVal[1] = Tbase_G[1][3];
	rbD.curCartVal.cVal[2] = Tbase_G[2][3];
}

/*函数功能：求解当前设定位置的逆解*/
bool InverseKinematics(CartisianTran & cart) {
	float x = cart.cVal[0];
	float y = cart.cVal[1];
	float z = cart.cVal[2] - L0;

	float r = sqrt(x * x + y * y) - LTool;
	float bf2 = z * z + r * r;
	float bf = sqrt(bf2);
	float alpha1 = atan(z / r);
	float alpha2 = acos((L1 / bf + bf / L1 - L2 * L2 / (L1*bf)) / 2);
	float beta2 = acos((L1 / L2 + L2 / L1 - bf2 / (L1 * L2)) / 2);

	float angle_1 = atan(y / x) * rad2deg;
	float angle_2 = (PI / 2 - alpha1 - alpha2) * rad2deg;
	float angle_3 = (PI - beta2 - alpha1 - alpha2) * rad2deg;

	// 设定角度
	int jointValTemp[3]{ 0 };
	jointValTemp[0] = base.setVal;
	jointValTemp[1] = rArm.setVal;
	jointValTemp[2] = fArm.setVal;

	if (!servoCmd('b', angle_1) || !servoCmd('r', angle_2) || !servoCmd('f', angle_3)) {
		Serial.println("+Warning:IK fail!");
		base.setVal = jointValTemp[0];
		rArm.setVal = jointValTemp[1];
		fArm.setVal = jointValTemp[2];

		Serial.print("_base:");
		Serial.print(angle_1);
		Serial.print(cart.cVal[0]);
		Serial.print("_rArm:");
		Serial.print(angle_2);
		Serial.print(cart.cVal[1]);
		Serial.print("_fArm:");
		Serial.print(angle_3);
		Serial.print(cart.cVal[2]);
		Serial.println("");
		return false;
	}
	else {
		// 能到达，则为当前值赋值
		rbD.curCartVal.cVal[0] = cart.cVal[0];
		rbD.curCartVal.cVal[1] = cart.cVal[1];
		rbD.curCartVal.cVal[2] = cart.cVal[2];
	}
	return true;
}

/*
函数功能，调用伺服时，打印的信息
参数：伺服电机名，电机旋转角度
*/
void servoPrint(char servoName, int servoData) {
	Serial.println("");
	Serial.print("+Command:Servo\'");
	Serial.print(servoName);
	Serial.print("\'to\'");
	Serial.print(servoData);
	Serial.print("\'at servoDelay value \'");
	Serial.print(DSD);
	Serial.print("\'.");
	Serial.println("");
}

/*
函数功能，设定指定关节的下一个角度
参数：关节名，关节角 544-2400
*/
bool setJointVal(RobotJoint & joint, const int servoData) {
	if (servoData >= *joint.limitMin && servoData <= *joint.limitMax)
		joint.setVal = servoData;
	else
		return false;
	return true;
}

/*
函数功能：执行指定关节电机的运动
参数：
关节名称，b,r,f,c
关节角度，servoData
*/
bool servoCmd(char servoName, float servoData) {
	switch (servoName)
	{
	case 'b':   // 电机指令 'b' 设置base电机角度
		servoData = (int)(-servoData * 20.6222222 + 1472 + 0.5);  // 转换为目标值，+0.5是为 四舍五入
		return setJointVal(base, servoData);
	case 'r': // 电机指令 'r' 设置rArm电机角度
		servoData = (int)(servoData * 10.31111111 + 1472 + 0.5); // 转换为目标值
		return setJointVal(rArm, servoData);
	case 'f':// 电机指令 'f' 设置fArm电机角度
		servoData = (int)(1472 - servoData * 10.3111111 + 0.5); // 转换为目标值 
		return setJointVal(fArm, servoData);
	case 'c':// 电机指令 'c' 设置claw电机角度
		servoData = (int)(servoData * 10.3111111 + 544 + 0.5); // 转换为目标值
		return setJointVal(claw, servoData);
	}
}

/*
函数功能：设定某个轴的运动方向
参数:电机名，方向数值

*/
void setDir(char servoName, int servoData) {
	switch (servoName)
	{
	case 'b':
		rbD.jointDir.jVal[0] = servoData;
		break;
	case 'r': // 电机指令 'r' 设置rArm电机角度
		rbD.jointDir.jVal[1] = servoData;
		break;
	case 'f':// 电机指令 'f' 设置fArm电机角度
		rbD.jointDir.jVal[2] = servoData;
		break;
	case 'c':// 电机指令 'c' 设置claw电机角度
		rbD.jointDir.jVal[3] = servoData;
		break;
	case 'x': // x 方向的速度
		rbD.cartDir.cVal[0] = servoData;
		break;
	case 'y': // y 方向的速度
		rbD.cartDir.cVal[1] = servoData;
		break;
	case 'z': // z 方向的速度
		rbD.cartDir.cVal[2] = servoData;
		break;
	default:
		break;
	}
}

/*
打印当前机械臂信息
*/
void reportStatus() {  // 机械臂状态信息查看
	float j1 = (45 - base.servo.read() / 2);
	float j2 = (rArm.servo.read() - 90);
	float j3 = (90 - fArm.servo.read());
	forwardKinematics(j1* deg2rad, j2* deg2rad, j3* deg2rad);
	Serial.println("");
	Serial.println("+++ Robot-Arm Status Report ++++");
	Serial.print("Base Position:"); Serial.println(j1);
	Serial.print("Rear Arm Position:"); Serial.println(j2);
	Serial.print("Front Arm Position:"); Serial.println(j3);
	Serial.print("Claw Position:"); Serial.println(claw.servo.read() - 90);
	Serial.print("current X Position:"); Serial.println(rbD.curCartVal.cVal[0]);
	Serial.print("current Y Position:"); Serial.println(rbD.curCartVal.cVal[1]);
	Serial.print("current Z Position:"); Serial.println(rbD.curCartVal.cVal[2]);
	Serial.println("++++++++++++++++++++++++++++++++");
	Serial.println("");
}
