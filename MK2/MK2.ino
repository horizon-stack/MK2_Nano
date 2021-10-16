#include "MK2types.h"

#include <MatrixMath.h>
#include <string.h>

RobotJoint base, rArm, fArm, claw;  // �ĸ��������
RobotJoint * curServo;

int DSD = 5;                   // default servo delay,��ʱʱ��

MoveMode mode = TARGET_MODE;   // Ĭ��Ϊ��ȡָ���ģʽ
CoordType coordType = JOINT_TYPE;        // Ĭ��Ϊ�ؽڿռ���˶�

RobotData rbD;                 //   ��е�۵�ǰ�Ĳ���

CartisianTran interpolationVal;

mtx_type Tbase_G[4][4];

void setup()
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			Tbase_G[i][j] = 0;

	base.servo.attach(11);      // base �ŷ������������11 �������'b'
	rArm.servo.attach(10);      // rArm �ŷ������������10 �������'r'
	fArm.servo.attach(9);       // fArm �ŷ������������9 �������'f'
	claw.servo.attach(6);       // claw �ŷ������������6 �������'c'

	// memset(&rbData, 1500, sizeof(RobotData));  // �Ƕ�ֵ��ʼ��
	base.setVal = 1472;
	rArm.setVal = 1472;
	fArm.setVal = 1472;
	claw.setVal = 1472;

	for (int i = 0; i < 4; i++) {
		rbD.setJointVal.jVal[i] = 1472;// ����Ŀ��ֵ
		rbD.curJointVal.jVal[i] = 1472;// ��ǰֵ��Ҳ������һʱ�̵�Ŀ��ֵ
	}

	base.servo.writeMicroseconds(base.setVal);
	rArm.servo.writeMicroseconds(rArm.setVal);
	fArm.servo.writeMicroseconds(fArm.setVal);
	claw.servo.writeMicroseconds(claw.setVal);   // ��ʼ���������

	base.limitMax = &baseLimtMax;
	base.limitMin = &baseLimtMin;
	rArm.limitMax = &rArmLimtMax;
	rArm.limitMin = &rArmLimtMin;
	fArm.limitMax = &fArmLimtMax;
	fArm.limitMin = &fArmLimtMin;
	claw.limitMax = &clawLimtMax;
	claw.limitMin = &clawLimtMin;

	// ��е�۳�ʼTCPλ�� ��ʼ��
	rbD.curCartVal.cVal[0] = L2 + LTool;
	rbD.curCartVal.cVal[1] = 0;
	rbD.curCartVal.cVal[2] = L0 + L1;
	memcpy(&rbD.setCartVal, &rbD.curCartVal, sizeof(CartisianTran));

	// ע���������ʼ���������Ĭ�����Ϊ0����е�����ϵ硢������ʱ�򣬻���һ�����Ϊ0��
	// Ҳ���ǣ�pwmΪ0����ʱ�����ת��0���������������ǻ�е�۸������ͣ����λ��
	Serial.begin(9600);
	Serial.setTimeout(10);  // 10ms�Ľ�����ʱ
	Serial.println("Please input serial data");
}

void loop()
{
	//��������
	if (Serial.available() > 0) {
		char serialCmd = Serial.read();  // ��ȡ���ָ���е�������Ϣ
		commandPaser(serialCmd);         // ָ�����
	}

	// ��������
	switch (mode) // ��ͬģʽ�в�ͬ�Ĵ���ʽ
	{
	case CONTIOUS_MODE: // ������һ����
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
			} // �ڵ�ǰ��������ϣ�������һ��Ŀ���

			if (InverseKinematics(rbD.setCartVal)) {
				// ���ִ������Ŀ���ɹ��ˣ����޸ĵ�ǰֵ
				memcpy(&rbD.curCartVal,&rbD.setCartVal, sizeof(CartisianTran));
			}else {// ���򵽴���λ���˻�Ŀ��ֵ���������˶�
				coordType = JOINT_TYPE;
				memcpy(&rbD.setCartVal,&rbD.curCartVal,sizeof(CartisianTran));
				Serial.println("+INFO:cartisian is in limit!");
			}// ִ�е�Ŀ���
			break;
		default:
			break;
		}
		break;
	case POINT_MODE:
		break;
	case TARGET_MODE:
		if (coordType == LINE_TYPE) {
			float posDiff = 0;  // Ŀ��λ�õ���ǰλ�õĲ�ֵ
			int inPlace = 0;    // �Ƿ�λ
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
				InverseKinematics(interpolationVal); // ִ�е�Ŀ���
			}
			else {
				coordType = JOINT_TYPE;
				Serial.println("+INFO:Line in place!");
			}
		}// ������һ����
		break;
	default:
		break;
	}

	// ��ֵ��ǰ�趨�ĽǶ�
	rbD.setJointVal.jVal[0] = base.setVal;
	rbD.setJointVal.jVal[1] = rArm.setVal;
	rbD.setJointVal.jVal[2] = fArm.setVal;
	rbD.setJointVal.jVal[3] = claw.setVal;        // ����Ŀ��ֵ
	rbD.state = Idle;                        // Ĭ���ǿ���

	for (int i = 0; i < 4; i++) {
		int inc = rbD.setJointVal.jVal[i] - rbD.curJointVal.jVal[i];
		if (inc != 0) {
			rbD.curJointVal.jVal[i] += inc / abs(inc);  // ������һ����λֵ
			rbD.state = Moving;  // ֻҪ��һ���仯�����Ƿǿ���
		}
	}
	// ֻ����Ҫ�˶�ʱ���Ż��·�ָ��
	if (rbD.state == Moving) {
		base.servo.writeMicroseconds(rbD.curJointVal.jVal[0]);
		rArm.servo.writeMicroseconds(rbD.curJointVal.jVal[1]);
		fArm.servo.writeMicroseconds(rbD.curJointVal.jVal[2]);
		claw.servo.writeMicroseconds(rbD.curJointVal.jVal[3]);
		delay(DSD); // ���������ٶ�
		//Serial.println(rbD.curVal.jVal[0]); // �������ݺ�ʱ������
	}
}

/*
�������ܣ�ָ�����
������ָ���� serialCmd
*/
void commandPaser(char serialCmd) {
	float serialData;
	float serialDataFloat;
	if (serialCmd == 'b' || serialCmd == 'r' ||
		serialCmd == 'f' || serialCmd == 'c') {
		coordType = JOINT_TYPE;
		serialData = Serial.parseFloat();   // ��ȡָ����Ϣ
		switch (mode)
		{
		case CONTIOUS_MODE:
			for (int i = 0; i < 4; i++)
				rbD.jointDir.jVal[i] = .0;
			setDir(serialCmd, (int)serialData);  // �趨�˶�����
			break;
		case POINT_MODE:
			switch (serialCmd)
			{
			case 'b':
				if (!setJointVal(base, base.setVal + (int)(serialData*10.31))) {
					Serial.println("+Warning:Base Servo Value Out Of Limit!");
				} // ִ��ָ����Ϣ
				break;
			case 'r':
				if (!setJointVal(rArm, rArm.setVal + (int)(serialData*10.31))) {
					Serial.println("+Warning:rArm Servo Value Out Of Limit!");
				} // ִ��ָ����Ϣ
				break;
			case 'f':
				if (!setJointVal(fArm, fArm.setVal + (int)(serialData*10.31))) {
					Serial.println("+Warning:fArm Servo Value Out Of Limit!");
				} // ִ��ָ����Ϣ
				break;
			case 'c':
				if (setJointVal(claw, claw.setVal + (int)(serialData*10.31))) {
					Serial.println("+Warning:claw Servo Value Out Of Limit!");
				} // ִ��ָ����Ϣ
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
			} // ִ��ָ����Ϣ
			break;
		default:
			break;
		}
		while (Serial.available()) { char wornStr = Serial.read(); };  // ��ջ�����
		return;
	}

	if (serialCmd == 'x' || serialCmd == 'y' || serialCmd == 'z') {
		coordType = CARTISIAN_TYPE;
		switch (mode)
		{
		case CONTIOUS_MODE:   // �൱���� x��y��z ,һֱ����ȥ��speedl��Ҫһֱ��ֵ
			serialData = Serial.parseInt();   // ��ȡָ����Ϣ
			for (int i = 0; i < 3; i++)
				rbD.cartDir.cVal[i] = .0;
			// ��Ҫ����һ�µ�ǰ��ʵʱֵ��֮���ֵ���ڴ˻���������
			// rbD.curCartVal ��ֵ
			forwardKinematics((45 - base.servo.read() / 2)* deg2rad, 
				(rArm.servo.read() - 90)* deg2rad, 
				(90 - fArm.servo.read())* deg2rad);
			setDir(serialCmd, serialData);    // �趨�˶�����
			break;
		case TARGET_MODE:  // �൱��Ptp������ռ��е�ĳһ����
			serialDataFloat = Serial.parseFloat();   // ��ȡָ����Ϣ
			//servoCmd(serialCmd, serialData); ??     // ִ��ָ����Ϣ
			break;
		default:
			break;
		}
		while (Serial.available()) { char wornStr = Serial.read(); };  // ��ջ�����
		return;
	}

	switch (serialCmd) {
	case 'm':  // ģʽѡ��
		for (int i = 0; i < 3; i++) {
			rbD.jointDir.jVal[i] = .0;
			rbD.cartDir.cVal[i] = .0;
		}
		rbD.jointDir.jVal[3] = .0;
		// �л�ģʽ��ʱ��һ��Ҫ���㣬�������л��������ܻ�����˶�
		serialData = Serial.parseFloat();   // ��ȡָ����Ϣ
		if ((int)serialData != NEXT_MODE) {
			setMode((int)serialData);
		}else {
			serialData = ((mode + 1) % (MODE_MAX - 1) + 1);
			setMode((int)serialData);
		}
		break;
	case 'v':  // �ٶ�����
		serialData = Serial.parseFloat();  // ��ȡ���ָ�����ٶ�ֵ
		Serial.println("");
		Serial.print("+Command:Set Velocity ");
		Serial.print(serialData);
		Serial.println("");
		if (serialData > 0) {
			DSD = (int)serialData;
		}
		break;
	case 'o':  // ��е����Ϣ���
		reportStatus();
		break;
	case 'j':  // �ؽڽ�ͬʱ�˶� joint��jb,r,f,c
		coordType = JOINT_TYPE;
		switch (mode)
		{
		case CONTIOUS_MODE:
			for (int i = 0; i < 4; i++)
				rbD.jointDir.jVal[i] = .0;
			setDir('b', Serial.parseInt());  // �趨�˶�����
			serialData = Serial.read();  // ��ȡ���ָ���е�������Ϣ
			setDir('r', Serial.parseInt());  // �趨�˶�����
			serialData = Serial.read();  // ��ȡ���ָ���е�������Ϣ
			setDir('f', Serial.parseInt());  // �趨�˶�����
			serialData = Serial.read();  // ��ȡ���ָ���е�������Ϣ
			setDir('c', Serial.parseInt());  // �趨�˶�����
			break;
		case POINT_MODE:
			setJointVal(base, base.setVal + Serial.parseInt());
			serialData = Serial.read();  // ��ȡ���ָ���е�������Ϣ
			setJointVal(rArm, rArm.setVal + Serial.parseInt());
			serialData = Serial.read();  // ��ȡ���ָ���е�������Ϣ
			setJointVal(fArm, fArm.setVal + Serial.parseInt());
			serialData = Serial.read();  // ��ȡ���ָ���е�������Ϣ
			setJointVal(claw, claw.setVal + Serial.parseInt());
			break;
		case TARGET_MODE:
			Serial.println("joint move");
			servoCmd('b', Serial.parseInt()); // ��ȡ���ָ�����ٶ�ֵ
			serialData = Serial.read();  // ��ȡ���ָ���е�������Ϣ
			servoCmd('r', Serial.parseInt()); // ��ȡ���ָ�����ٶ�ֵ
			serialData = Serial.read();  // ��ȡ���ָ���е�������Ϣ
			servoCmd('f', Serial.parseInt()); // ��ȡ���ָ�����ٶ�ֵ
			serialData = Serial.read();  // ��ȡ���ָ���е�������Ϣ
			servoCmd('c', Serial.parseInt()); // ��ȡ���ָ�����ٶ�ֵ
			break;
		default:
			break;
		}
		break;
	case 'p':  // ĩ��λ��ģʽ��Ptp��px,y,z,c
		coordType = CARTISIAN_TYPE;
		switch (mode)
		{
		case CONTIOUS_MODE:  // �൱�� speedl
			break;
		case POINT_MODE:     // �൱������ƶ�һ��
			break;
		case TARGET_MODE:    // �൱��Ptp
			// ����λ��ֵ
			for (int i = 0; i < 3; i++) {
				rbD.setCartVal.cVal[i] = Serial.parseFloat();
				serialData = Serial.read();
			}
			// ����Ƕ�ֵ���
			InverseKinematics(rbD.setCartVal); // ִ�е�Ŀ���
			break;
		default:
			break;
		}
		break;
	case 'l': // ��ֱ���ߵ��ռ�ĳ�� line:lx,y,z
		for (int i = 0; i < 3; i++) {
			rbD.setCartVal.cVal[i] = Serial.parseFloat();
			serialData = Serial.read();
		}
		coordType = LINE_TYPE;
		break;
	default:  // ������ģʽ������ִ��
		if (serialCmd == ' ') {
			while (Serial.available()) { char wornStr = Serial.read(); };  // ��ջ�����
			return;
		}
		Serial.print("Unknow Command");
		Serial.print(serialCmd);
		Serial.println("");
		break;
	}
	while (Serial.available()) { char wornStr = Serial.read(); };  // ��ջ�����
}

void setMode(int serialData) {
	switch (serialData) {
	case CONTIOUS_MODE:  // contious �����˶�ģʽ
		mode = CONTIOUS_MODE;
		Serial.println("+Command:Set Move Mode: CONTIOUS_MODE");
		break;
	case POINT_MODE:  // �㶯ģʽ
		mode = POINT_MODE;
		Serial.println("+Command:Set Move Mode: POINT_MODE");
		break;
	case TARGET_MODE:  // target Ŀ���ģʽ
		mode = TARGET_MODE;
		Serial.println("+Command:Set Move Mode: TARGET_MODE");
		break;
	default:
		Serial.print("Unknow Mode");
		break;
	}
}

/*��е������*/
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

/*�������ܣ���⵱ǰ�趨λ�õ����*/
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

	// �趨�Ƕ�
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
		// �ܵ����Ϊ��ǰֵ��ֵ
		rbD.curCartVal.cVal[0] = cart.cVal[0];
		rbD.curCartVal.cVal[1] = cart.cVal[1];
		rbD.curCartVal.cVal[2] = cart.cVal[2];
	}
	return true;
}

/*
�������ܣ������ŷ�ʱ����ӡ����Ϣ
�������ŷ�������������ת�Ƕ�
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
�������ܣ��趨ָ���ؽڵ���һ���Ƕ�
�������ؽ������ؽڽ� 544-2400
*/
bool setJointVal(RobotJoint & joint, const int servoData) {
	if (servoData >= *joint.limitMin && servoData <= *joint.limitMax)
		joint.setVal = servoData;
	else
		return false;
	return true;
}

/*
�������ܣ�ִ��ָ���ؽڵ�����˶�
������
�ؽ����ƣ�b,r,f,c
�ؽڽǶȣ�servoData
*/
bool servoCmd(char servoName, float servoData) {
	switch (servoName)
	{
	case 'b':   // ���ָ�� 'b' ����base����Ƕ�
		servoData = (int)(-servoData * 20.6222222 + 1472 + 0.5);  // ת��ΪĿ��ֵ��+0.5��Ϊ ��������
		return setJointVal(base, servoData);
	case 'r': // ���ָ�� 'r' ����rArm����Ƕ�
		servoData = (int)(servoData * 10.31111111 + 1472 + 0.5); // ת��ΪĿ��ֵ
		return setJointVal(rArm, servoData);
	case 'f':// ���ָ�� 'f' ����fArm����Ƕ�
		servoData = (int)(1472 - servoData * 10.3111111 + 0.5); // ת��ΪĿ��ֵ 
		return setJointVal(fArm, servoData);
	case 'c':// ���ָ�� 'c' ����claw����Ƕ�
		servoData = (int)(servoData * 10.3111111 + 544 + 0.5); // ת��ΪĿ��ֵ
		return setJointVal(claw, servoData);
	}
}

/*
�������ܣ��趨ĳ������˶�����
����:�������������ֵ

*/
void setDir(char servoName, int servoData) {
	switch (servoName)
	{
	case 'b':
		rbD.jointDir.jVal[0] = servoData;
		break;
	case 'r': // ���ָ�� 'r' ����rArm����Ƕ�
		rbD.jointDir.jVal[1] = servoData;
		break;
	case 'f':// ���ָ�� 'f' ����fArm����Ƕ�
		rbD.jointDir.jVal[2] = servoData;
		break;
	case 'c':// ���ָ�� 'c' ����claw����Ƕ�
		rbD.jointDir.jVal[3] = servoData;
		break;
	case 'x': // x ������ٶ�
		rbD.cartDir.cVal[0] = servoData;
		break;
	case 'y': // y ������ٶ�
		rbD.cartDir.cVal[1] = servoData;
		break;
	case 'z': // z ������ٶ�
		rbD.cartDir.cVal[2] = servoData;
		break;
	default:
		break;
	}
}

/*
��ӡ��ǰ��е����Ϣ
*/
void reportStatus() {  // ��е��״̬��Ϣ�鿴
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
