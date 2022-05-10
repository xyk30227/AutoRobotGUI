//#include "StdAfx.h"
#include "FRANCRobot.h"

#define  PI 3.1415926
typedef struct _FPrivateRegs
{
	uint16_t cmd;
	uint16_t cmdhead;
	uint32_t ang[6];
	uint16_t acc;
	uint16_t speed;
	uint32_t pos[6];
	uint16_t postype; //0 ������ά����  1����ؽ�����
	uint16_t regsnum;

	_FPrivateRegs()
	{
		cmd = 1;
		cmdhead = 2;
		ang[0]=ang[1]=ang[2]=ang[3]=ang[4]=ang[5]=0;
		acc = uint16_t(0.5 * 1000);
		speed = 1 * 1000;
		pos[0]=pos[1]=pos[2]=pos[3]=pos[4]=pos[5]=0;
		postype = 0; 
		regsnum = 29;
	}
}FPrivateRegs, *pFPrivateRegs;




CFRANCRobot::CFRANCRobot(void)
{
}


CFRANCRobot::~CFRANCRobot(void)
{
}

void CFRANCRobot::SetServerIPAndPort(tstring IP, int Port)
{
	this->m_strTarget_IP = IP;
	m_iControl_Port = Port;
}
void CFRANCRobot::SpeedChange(double _dbSpeed)
{
	m_dbSpeedRate = m_dbBasicSpeed + _dbSpeed;
}
void CFRANCRobot::AccelerationChange(double _dbAccleration)
{
	m_dbAccelerationRate = m_dbBasicAcceleration + _dbAccleration;
}



int CFRANCRobot::InitRobot(std::wstring RobotIP)
{
//	InitializeCriticalSection(&cs);
	//bCloseFlag=FALSE;
	m_pModbus = new CModbusSocket();
	if (m_pModbus->InitModbusSocket(RobotIP, 502, 1000) < 0)
	{
		return -1;
	}
#ifdef ROBOT_TEST

#else
#endif
	//��һЩ�������ݳ�ʼ��
	m_iControl_Port = 30002;
	//m_strTarget_IP = UR_IP;
	m_iDataRefreshRate = 100;
	//����ֻʹ��Ĭ��ֵ
	m_dbBasicSpeed = BASICSPEED;
	m_dbBasicAcceleration = BASICACCELERATION;
	m_dbSpeedRate = m_dbBasicSpeed * 10 / 10.0;
	m_dbAccelerationRate = m_dbBasicAcceleration * 10 / 10.0;
	m_iWaitCMDBack = WAITCMDBACK;

	//
	m_iMoveMode = 1;
	return 1;
}
int CFRANCRobot::ConnectUR(void)
{
	return m_pModbus->PingToSever();

}
int CFRANCRobot::CloseRobot()
{
//	EnterCriticalSection(&cs);

//	LeaveCriticalSection(&cs);
//	m_listPosRec.clear();
//	DeleteCriticalSection(&cs);
	delete m_pModbus;
//	delete m_pDashBoard;
	return 1;
}


int CFRANCRobot::GetRobotInformation()
{
	if (GetPositions() > 0 && GetAngles() > 0)
	{
		return 1;
	}
	return -1;
}

//ȡ����������ֵ�ķ���
int CFRANCRobot::GetPositions()
{
	//����ʵʱ��ȡUR���ݣ��ȶ�ȡ��������ֵ���ֲ�������ַ����400��
	int SixPositions[12] = { 0 };
	int m_iNum = 0;
	//AfxWriteToLog("��ȡModubusֵ\n");

	//xyk 2018.5.23
	if ((m_iNum = m_pModbus->ReadMultipleRegister(12, 12, SixPositions)) < 0)
	//if ((m_iNum = m_pModbus->ReadMultipleRegister(12, 0, SixPositions)) < 0)
	{
		//AfxWriteToLog("������ȡֵ��[%d]\n", m_iNum);
		return -1;
	}
	//AfxWriteToLog("��ȡModubusֵ���\n");
	// 
	// 	//����XYZ��ȡ�õĲ�����ֱ��ʹ�ã�����Ҫ����ת��
	double PositionsFiltered[6] = { 0.0 };

	for (int i = 0; i < 6; i++)
	{
		//��Modbus UR�ο��ĵ������XYZ�г���32768�����ɸ�ֵ����Ȼ��0-65535����������Ҫ�Լ�����з�������-32767������32768����
		SixPositions[i] = (SixPositions[i * 2 + 1] << 16) + SixPositions[i * 2];
		if (SixPositions[i] > 2147483648)
		{
			SixPositions[i] = SixPositions[i] - 4294967296;
		}

		//ǰ������XYZ��ͬ������(�������е�XYZ��Ҫ����10)//
		//��������UVWֻҪֱ�ӳ���1000����
		if (i < 3)
		{
			PositionsFiltered[i] = SixPositions[i] / 1000.0;
		}
		else
		{
			PositionsFiltered[i] = SixPositions[i] / 1000.0;
		}
	}
	//����֮��͸�д�Լ��ĳ�Աֵ(ע��XYZֵ����Ҫԭ֭ԭζ�ĸ��������������)

	m_dbarryPos[0] = (PositionsFiltered[0]);
	m_dbarryPos[1] = (PositionsFiltered[1]);
	m_dbarryPos[2] = (PositionsFiltered[2]);

	m_dbarryPos[3] = (PositionsFiltered[3]);
	m_dbarryPos[4] = (PositionsFiltered[4]);
	m_dbarryPos[5] = (PositionsFiltered[5]);

	//�����m_pos���г�ʼ��
	memcpy(m_sttPos.dbPos, m_dbarryPos, sizeof(m_dbarryPos));
	m_sttPos.dbAccelerationRate = this->m_dbAccelerationRate;	//m/s
	m_sttPos.dbSpeedRate = this->m_dbSpeedRate;		//m
	//m_sttPos.posAttr = _posAttr::processPos;
	return 1;
}

//ȡ�������Ƕ�ֵ�ķ���
int CFRANCRobot::GetAngles()
{
	//����ʵʱ��ȡUR���ݣ���ȡ�����ؽڽǶ�ֵ���ֲ�������ַ����270��
//	int IsSixAngles[6] = { 0 };
	int SixAngles[12] = { 0 };
	int i_errorno = 0;

	//xyk 2018.5.23
	//if ((i_errorno = m_pModbus->ReadMultipleRegister(12, 270, SixAngles)) < 0)
	if ((i_errorno = m_pModbus->ReadMultipleRegister(12, 0, SixAngles)) < 0)
	{
		return i_errorno;
	}
	//���������Ƕ�ֵ��ֻ��double���Ϳ��Է�����˫��������ֵ
	double AnglesFiltered[6] = { 0.0 };

	for (int i = 0; i < 6; i++)
	{
		//����ȡ���Ķ��ǻ���ֵ��Ҫת��Ϊ�Ƕ�ֵ(����1000֮���ǻ��ȣ�Ҫ��ת��Ϊ�Ƕ�)
		SixAngles[i] =  (SixAngles[i * 2 + 1] << 16) + SixAngles[i * 2];
		if (SixAngles[i] > 2147483648)
		{
			SixAngles[i] = SixAngles[i] - 4294967296;
		}
		AnglesFiltered[i] = SixAngles[i] / 1000.0;//* (180 / PI);
	}

	//����֮��͸�д�Լ��ĳ�Աֵ(ת��Ϊ����ֵ)
	m_dbarryAng[0] = (AnglesFiltered[0]);
	m_dbarryAng[1] = (AnglesFiltered[1]);
	m_dbarryAng[2] = (AnglesFiltered[2]);
	m_dbarryAng[3] = (AnglesFiltered[3]);
	m_dbarryAng[4] = (AnglesFiltered[4]);
	m_dbarryAng[5] = (AnglesFiltered[5]);

	memcpy(m_sttPos.dbAng, m_dbarryAng, sizeof(m_dbarryAng));
	return 1;
}

//��ȡ����ִ��״̬����
int CFRANCRobot::Read_commandBack()
{
#ifdef ROBOT_TEST
	Sleep(2500);
	return m_uchrCmdNum;
#else
	int cmdState[2] = { 0 };
	//Sleep(m_iWaitCMDBack);	//�ȴ�1s
	m_pModbus->ReadMultipleRegister(1, 50, cmdState);
	return cmdState[0];
#endif 
}



//���ֲ����֪����260-265�ļĴ��������˵�ǰ��е�ֵ�״̬��Ϣ
int CFRANCRobot::GetRobotState()
{
	//����ʵʱ��ȡUR���ݣ���ȡ�Ƿ񿪻����Ƿ����ͣ����״̬��Ϣ���ֲ�������ַ����260��
	int i_errorno = 0;
	if ((i_errorno = m_pModbus->ReadMultipleRegister(6, 260, m_iarryRobotState)) < 0)
	{
		return i_errorno;
	}
	return 1;
}

//�ƶ���Ҫ����
int CFRANCRobot::MoveToPos(const UrRecPosAng& pos)
{
	return MoveL(const_cast<UrRecPosAng&>(pos), L"", 0);
}


void CFRANCRobot::Return2Ori()
{
	return;
}





//�������λ����Ϣת��Ϊshort����
unsigned short * CFRANCRobot::ToShortArry(const unsigned short usCmd, unsigned short * const data)
{
	if (data == NULL)
	{
		return NULL;
	}
	m_uchrCmdNum++;
	if (m_uchrCmdNum >= 30000 || m_uchrCmdNum <= 1)
	{
		m_uchrCmdNum = 2;
	}
	switch (usCmd)
	{
	case 2:
	case 3:
	case 4:
	case 5:
		data[1] = usCmd;
		data[0] = m_uchrCmdNum;
		break;
	default:
		break;
	}
	return data;
}
// ����ʾ��ģʽ
int CFRANCRobot::ChangeModeToTech(void)
{
	return 1;
}
// ����ʾ��ģʽ
int CFRANCRobot::EndTeachMode(void)
{
	return 1;
}
// ��ͣ������
int CFRANCRobot::PauseUR(void)
{
	return 1;
}
// �رջ�����
int CFRANCRobot::StopUR(void)
{
	return  1;
}

//ֱ���ƶ�
int CFRANCRobot::MoveL(UrRecPosAng& UrPos, const tstring& axis, const int idir)
{
	unsigned short data[30];
	return Send_command(1, ToShortArry(1, UrPos, data));
}

int CFRANCRobot::Send_command(const unsigned char cmd, const unsigned short * const data)
{
	switch (cmd)
	{
	case 1://�ƶ���ĳһλ�� movel
		return m_pModbus->WriteMultipleRegister(29, 58, data);
	case 2:	//ʾ��ģʽ�����Խ��е��ڻ�������̬
		return m_pModbus->WriteMultipleRegister(2, 58, data);
	case 3://ֹͣʾ��ģʽ
		return m_pModbus->WriteMultipleRegister(2, 58, data);
	case 4://��ͣ
		return m_pModbus->WriteMultipleRegister(2, 58, data);
	case 5://ֹͣ�ػ�
		return m_pModbus->WriteMultipleRegister(2, 58, data);
	default:
		//ADDMSG(L"δ����ָ�");
		return -1;
	}
}

unsigned short * CFRANCRobot::ToShortArry(const unsigned short usCmd, const UrRecPosAng& UrPos, unsigned short * const data)
{
	//if (data == NULL)
	//{
	//	return NULL;
	//}
	//m_uchrCmdNum++;
	//if (m_uchrCmdNum >= 30000 || m_uchrCmdNum <= 1)
	//{
	//	m_uchrCmdNum = 2;
	//}
	//data[1] = usCmd;
	//data[0] = m_uchrCmdNum;
	//for (int i = 0; i < 6; i++)
	//{
	//	if (UrPos.dbPos[i] < 0)
	//	{
	//		data[i*2 + 2] = (unsigned short)((unsigned int)(UrPos.dbPos[i] * 1000 + 4294967296) % 65536);
	//		data[i*2 + 3] = (unsigned short)((unsigned int)(UrPos.dbPos[i] * 1000 + 4294967296) / 65536);
	//	}
	//	else
	//	{
	//		data[i * 2 + 2] = (unsigned short)((unsigned int)(UrPos.dbPos[i] * 1000 ) % 65536);
	//		data[i * 2 + 3] = (unsigned short)((unsigned int)(UrPos.dbPos[i] * 1000 ) / 65536);
	//	}
	//}
	////data[8] = m_dbSpeedRate * 1000;
	////data[9] = m_dbAccelerationRate * 1000;

	////for (int i = 0; i< 6; i++)
	////{
	////	if (UrPos.dbAng[i] < 0)
	////	{
	////		data[i + 10] = (unsigned short)(UrPos.dbAng[i] * 1000 / (180 / PI) + 65536);
	////	}
	////	else
	////	{
	////		data[i + 10] = (unsigned short)(UrPos.dbAng[i] * 1000 / (180 / PI));
	////	}
	////}

	////2017.01.16 �����ǶȻ���λ�� 0 ����λ�ã� 1����Ƕ�
	////data[16] = m_iMoveMode;



// 	if (data == NULL)
// 	{
// 		return NULL;
// 	}
	m_uchrCmdNum++;
	if (m_uchrCmdNum >= 30000 || m_uchrCmdNum <= 1)
	{
		m_uchrCmdNum = 2;
	}
	FPrivateRegs regs;
	for (int i = 0; i < 6;i++)
	{
		regs.ang[i] = 0;
		regs.pos[i] = 0;
	}
	if (UrPos.iMoveType == 1)
	{
		regs.cmd = UrPos.iMoveTime;
		regs.cmdhead = m_uchrCmdNum;
		regs.postype = 0;
		regs.acc = 0.5;
		regs.speed = 1;
		int imulx = 1;
		int addx = 0;
		


		for (int i = 0; i < 6; i++)
		{
			double dAng = UrPos.dbAng[i];//* 360 / 2 / PI;
			imulx = 1000;
			if (UrPos.dbAng[i] < 0)
			{
				addx = 4294967296;
			}
			else
			{
				addx = 0;
			}
			regs.ang[i] = (uint32_t)(dAng * imulx + addx);
		}		
	}
	else
	{
		regs.cmd = UrPos.iMoveTime;
		regs.cmdhead = m_uchrCmdNum;
		regs.postype = 1;
		regs.acc = 0.5 * 1000;
		regs.speed = 1 * 1000;
		int imulx = 1;
		int addx = 0;


		for (int i = 0; i < 6; i++)
		{	
			double pos = 0;
			pos = UrPos.dbPos[i];
			imulx = 1000;
			if (UrPos.dbPos[i] < 0)
			{
				addx = 4294967296;
			}
			else
			{
				addx = 0;
			}
			regs.pos[i] = (uint32_t)(pos * imulx + addx);
		}
	}
	memcpy(data, (uint16_t*)&regs, sizeof(uint16_t) * 30);
	return data;
	//return (uint16_t*)&regs;
}
void CFRANCRobot::ChangeMoveMode(int iMode)
{
	m_iMoveMode = iMode;
}

int CFRANCRobot::GetCurPositions(UrRecPosAng& pPosAng, enum _posAttr posAttr)
{
	//GetPositions();
	memcpy(pPosAng.dbPos, m_sttPos.dbPos, sizeof(double) * 6);
	pPosAng.posAttr = posAttr;
	return 1;
}
int CFRANCRobot::GetCurAngles(UrRecPosAng& pPosAng, enum _posAttr posAttr)
{
	//GetAngles();
	memcpy(pPosAng.dbAng, m_sttPos.dbAng, sizeof(double) * 6);
	pPosAng.posAttr = posAttr;
	return 1;
}
int CFRANCRobot::GetCurRobotState(URState &pState)
{
	//if (GetRobotState() < 0)
	//{
	//	return -1;
	//}
	//else
	//{
	pState = *(URState*)m_iarryRobotState;
	//memcpy(pState, m_iarryRobotState, sizeof(uint16_t) * 6);
	return 1;
	//}
}
bool CFRANCRobot::LoadAndOpenProgram()
{
	GetPositions();
	UrRecPosAng pOriPos, pMovePos;
	memcpy(pOriPos.dbPos, m_dbarryPos, sizeof(double) * 6);
	memcpy(pMovePos.dbPos, m_dbarryPos, sizeof(double) * 6);

	pMovePos.dbPos[2] = pMovePos.dbPos[2] + 3.0;

	//Z���ƶ�3
	MoveToPos(pMovePos);
	Sleep(3000);
	
	GetPositions();
	if (m_dbarryPos[2] - pOriPos.dbPos[2] > 1.000)
	{
		return true;
	}
	else
	{
		return false;
	}

}

void CFRANCRobot::InitCmdNum()
{
	m_uchrCmdNum = 0;
}