//#include "StdAfx.h"
#include "URRobot.h"


#define  PI 3.1415926
CURRobot::CURRobot(void)
{
}


CURRobot::~CURRobot(void)
{

}

void CURRobot::SetServerIPAndPort(tstring IP, int Port)
{
	this->m_strTarget_IP = IP;
	m_iControl_Port = Port;
}
void CURRobot::SpeedChange(double _dbSpeed)
{
	m_dbSpeedRate = m_dbBasicSpeed + _dbSpeed;
}
void CURRobot::AccelerationChange(double _dbAccleration)
{
	m_dbAccelerationRate = m_dbBasicAcceleration + _dbAccleration;
}



int CURRobot::InitRobot(std::wstring RobotIP)
{
	//InitializeCriticalSection(&cs);
	//bCloseFlag=FALSE;
	m_pModbus   = new CModbusSocket();
	if (m_pModbus->InitModbusSocket(RobotIP,502,1000) < 0)
	{
		return -1;
	}
#ifdef ROBOT_TEST

#else
#endif
	m_pDashBoard = new CDashBoard();
	if (m_pDashBoard->InitDashBoard(RobotIP,29999,1000) < 0)
	{
		return -1;
	}
	//AfxWriteToLog("����Dash�ɹ���\n");
	//��һЩ�������ݳ�ʼ��
	m_iControl_Port = 30002;
	//m_strTarget_IP = UR_IP;
	m_iDataRefreshRate = 100;
	//����ֻʹ��Ĭ��ֵ
	m_dbBasicSpeed = BASICSPEED;
	m_dbBasicAcceleration = BASICACCELERATION;
	m_dbSpeedRate = m_dbBasicSpeed * 10 /10.0;
	m_dbAccelerationRate = m_dbBasicAcceleration * 10 /10.0;
	m_iWaitCMDBack = WAITCMDBACK;

	//
	m_iMoveMode = 1;
	return 1;
}
int CURRobot::CloseRobot()
{
	//EnterCriticalSection(&cs);	
	//LeaveCriticalSection(&cs);
//	m_listPosRec.clear();
//	DeleteCriticalSection(&cs);
	delete m_pModbus;	
	delete m_pDashBoard;
	return 1;
}


int CURRobot::GetRobotInformation()
{
	if (GetPositions() > 0 && GetAngles() > 0 && GetRobotState() > 0)
	{		
		return 1;			
	}
	return -1;
}

//ȡ����������ֵ�ķ���
int CURRobot::GetPositions()
{
	//����ʵʱ��ȡUR���ݣ��ȶ�ȡ��������ֵ���ֲ�������ַ����400��
	int SixPositions[6] = {0};
	int m_iNum = 0;
	//AfxWriteToLog("��ȡModubusֵ\n");
	if((m_iNum = m_pModbus->ReadMultipleRegister(6, 400,SixPositions)) < 0)
	{
		//AfxWriteToLog("������ȡֵ��[%d]\n", m_iNum);
		return -1;
	}
	//AfxWriteToLog("��ȡModubusֵ���\n");
	// 
	// 	//����XYZ��ȡ�õĲ�����ֱ��ʹ�ã�����Ҫ����ת��
	double PositionsFiltered[6] = {0.0};

	for (int i = 0; i < 6; i++)
	{
		//��Modbus UR�ο��ĵ������XYZ�г���32768�����ɸ�ֵ����Ȼ��0-65535����������Ҫ�Լ�����з�������-32767������32768����
		if (SixPositions[i] > 32768)
		{
			SixPositions[i] = SixPositions[i] - 65535;
		}

		//ǰ������XYZ��ͬ������(�������е�XYZ��Ҫ����10)//
		//��������UVWֻҪֱ�ӳ���1000����
		if (i < 3)
		{
			PositionsFiltered[i] = SixPositions[i]/10.0;
		}
		else 
		{
			PositionsFiltered[i] = SixPositions[i]/1000.0;
		}
	}	

	// 	for (int i=0;i<6;i++)
	// 	{
	// 		PositionsFiltered[i] = double((g_kScanner.iter+i)/0.1);
	// 	}
	// 	g_kScanner.iter++;
	//����֮��͸�д�Լ��ĳ�Աֵ(ע��XYZֵ����Ҫԭ֭ԭζ�ĸ��������������)

	m_dbarryPos[0] =  (PositionsFiltered[0]);
	m_dbarryPos[1] =  (PositionsFiltered[1]);
	m_dbarryPos[2] =  (PositionsFiltered[2]);

	m_dbarryPos[3] =  (PositionsFiltered[3]);
	m_dbarryPos[4] =  (PositionsFiltered[4]);
	m_dbarryPos[5] =  (PositionsFiltered[5]);

	//�����m_pos���г�ʼ��
	memcpy(m_sttPos.dbPos, m_dbarryPos, sizeof(m_dbarryPos));
	m_sttPos.dbAccelerationRate = this->m_dbAccelerationRate;	//m/s
	m_sttPos.dbSpeedRate = this->m_dbSpeedRate;		//m
	//m_sttPos.posAttr = _posAttr::processPos;
	return 1;
}

//ȡ�������Ƕ�ֵ�ķ���
int CURRobot::GetAngles()
{
	//����ʵʱ��ȡUR���ݣ���ȡ�����ؽڽǶ�ֵ���ֲ�������ַ����270��
	int IsSixAngles[6] = {0};
	int SixAngles[6] ={0};
	int i_errorno = 0;

	if((i_errorno = m_pModbus->ReadMultipleRegister(6, 320, IsSixAngles)) < 0)
	{
		return i_errorno;
	}

	if((i_errorno = m_pModbus->ReadMultipleRegister(6, 270, SixAngles)) < 0)
	{
		return i_errorno;
	}
	//���������Ƕ�ֵ��ֻ��double���Ϳ��Է�����˫��������ֵ
	double AnglesFiltered[6] = {0.0};

	for (int i = 0; i < 6; i++)
	{
		//����ȡ���Ķ��ǻ���ֵ��Ҫת��Ϊ�Ƕ�ֵ(����1000֮���ǻ��ȣ�Ҫ��ת��Ϊ�Ƕ�)
		AnglesFiltered[i] = SixAngles[i]/1000.0*(180/PI);

		if (IsSixAngles[i] > 0)
		{
			AnglesFiltered[i] = AnglesFiltered[i]-360.0;
		}		
	}

	//����֮��͸�д�Լ��ĳ�Աֵ(ת��Ϊ����ֵ)
	m_dbarryAng[0] =  (AnglesFiltered[0]);
	m_dbarryAng[1] =  (AnglesFiltered[1]);
	m_dbarryAng[2] =  (AnglesFiltered[2]);
	m_dbarryAng[3] =  (AnglesFiltered[3]);
	m_dbarryAng[4] =  (AnglesFiltered[4]);
	m_dbarryAng[5] =  (AnglesFiltered[5]);

	memcpy(m_sttPos.dbAng, m_dbarryAng, sizeof(m_dbarryAng));
	// 	m_sttPos.dbAccelerationRate = this->m_dbAccelerationRate;	//m/s
	// 	m_sttPos.dbSpeedRate = this->m_dbSpeedRate;		//m
	// 	m_sttPos.posAttr = _posAttr::processPos;

	return 1;
}

//��ȡ����ִ��״̬����
int CURRobot::Read_commandBack()
{
	int cmdState[2] = {0};
	//Sleep(m_iWaitCMDBack);	//�ȴ�1s
	if (m_pModbus->ReadMultipleRegister(1, 145, cmdState) < 0)
	{
		return -1;
	}
	else
	{
		return cmdState[0];
	}
}



//���ֲ����֪����260-265�ļĴ��������˵�ǰ��е�ֵ�״̬��Ϣ
int CURRobot::GetRobotState()
{
	//����ʵʱ��ȡUR���ݣ���ȡ�Ƿ񿪻����Ƿ����ͣ����״̬��Ϣ���ֲ�������ַ����260��
	int i_errorno = 0;
	if((i_errorno = m_pModbus->ReadMultipleRegister(6, 260,m_iarryRobotState)) < 0)
	{
		return -1;
	}
	return 1;
}



//�ƶ���Ҫ����
int CURRobot::MoveToPos(const UrRecPosAng& pos)
{
	return MoveL(const_cast<UrRecPosAng&>(pos),L"",0);
}


void CURRobot::Return2Ori()
{
	return ;
}


//�������λ����Ϣת��Ϊshort����
unsigned short * CURRobot::ToShortArry(const unsigned short usCmd, unsigned short * const data)
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

int CURRobot::ConnectUR(void)
{
	return m_pModbus->PingToSever();

}
// ����ʾ��ģʽ
int CURRobot::ChangeModeToTech(void)
{
	unsigned short data[2];
	if(Send_command(2,ToShortArry(2,data)) < 0)
	{
		return -1;
	}
	return 1;
}
// ����ʾ��ģʽ
int CURRobot::EndTeachMode(void)
{
	unsigned short data[2];
	if(Send_command(3, ToShortArry(3,data)) < 0)
	{
		return -1;
	}
	return 1;
}

// ��ͣ������
int CURRobot::PauseUR(void)
{
	unsigned short data[2] = {0};
	if(Send_command(4, ToShortArry(4,data)) < 0)
	{
		return -1;
	}
	return 1;
}
// �رջ�����
int CURRobot::StopUR(void)
{
	unsigned short data[2] = {0};
	if(Send_command(5, ToShortArry(5,data)) < 0)
	{
		return -1;
	}
	return  1;
}






//ֱ���ƶ�
int CURRobot::MoveL(UrRecPosAng& UrPos, const tstring& axis, const int idir)
{
	unsigned short data[17];
	return Send_command(1,ToShortArry(1, UrPos, data));
}

int CURRobot::Send_command(const unsigned char cmd, const unsigned short * const data)
{
	switch (cmd)
	{
	case 1://�ƶ���ĳһλ�� movel
		return m_pModbus->WriteMultipleRegister(17, 128, data);
	case 2:	//ʾ��ģʽ�����Խ��е��ڻ�������̬
		return m_pModbus->WriteMultipleRegister(2, 128, data);
	case 3://ֹͣʾ��ģʽ
		return m_pModbus->WriteMultipleRegister(2, 128, data);
	case 4://��ͣ
		return m_pModbus->WriteMultipleRegister(2, 128, data);
	case 5://ֹͣ�ػ�
		return m_pModbus->WriteMultipleRegister(2, 128, data);
	default:
		//ADDMSG(L"δ����ָ�");
		return -1;
	}
}

unsigned short * CURRobot::ToShortArry(const unsigned short usCmd, const UrRecPosAng& UrPos,  unsigned short * const data)
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
	data[1] = usCmd;
	data[0] = m_uchrCmdNum;
	for (int i = 0; i < 6; i++)
	{
		if (UrPos.dbPos[i] < 0)
		{
			if (i < 3)
			{
				data[i + 2] = (unsigned short)(UrPos.dbPos[i] * 10 + 65536);
			}
			else
			{
				data[i + 2] = (unsigned short)(UrPos.dbPos[i] * 1000 + 65536);
			}
		}
		else
		{
			if (i < 3)
			{
				data[i + 2] = (unsigned short)(UrPos.dbPos[i] * 10);
			}
			else
			{
				data[i + 2] = (unsigned short)(UrPos.dbPos[i] * 1000);
			}
		}
	}
	data[8] = unsigned short(m_dbSpeedRate        * 1000);
	data[9] = unsigned short(m_dbAccelerationRate * 1000); 

	for (int i=0; i< 6; i++)
	{
		if (UrPos.dbAng[i] < 0)
		{
			data[i + 10] = (unsigned short)(UrPos.dbAng[i] * 1000 / (180/PI) + 65536);
		}
		else
		{
			data[i + 10] = (unsigned short)(UrPos.dbAng[i] * 1000 / (180/PI) );
		}
	}

	//2017.01.16 �����ǶȻ���λ�� 0 ����λ�ã� 1����Ƕ�
	data[16] = m_iMoveMode;

	return data;
}
void CURRobot::ChangeMoveMode(int iMode)
{
	m_iMoveMode = iMode;
}

int CURRobot::GetCurPositions(UrRecPosAng& pPosAng, enum _posAttr posAttr)
{
	//if (GetPositions() < 0)
	//{
	//	return -1;
	//}
	//else
	//{
		memcpy(pPosAng.dbPos, m_sttPos.dbPos, sizeof(double) * 6);
		pPosAng.posAttr = posAttr;
		return 1;
	//}	
}
int CURRobot::GetCurAngles(UrRecPosAng& pPosAng, enum _posAttr posAttr)
{
	//if (GetAngles() < 0)
	//{
	//	return -1;
	//}
	//else
	//{
		memcpy(pPosAng.dbAng, m_sttPos.dbAng, sizeof(double) * 6);
		pPosAng.posAttr = posAttr;
		return 1;
	//}
}
int CURRobot::GetCurRobotState(URState &pState)
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



bool CURRobot::LoadAndOpenProgram()
{
	char *pisload = "get loaded program\n";
	if ((m_pDashBoard->WriteDashBoard(pisload)) != 1)
	{
		Sleep(300);
		char *pload = "load /programs/movel.urp\n";
		if ((m_pDashBoard->WriteDashBoard(pload)) != 1)
		{
			//AfxWriteToLog("δ���س���!\n");
			return false;
		}
	}
	Sleep(300);
	//AfxWriteToLog("׼������!\n");
	if ((m_pDashBoard->ReadDashBoard()) != 1)
	{
		Sleep(300);
		char *pplay = "play\n";
		if ((m_pDashBoard->WriteDashBoard(pplay)) != 1)
		{
			//AfxWriteToLog("δ���г���!\n");
			return false;
		}
	}
	return true;
}

void CURRobot::InitCmdNum()
{
	m_uchrCmdNum = 0;
}