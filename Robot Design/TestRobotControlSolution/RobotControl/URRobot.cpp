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
	//AfxWriteToLog("连接Dash成功！\n");
	//将一些常用数据初始化
	m_iControl_Port = 30002;
	//m_strTarget_IP = UR_IP;
	m_iDataRefreshRate = 100;
	//这里只使用默认值
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

//取得六个坐标值的方法
int CURRobot::GetPositions()
{
	//这里实时读取UR数据（先读取六个坐标值，手册来看地址就是400）
	int SixPositions[6] = {0};
	int m_iNum = 0;
	//AfxWriteToLog("读取Modubus值\n");
	if((m_iNum = m_pModbus->ReadMultipleRegister(6, 400,SixPositions)) < 0)
	{
		//AfxWriteToLog("机器人取值，[%d]\n", m_iNum);
		return -1;
	}
	//AfxWriteToLog("读取Modubus值完成\n");
	// 
	// 	//对于XYZ，取得的并不能直接使用，而是要经过转换
	double PositionsFiltered[6] = {0.0};

	for (int i = 0; i < 6; i++)
	{
		//见Modbus UR参考文档：如果XYZ有超过32768，则变成负值（虽然是0-65535，但是我们要自己变成有符号数“-32767”到“32768”）
		if (SixPositions[i] > 32768)
		{
			SixPositions[i] = SixPositions[i] - 65535;
		}

		//前面三个XYZ做同样处理(对于所有的XYZ都要除以10)//
		//后面三个UVW只要直接除以1000即可
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
	//读到之后就改写自己的成员值(注意XYZ值还是要原汁原味的浮点数，方便计算)

	m_dbarryPos[0] =  (PositionsFiltered[0]);
	m_dbarryPos[1] =  (PositionsFiltered[1]);
	m_dbarryPos[2] =  (PositionsFiltered[2]);

	m_dbarryPos[3] =  (PositionsFiltered[3]);
	m_dbarryPos[4] =  (PositionsFiltered[4]);
	m_dbarryPos[5] =  (PositionsFiltered[5]);

	//这里对m_pos进行初始化
	memcpy(m_sttPos.dbPos, m_dbarryPos, sizeof(m_dbarryPos));
	m_sttPos.dbAccelerationRate = this->m_dbAccelerationRate;	//m/s
	m_sttPos.dbSpeedRate = this->m_dbSpeedRate;		//m
	//m_sttPos.posAttr = _posAttr::processPos;
	return 1;
}

//取得六个角度值的方法
int CURRobot::GetAngles()
{
	//这里实时读取UR数据（读取六个关节角度值，手册来看地址就是270）
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
	//对于六个角度值，只有double类型可以放这种双精度类型值
	double AnglesFiltered[6] = {0.0};

	for (int i = 0; i < 6; i++)
	{
		//这里取到的都是弧度值，要转换为角度值(除以1000之后还是弧度，要再转换为角度)
		AnglesFiltered[i] = SixAngles[i]/1000.0*(180/PI);

		if (IsSixAngles[i] > 0)
		{
			AnglesFiltered[i] = AnglesFiltered[i]-360.0;
		}		
	}

	//读到之后就改写自己的成员值(转换为弧度值)
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

//读取命令执行状态返回
int CURRobot::Read_commandBack()
{
	int cmdState[2] = {0};
	//Sleep(m_iWaitCMDBack);	//等待1s
	if (m_pModbus->ReadMultipleRegister(1, 145, cmdState) < 0)
	{
		return -1;
	}
	else
	{
		return cmdState[0];
	}
}



//从手册可以知道，260-265的寄存器保存了当前机械手的状态信息
int CURRobot::GetRobotState()
{
	//这里实时读取UR数据（读取是否开机，是否紧急停机等状态信息，手册来看地址就是260）
	int i_errorno = 0;
	if((i_errorno = m_pModbus->ReadMultipleRegister(6, 260,m_iarryRobotState)) < 0)
	{
		return -1;
	}
	return 1;
}



//移动重要函数
int CURRobot::MoveToPos(const UrRecPosAng& pos)
{
	return MoveL(const_cast<UrRecPosAng&>(pos),L"",0);
}


void CURRobot::Return2Ori()
{
	return ;
}


//将命令和位置信息转换为short类型
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
// 进入示教模式
int CURRobot::ChangeModeToTech(void)
{
	unsigned short data[2];
	if(Send_command(2,ToShortArry(2,data)) < 0)
	{
		return -1;
	}
	return 1;
}
// 结束示教模式
int CURRobot::EndTeachMode(void)
{
	unsigned short data[2];
	if(Send_command(3, ToShortArry(3,data)) < 0)
	{
		return -1;
	}
	return 1;
}

// 暂停机器人
int CURRobot::PauseUR(void)
{
	unsigned short data[2] = {0};
	if(Send_command(4, ToShortArry(4,data)) < 0)
	{
		return -1;
	}
	return 1;
}
// 关闭机器人
int CURRobot::StopUR(void)
{
	unsigned short data[2] = {0};
	if(Send_command(5, ToShortArry(5,data)) < 0)
	{
		return -1;
	}
	return  1;
}






//直线移动
int CURRobot::MoveL(UrRecPosAng& UrPos, const tstring& axis, const int idir)
{
	unsigned short data[17];
	return Send_command(1,ToShortArry(1, UrPos, data));
}

int CURRobot::Send_command(const unsigned char cmd, const unsigned short * const data)
{
	switch (cmd)
	{
	case 1://移动到某一位置 movel
		return m_pModbus->WriteMultipleRegister(17, 128, data);
	case 2:	//示教模式，可以进行调节机器人姿态
		return m_pModbus->WriteMultipleRegister(2, 128, data);
	case 3://停止示教模式
		return m_pModbus->WriteMultipleRegister(2, 128, data);
	case 4://暂停
		return m_pModbus->WriteMultipleRegister(2, 128, data);
	case 5://停止关机
		return m_pModbus->WriteMultipleRegister(2, 128, data);
	default:
		//ADDMSG(L"未定义指令！");
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

	//2017.01.16 决定角度还是位置 0 代表位置； 1代表角度
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
			//AfxWriteToLog("未加载程序!\n");
			return false;
		}
	}
	Sleep(300);
	//AfxWriteToLog("准备运行!\n");
	if ((m_pDashBoard->ReadDashBoard()) != 1)
	{
		Sleep(300);
		char *pplay = "play\n";
		if ((m_pDashBoard->WriteDashBoard(pplay)) != 1)
		{
			//AfxWriteToLog("未运行程序!\n");
			return false;
		}
	}
	return true;
}

void CURRobot::InitCmdNum()
{
	m_uchrCmdNum = 0;
}