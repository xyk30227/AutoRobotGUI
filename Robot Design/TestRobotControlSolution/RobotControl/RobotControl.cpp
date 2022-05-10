// RobotType.cpp : 定义 DLL 应用程序的导出函数。
//

#ifndef ROBOT_CONTROL_CPP
#define ROBOT_CONTROL_CPP
#endif

#include <afxmt.h>
#include <afxwin.h>

#include "API_RobotControl.h"
#include "URRobot.h"
#include "FRANCRobot.h"

static FILE* gs_pLogFile = NULL;
CWinApp theApp;
CEvent g_eventfresh, g_eventmove;
using namespace std;

void AfxWriteToLog(const char* str, ...)
{
#ifndef DEBUG_SV_LOG
	//return;
#endif
	char     buffer[512];
	va_list  args;
	va_start(args, str);
	vsprintf(buffer, str, args);
	//std::printf( buffer );
	if (gs_pLogFile == NULL)
	{
		return;
	}
	fprintf(gs_pLogFile, buffer);
	va_end(args);

	fflush(gs_pLogFile);
}

CRobotControl::CRobotControl(void)
{	
	m_pRobotObj = nullptr;
	m_bWorking = false;
	m_bMoving = false;
	m_pvParams=NULL;
	m_pfOnRobotReachPos=NULL;
	g_eventfresh.SetEvent();
	g_eventmove.SetEvent();
}

CRobotControl::~CRobotControl(void)
{	
	m_bWorking = false;
	m_bMoving = false;	
	m_pvParams=NULL;
	m_pfOnRobotReachPos=NULL;
	WaitForSingleObject(g_eventfresh, INFINITE);
	WaitForSingleObject(g_eventmove, INFINITE);
}



DWORD CRobotControl::FreshData(LPVOID lpData)
{
	CRobotControl *pRobotTy = (CRobotControl*)(lpData);
	while (pRobotTy->m_bWorking)
	{
		Sleep(100);
		if (pRobotTy->GetRobotInformation() < 0)
		{
			pRobotTy->m_bWorking = false;
			break;
		}
		else
		{
			//pRobotTy->m_bWorking = true;
			pRobotTy->GetCurAngles(pRobotTy->m_curUrPosAng,processPos);
			pRobotTy->GetCurPositions(pRobotTy->m_curUrPosAng,processPos);			
		}
	}
	g_eventfresh.SetEvent();
	//WaitForSingleObject();
	return 1;
}

RC_STATUS CRobotControl::ConnectRobot(int iRobotType, const char* strRobotIP)
{
	CString strLogFile = L"F:\\log.txt";
	USES_CONVERSION;
	const char* acFileName = T2A(strLogFile);
	CString strTmpRobotIP  = A2T(strRobotIP);

	gs_pLogFile = fopen(acFileName, "a+");

	switch (iRobotType)
	{
	case ROBOT_UR:
		m_pRobotObj = std::make_shared<CURRobot>();
		break;
	case ROBOT_FANUC:
		m_pRobotObj = std::make_shared<CFRANCRobot>();
		break;
	default:
		return RC_FAIL;
	}
	
	//if (((CRobotObject *)m_pRobotObj.get())->InitRobot(RobotIP))
	m_pRobotObj->m_dbBasicSpeed = 1235;
	double *dbData =&( m_pRobotObj->m_dbBasicSpeed);
	int iSize = sizeof(UrRecPosAng);
	if (m_pRobotObj->InitRobot(strTmpRobotIP.GetString()) > 0)
	{
#ifdef ROBOT_TEST
#else
		m_bWorking = true;
		g_eventfresh.ResetEvent();
		DWORD ThreadId;
		HANDLE hThread;
		hThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)FreshData, this, 0, &ThreadId);
#endif		
		return RC_OK;
	}
	else
	{
		return RC_FAIL;
	}
}


RC_STATUS CRobotControl::DisconnectRobot()
{
	return RC_STATUS(m_pRobotObj ? m_pRobotObj->CloseRobot() : -1);
}

int CRobotControl::GetRobotInformation()
{
	return m_pRobotObj ? m_pRobotObj->GetRobotInformation() : -1;
}

RC_STATUS CRobotControl::GetCurPositions(UrRecPosAng& pPosAng, enum _posAttr posAttr)
{
#ifdef ROBOT_TEST
	double dPos[6] = { 0.5 };
	memcpy(pPosAng.dbPos, dPos, sizeof(double) * 6);
	pPosAng.posAttr = posAttr;
	return 1;
#else
	return RC_STATUS(m_pRobotObj ? m_pRobotObj->GetCurPositions(pPosAng, posAttr) : -1);
#endif	
}

RC_STATUS CRobotControl::GetCurAngles(UrRecPosAng& pPosAng, enum _posAttr posAttr)
{
//#ifdef ROBOT_TEST
//	double dAng[6] = { 0.5 };
//	memcpy(pPosAng.dbAng, dAng, sizeof(double) * 6);
//	pPosAng.posAttr = posAttr;
//	return 1;
//#else
	return RC_STATUS(m_pRobotObj ? m_pRobotObj->GetCurAngles(pPosAng, posAttr) : -1);
//#endif	
}

RC_STATUS CRobotControl::GetCurRobotState(URState &pState)
{
	return RC_STATUS(m_pRobotObj ? m_pRobotObj->GetCurRobotState(pState) : -1);
}


RC_STATUS CRobotControl::MoveToPos(const UrRecPosAng& pos, CallBackFunc_RobotReachPos pfOnRobotReachPos, void* pvParams)
{	
	m_PosAng2Move = pos;
	m_pfOnRobotReachPos=pfOnRobotReachPos;
	m_pvParams=pvParams;

	if (m_pRobotObj->MoveToPos(pos) > 0)
	{
		m_bMoving = true;
		g_eventmove.ResetEvent();
		EndMove();
		return RC_OK;
	}
	else
	{
		return RC_FAIL;
	}
}

void CRobotControl::EndMove()
{
	DWORD ThreadId;
	HANDLE hThread;
	hThread = CreateThread(NULL, 1, (LPTHREAD_START_ROUTINE)CallMoveBack, this, 0, &ThreadId);

// 	while (m_bMoving)
// 	{
// 		Sleep(100);
// 		if (m_pRobotObj->Read_commandBack() == m_pRobotObj->m_uchrCmdNum)
// 		{
// 			AfxWriteToLog("%d\n", m_pRobotObj->m_uchrCmdNum);
// 			OnMoveCallBack();
// 			m_bMoving = false;
// 			break;
// 		}
// 	}
// 	g_eventmove.SetEvent();
// 	return ;

}
//使用时请重载改函数，移动完成后调用该函数
DWORD CRobotControl::CallMoveBack(LPVOID lpData)
{
	CRobotControl *pRobotTy = (CRobotControl*)(lpData);
	DWORD dW1 = GetTickCount();
	while (pRobotTy->m_bMoving)
	{
		Sleep(100);
//		UrRecPosAng tempPosAng;
//		pRobotTy->GetCurAngles(tempPosAng, processPos);
// 		int iMove2Point = 0;
// 
// 		for (int i = 0; i < 6; i++)
// 		{
// 			if (abs(tempPosAng.dbAng[i] - pRobotTy->m_PosAng2Move.dbAng[i]) < 1.000)
// 			{
// 				iMove2Point++;
// 			}
// 		}
// 		AfxWriteToLog("m_uchrCmdNum:%d\n", pRobotTy->m_pRobotObj->m_uchrCmdNum);
// 
// 		AfxWriteToLog("tempPosAng:%lf,%lf,%lf,%lf,%lf,%lf\n", tempPosAng.dbAng[0], tempPosAng.dbAng[1], tempPosAng.dbAng[2], tempPosAng.dbAng[3], tempPosAng.dbAng[4], tempPosAng.dbAng[5]);
// 		AfxWriteToLog("m_PosAng2Move:%lf,%lf,%lf,%lf,%lf,%lf\n", pRobotTy->m_PosAng2Move.dbAng[0], pRobotTy->m_PosAng2Move.dbAng[1], pRobotTy->m_PosAng2Move.dbAng[2],
// 																  pRobotTy->m_PosAng2Move.dbAng[3], pRobotTy->m_PosAng2Move.dbAng[4], pRobotTy->m_PosAng2Move.dbAng[5]);

//		if ((pRobotTy->m_pRobotObj->Read_commandBack() == pRobotTy->m_pRobotObj->m_uchrCmdNum) && (iMove2Point == 6))

		if (pRobotTy->m_pRobotObj->Read_commandBack() == pRobotTy->m_pRobotObj->m_uchrCmdNum )
		{
			AfxWriteToLog("%d\n", pRobotTy->m_pRobotObj->m_uchrCmdNum);
			//pRobotTy->OnMoveCallBack();

			if(pRobotTy->m_pfOnRobotReachPos)
			{
				pRobotTy->m_pfOnRobotReachPos(pRobotTy->m_pvParams);
			}
			pRobotTy->m_bMoving = false;			
			break;
		}		
	}
	g_eventmove.SetEvent();
	return 0;
}

// void CRobotControl::OnMoveCallBack()
// {
// 	//printf("位置到达!\n");
// }

void CRobotControl::Return2Ori()
{
	//待完善
	return;
}

bool CRobotControl::LoadAndOpenProgram()
{
	return m_pRobotObj ? m_pRobotObj->LoadAndOpenProgram() : false;
}

void CRobotControl::InitCmdNum()
{
	m_pRobotObj->InitCmdNum();
}


