// AutoRobotDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "AutoRobotTableGUI.h"
#include "AutoRobotDlg.h"
#include "afxdialogex.h"


// CAutoRobotDlg 对话框

IMPLEMENT_DYNAMIC(CAutoRobotDlg, CDialog)

CAutoRobotDlg::CAutoRobotDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CAutoRobotDlg::IDD, pParent)
{

}

CAutoRobotDlg::~CAutoRobotDlg()
{
}

void CAutoRobotDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CAutoRobotDlg, CDialog)
	ON_BN_CLICKED(IDC_BUTTON_STARTPAUSE, &CAutoRobotDlg::OnBnClickedButtonStartpause)
END_MESSAGE_MAP()

CallBackFunc_RobotReachPos pRobotInPosFuc(void* pVoid)
{
	CAutoRobotDlg *pAutoRobotDlg = (CAutoRobotDlg*)pVoid;
	if (pAutoRobotDlg->m_evtRobotInPos)
	{
		pAutoRobotDlg->m_evtRobotInPos.SetEvent();
	}
}

// CAutoRobotDlg 消息处理程序
DWORD WINAPI CAutoRobotDlg::ThreadMoveAndSendMessage(PVOID pParam)
{
	CAutoRobotDlg *pAutoRobotDlg = (CAutoRobotDlg*)pParam;

	//单次运动
	pAutoRobotDlg->m_evtRobotInPos.ResetEvent();
	UrRecPosAng tempPosAng;	
	pAutoRobotDlg->m_pRobotControl->MoveToPos(tempPosAng, (CallBackFunc_RobotReachPos)pRobotInPosFuc, pAutoRobotDlg);
	WaitForSingleObject(pAutoRobotDlg->m_evtRobotInPos, INFINITE);

	
	return true;
}
void CAutoRobotDlg::OnBnClickedButtonStartpause()
{
	// TODO:  在此添加控件通知处理程序代码

	//根据选项读取路径

	//根据需求定义机器人回调函数
	
	//控制机器人运动
	HANDLE m_hThread;
	DWORD  m_dwThreadID;
	CreateThread(NULL, 0, ThreadMoveAndSendMessage, this, CREATE_SUSPENDED, &m_dwThreadID);
	//到位置通过Socket向扫描仪发送扫描指令

	//位置结束发送拼接指令
}


BOOL CAutoRobotDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  在此添加额外的初始化
	m_pRobotControl = nullptr;
	m_evtRobotInPos.ResetEvent();
	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常:  OCX 属性页应返回 FALSE
}
