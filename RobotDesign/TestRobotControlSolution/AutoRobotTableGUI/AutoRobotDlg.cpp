// AutoRobotDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "AutoRobotTableGUI.h"
#include "AutoRobotDlg.h"
#include "afxdialogex.h"


// CAutoRobotDlg �Ի���

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

// CAutoRobotDlg ��Ϣ�������
DWORD WINAPI CAutoRobotDlg::ThreadMoveAndSendMessage(PVOID pParam)
{
	CAutoRobotDlg *pAutoRobotDlg = (CAutoRobotDlg*)pParam;

	//�����˶�
	pAutoRobotDlg->m_evtRobotInPos.ResetEvent();
	UrRecPosAng tempPosAng;	
	pAutoRobotDlg->m_pRobotControl->MoveToPos(tempPosAng, (CallBackFunc_RobotReachPos)pRobotInPosFuc, pAutoRobotDlg);
	WaitForSingleObject(pAutoRobotDlg->m_evtRobotInPos, INFINITE);

	
	return true;
}
void CAutoRobotDlg::OnBnClickedButtonStartpause()
{
	// TODO:  �ڴ���ӿؼ�֪ͨ����������

	//����ѡ���ȡ·��

	//��������������˻ص�����
	
	//���ƻ������˶�
	HANDLE m_hThread;
	DWORD  m_dwThreadID;
	CreateThread(NULL, 0, ThreadMoveAndSendMessage, this, CREATE_SUSPENDED, &m_dwThreadID);
	//��λ��ͨ��Socket��ɨ���Ƿ���ɨ��ָ��

	//λ�ý�������ƴ��ָ��
}


BOOL CAutoRobotDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  �ڴ���Ӷ���ĳ�ʼ��
	m_pRobotControl = nullptr;
	m_evtRobotInPos.ResetEvent();
	return TRUE;  // return TRUE unless you set the focus to a control
	// �쳣:  OCX ����ҳӦ���� FALSE
}
