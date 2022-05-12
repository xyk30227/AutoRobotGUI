#pragma once

#include "API_RobotControl.h"
// CAutoRobotDlg �Ի���

class CAutoRobotDlg : public CDialog
{
	DECLARE_DYNAMIC(CAutoRobotDlg)

public:
	CAutoRobotDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CAutoRobotDlg();

// �Ի�������
	enum { IDD = IDD_DIALOG_AUTOROBOT };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	CRobotControl *m_pRobotControl;
	CEvent m_evtRobotInPos;
public:
	static DWORD WINAPI ThreadMoveAndSendMessage(PVOID pParam);
	//��ȡ�����˿���ָ��
	void GetRobotControl(CRobotControl *&pRobotControl) { m_pRobotControl = pRobotControl; };
public:
	afx_msg void OnBnClickedButtonStartpause();
	virtual BOOL OnInitDialog();
};
