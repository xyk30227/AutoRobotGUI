#pragma once

#include "API_RobotControl.h"
// CAutoRobotDlg 对话框

class CAutoRobotDlg : public CDialog
{
	DECLARE_DYNAMIC(CAutoRobotDlg)

public:
	CAutoRobotDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CAutoRobotDlg();

// 对话框数据
	enum { IDD = IDD_DIALOG_AUTOROBOT };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CRobotControl *m_pRobotControl;
	CEvent m_evtRobotInPos;
public:
	static DWORD WINAPI ThreadMoveAndSendMessage(PVOID pParam);
	//获取机器人控制指针
	void GetRobotControl(CRobotControl *&pRobotControl) { m_pRobotControl = pRobotControl; };
public:
	afx_msg void OnBnClickedButtonStartpause();
	virtual BOOL OnInitDialog();
};
