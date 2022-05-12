#pragma once

#include "API_RobotControl.h"
// CManualRobotDlg 对话框

class CManualRobotDlg : public CDialog
{
	DECLARE_DYNAMIC(CManualRobotDlg)

public:
	CManualRobotDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CManualRobotDlg();

// 对话框数据
	enum { IDD = IDD_DIALOG_MANUALROBOT };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()

public:
	CRobotControl *m_RobotControl;
public:
	//获取机器人控制指针
	void GetRobotControl(CRobotControl *&pRobotControl) { m_RobotControl = pRobotControl; };
};
