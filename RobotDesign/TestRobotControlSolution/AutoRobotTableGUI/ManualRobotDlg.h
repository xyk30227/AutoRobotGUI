#pragma once

#include "API_RobotControl.h"
// CManualRobotDlg �Ի���

class CManualRobotDlg : public CDialog
{
	DECLARE_DYNAMIC(CManualRobotDlg)

public:
	CManualRobotDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CManualRobotDlg();

// �Ի�������
	enum { IDD = IDD_DIALOG_MANUALROBOT };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()

public:
	CRobotControl *m_RobotControl;
public:
	//��ȡ�����˿���ָ��
	void GetRobotControl(CRobotControl *&pRobotControl) { m_RobotControl = pRobotControl; };
};
