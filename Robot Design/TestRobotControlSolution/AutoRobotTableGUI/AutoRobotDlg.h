#pragma once


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
};
