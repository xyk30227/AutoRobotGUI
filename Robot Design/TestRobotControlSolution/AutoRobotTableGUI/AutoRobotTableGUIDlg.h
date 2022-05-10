
// AutoRobotTableGUIDlg.h : ͷ�ļ�
//

#pragma once
#include "AutoRobotDlg.h"
#include "ManualRobotDlg.h"
// CAutoRobotTableGUIDlg �Ի���
class CAutoRobotTableGUIDlg : public CDialogEx
{
// ����
public:
	CAutoRobotTableGUIDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_AUTOROBOTTABLEGUI_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��

	CTabCtrl m_RobotCtrl;
	CAutoRobotDlg m_AutoDlg;
	CManualRobotDlg m_ManualDlg;
	CDialog* m_pDialog[5];  //��������Ի������ָ��
	int m_CurSelTab = 0;
// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnTcnSelchangeTabAutomanual(NMHDR *pNMHDR, LRESULT *pResult);
};
