
// AutoRobotTableGUIDlg.h : 头文件
//

#pragma once
#include "AutoRobotDlg.h"
#include "ManualRobotDlg.h"
// CAutoRobotTableGUIDlg 对话框
class CAutoRobotTableGUIDlg : public CDialogEx
{
// 构造
public:
	CAutoRobotTableGUIDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_AUTOROBOTTABLEGUI_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持

	CTabCtrl m_RobotCtrl;
	CAutoRobotDlg m_AutoDlg;
	CManualRobotDlg m_ManualDlg;
	CDialog* m_pDialog[5];  //用来保存对话框对象指针
	int m_CurSelTab = 0;
// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnTcnSelchangeTabAutomanual(NMHDR *pNMHDR, LRESULT *pResult);
};
