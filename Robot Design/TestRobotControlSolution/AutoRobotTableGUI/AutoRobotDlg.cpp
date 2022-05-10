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
END_MESSAGE_MAP()


// CAutoRobotDlg 消息处理程序
