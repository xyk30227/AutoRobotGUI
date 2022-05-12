
// AutoRobotTableGUIDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "AutoRobotTableGUI.h"
#include "AutoRobotTableGUIDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CAutoRobotTableGUIDlg 对话框



CAutoRobotTableGUIDlg::CAutoRobotTableGUIDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CAutoRobotTableGUIDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CAutoRobotTableGUIDlg::DoDataExchange(CDataExchange* pDX)
{
	DDX_Control(pDX, IDC_TAB_AUTOMANUAL, m_RobotCtrl);
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAutoRobotTableGUIDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_AUTOMANUAL, &CAutoRobotTableGUIDlg::OnTcnSelchangeTabAutomanual)
	ON_BN_CLICKED(IDC_BUTTON_INITDEVICE, &CAutoRobotTableGUIDlg::OnBnClickedButtonInitdevice)
	ON_BN_CLICKED(IDC_BUTTON_RELEASEDEVICE, &CAutoRobotTableGUIDlg::OnBnClickedButtonReleasedevice)
	ON_WM_DESTROY()
END_MESSAGE_MAP()


// CAutoRobotTableGUIDlg 消息处理程序

BOOL CAutoRobotTableGUIDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO:  在此添加额外的初始化代码
	//为Tab Control增加两个页面
	m_RobotCtrl.InsertItem(0, _T("手动示教"));
	m_RobotCtrl.InsertItem(1, _T("自动检测"));

	//创建两个对话框
	m_ManualDlg.Create(IDD_DIALOG_MANUALROBOT, &m_RobotCtrl);
	m_AutoDlg.Create(IDD_DIALOG_AUTOROBOT, &m_RobotCtrl);
	//设定在Tab内显示的范围
	CRect rc;
	m_RobotCtrl.GetClientRect(rc);
	rc.top += 20;
	rc.bottom -= 0;
	rc.left += 0;
	rc.right -= 0;
	m_ManualDlg.MoveWindow(&rc);
	m_AutoDlg.MoveWindow(&rc);

	//把对话框对象指针保存起来
	m_pDialog[0] = &m_ManualDlg;
	m_pDialog[1] = &m_AutoDlg;
	//显示初始页面
	m_pDialog[0]->ShowWindow(SW_SHOW);
	m_pDialog[1]->ShowWindow(SW_HIDE);
	//保存当前选择
	m_CurSelTab = 0;
	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CAutoRobotTableGUIDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CAutoRobotTableGUIDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CAutoRobotTableGUIDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CAutoRobotTableGUIDlg::OnTcnSelchangeTabAutomanual(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO:  在此添加控件通知处理程序代码

	//把当前的页面隐藏起来
	m_pDialog[m_CurSelTab]->ShowWindow(SW_HIDE);
	//得到新的页面索引
	m_CurSelTab = m_RobotCtrl.GetCurSel();
	//把新的页面显示出来
	m_pDialog[m_CurSelTab]->ShowWindow(SW_SHOW);

	*pResult = 0;
}


void CAutoRobotTableGUIDlg::OnBnClickedButtonInitdevice()
{
	// TODO:  在此添加控件通知处理程序代码
	m_pRobotControl = CRobotControl::CreateRobotControl();
	int iRet = m_pRobotControl->ConnectRobot(0, "192.168.1.10");
	
	//获取机器人指针
	m_AutoDlg.GetRobotControl(m_pRobotControl);
	m_ManualDlg.GetRobotControl(m_pRobotControl);
}


void CAutoRobotTableGUIDlg::OnBnClickedButtonReleasedevice()
{
	// TODO:  在此添加控件通知处理程序代码
	CRobotControl::ReleaseRobotControl(m_pRobotControl);

}


void CAutoRobotTableGUIDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO:  在此处添加消息处理程序代码
	OnBnClickedButtonReleasedevice();
}
