
// AutoRobotTableGUIDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "AutoRobotTableGUI.h"
#include "AutoRobotTableGUIDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
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


// CAutoRobotTableGUIDlg �Ի���



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


// CAutoRobotTableGUIDlg ��Ϣ�������

BOOL CAutoRobotTableGUIDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// ��������...���˵�����ӵ�ϵͳ�˵��С�

	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
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

	// ���ô˶Ի����ͼ�ꡣ  ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	// TODO:  �ڴ���Ӷ���ĳ�ʼ������
	//ΪTab Control��������ҳ��
	m_RobotCtrl.InsertItem(0, _T("�ֶ�ʾ��"));
	m_RobotCtrl.InsertItem(1, _T("�Զ����"));

	//���������Ի���
	m_ManualDlg.Create(IDD_DIALOG_MANUALROBOT, &m_RobotCtrl);
	m_AutoDlg.Create(IDD_DIALOG_AUTOROBOT, &m_RobotCtrl);
	//�趨��Tab����ʾ�ķ�Χ
	CRect rc;
	m_RobotCtrl.GetClientRect(rc);
	rc.top += 20;
	rc.bottom -= 0;
	rc.left += 0;
	rc.right -= 0;
	m_ManualDlg.MoveWindow(&rc);
	m_AutoDlg.MoveWindow(&rc);

	//�ѶԻ������ָ�뱣������
	m_pDialog[0] = &m_ManualDlg;
	m_pDialog[1] = &m_AutoDlg;
	//��ʾ��ʼҳ��
	m_pDialog[0]->ShowWindow(SW_SHOW);
	m_pDialog[1]->ShowWindow(SW_HIDE);
	//���浱ǰѡ��
	m_CurSelTab = 0;
	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
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

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ  ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CAutoRobotTableGUIDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù��
//��ʾ��
HCURSOR CAutoRobotTableGUIDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CAutoRobotTableGUIDlg::OnTcnSelchangeTabAutomanual(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO:  �ڴ���ӿؼ�֪ͨ����������

	//�ѵ�ǰ��ҳ����������
	m_pDialog[m_CurSelTab]->ShowWindow(SW_HIDE);
	//�õ��µ�ҳ������
	m_CurSelTab = m_RobotCtrl.GetCurSel();
	//���µ�ҳ����ʾ����
	m_pDialog[m_CurSelTab]->ShowWindow(SW_SHOW);

	*pResult = 0;
}


void CAutoRobotTableGUIDlg::OnBnClickedButtonInitdevice()
{
	// TODO:  �ڴ���ӿؼ�֪ͨ����������
	m_pRobotControl = CRobotControl::CreateRobotControl();
	int iRet = m_pRobotControl->ConnectRobot(0, "192.168.1.10");
	
	//��ȡ������ָ��
	m_AutoDlg.GetRobotControl(m_pRobotControl);
	m_ManualDlg.GetRobotControl(m_pRobotControl);
}


void CAutoRobotTableGUIDlg::OnBnClickedButtonReleasedevice()
{
	// TODO:  �ڴ���ӿؼ�֪ͨ����������
	CRobotControl::ReleaseRobotControl(m_pRobotControl);

}


void CAutoRobotTableGUIDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO:  �ڴ˴������Ϣ����������
	OnBnClickedButtonReleasedevice();
}
