
// AutoRobotTableGUI.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CAutoRobotTableGUIApp: 
// �йش����ʵ�֣������ AutoRobotTableGUI.cpp
//

class CAutoRobotTableGUIApp : public CWinApp
{
public:
	CAutoRobotTableGUIApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CAutoRobotTableGUIApp theApp;