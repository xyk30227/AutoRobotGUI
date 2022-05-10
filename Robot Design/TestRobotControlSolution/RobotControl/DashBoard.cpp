//#include "StdAfx.h"

#include <afxsock.h>
//#include <winsock2.h> 

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <list>

#include "DashBoard.h"

int CDashBoard::m_iErrorNo = 0;
int CDashBoard::InitDashBoard(tstring UR_IP,DWORD UR_Port,int timeOut)
{
	m_strRemoteIP = UR_IP;
	m_iRemotePort = 29999;
	m_iSocketTimeOut = timeOut;
	m_bWsaOn = false;
	//m_bSockOn = false;
	WSADATA wdata;
	if ( WSAStartup(MAKEWORD(2,2), &wdata) !=0 )
	{
		m_iErrorNo = WSAGetLastError();
		return -1;
	}
	m_bWsaOn = true;
	int iRec = ConnectServer();
	if (iRec < 0)
	{
		m_bSockOn = false;
		return -1;
	}
	InitializeCriticalSection(&m_criSection);  //使用前必须初始化
	return 1;
}

CDashBoard::CDashBoard(void)
{

}


CDashBoard::~CDashBoard(void)
{
	EnterCriticalSection(&m_criSection);
	CloseConnect();
	if (m_bWsaOn)
	{
		WSACleanup();
	}
	LeaveCriticalSection(&m_criSection);
	DeleteCriticalSection(&m_criSection);
	//AfxWriteToLog("析构DASHBOARD完成\n");
}

int CDashBoard::SetServerIP(const tstring& IP)
{
	m_strRemoteIP = IP;
	addrSrv.sin_addr.S_un.S_addr = inet_addr(string(IP.begin(),IP.end()).c_str());
	if (m_bSockOn)
	{
		m_bSockOn = false;
		CloseConnect();
	}
	return ConnectServer();
}

 //这里只是断开连接，而非关闭SOCKET句柄
int CDashBoard::ConnectServer()
{		
	//listMsg.push_back(L"Modbus Sock 设置成功！");
	//用同步方法进行连接
	//先初始化地址
	if (m_bWsaOn == false)
	{		
		return (m_iErrorNo = MB_ERROR_SERVER);
	}
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_addr.S_un.S_addr = inet_addr((string(m_strRemoteIP.begin(),m_strRemoteIP.end())).c_str());
	addrSrv.sin_port = htons(m_iRemotePort);
	sock = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
	if (sock == INVALID_SOCKET)
	{
		goto ErrorBack;
	}

	//设置这个socket的超时时间
	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = m_iSocketTimeOut;
	int result = setsockopt(sock,SOL_SOCKET,SO_SNDTIMEO,(char*)&timeout,sizeof(timeout));
	if (result < 0)
	{
		goto ErrorBack;
	}

	result = connect(sock,(sockaddr*)&addrSrv,sizeof(sockaddr));
#ifdef ROBOT_TEST
#else
	char m_charreceive[128] = {0};
	result = recv(sock, m_charreceive, 128, 0);
//	AfxWriteToLog("接收返回值！\n");
	if (strcmp(m_charreceive, "Connected: Universal Robots Dashboard Server\n") == 0)
	{
//		AfxWriteToLog("返回值正确！\n");
		NULL;
	}
	else
	{
		goto ErrorBack;
	}
#endif



	if (result < 0)
	{		
		goto ErrorBack;
	}

	m_bSockOn  = true;
	return TRUE;
ErrorBack:
	m_bSockOn = false;
	m_iErrorNo = 0 - WSAGetLastError();
	throw(m_iErrorNo);
	closesocket(sock);
	//LeaveCriticalSection(&m_criSection);
	return m_iErrorNo;
}
//这里只是断开连接，而非关闭SOCKET句柄
void CDashBoard::CloseConnect()
{
	shutdown(sock,SD_BOTH);
	closesocket(sock);
	sock = INVALID_SOCKET;
}


int CDashBoard::WriteDashBoard(const char* data)
{
	//将传入的字符串按RegisterDivision进行分割
	//创建一个列表，用于存储数据
	//int errorcode = 0 ;
	EnterCriticalSection(&m_criSection);
	

	int size = send(sock, data, (int)strlen(data), 0);
	if (size < 0)
	{
		goto ErrorWrite;
	}
	//这里要读取数据报告
	char ReadBufferData[256] = {0};
	size = recv(sock,ReadBufferData,256,0);

//	AfxWriteToLog(ReadBufferData);
	if (!IsMessageMatch(data, ReadBufferData))
	{
//		AfxWriteToLog("返回错误!\n");
		LeaveCriticalSection(&m_criSection);
		return 0;
	}
	LeaveCriticalSection(&m_criSection);
	return 1;
ErrorWrite:
	m_iErrorNo = 0 - WSAGetLastError();

	LeaveCriticalSection(&m_criSection);
	return m_iErrorNo;
}


int CDashBoard::ReadDashBoard()
{
	EnterCriticalSection(&m_criSection);

	if (m_bSockOn == false)
	{
		return m_iErrorNo;
	}
	
	//2 发送查询命令
	int m_ireturnvalue = 0;
	char *m_charcheck = "programState\n";
	//AfxWriteToLog(m_charcheck);
	int size = send(sock, m_charcheck, (int)strlen(m_charcheck), 0);
	//AfxWriteToLog("Dashboard查询发送完成\n");
	char m_charrecvdata[128] = {0};
	size = recv(sock, m_charrecvdata, 128, 0);
	//AfxWriteToLog(m_charrecvdata);
	if ( strcmp(m_charcheck , "programState\n") == 0)
	{
		if (strcmp(m_charrecvdata , "PLAYING movel.urp\n") == 0)
		{
			m_ireturnvalue = 1;
			//return 1;
		}
		else if (strcmp(m_charrecvdata , "PAUSED movel.urp\n") == 0)
		{
			m_ireturnvalue = 2;
			//return 2;
		}
		else if (strcmp(m_charrecvdata , "STOPPED movel.urp\n") == 0)
		{
			m_ireturnvalue = 3;
			//return 3;
		}
	}
	
	LeaveCriticalSection(&m_criSection);
	//CloseConnect();
	return m_ireturnvalue;
}


bool CDashBoard::IsMessageMatch(const char* indata, const char *outdata)
{
	if (indata == NULL || outdata == NULL)
	{
		return false;
	}

	if (strcmp(indata, "get loaded program\n") == 0)
	{
		if (strcmp(outdata, "Loaded program: /programs/movel.urp\n") == 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else if (strcmp(indata, "load /programs/movel.urp\n") == 0)
	{
		if (strcmp(outdata, "Loading program: /programs/movel.urp\n") == 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else if (strcmp(indata , "play\n") == 0)
	{
		if (strcmp(outdata , "Starting program\n") == 0)
		{
			return true;
		}
		else 
		{
			return false;
		}
	}
	else if (strcmp(indata , "pause\n") ==0 )
	{
		if (strcmp(outdata , "Pausing program\n") == 0)
		{
			return true;
		}
		else 
		{
			return false;
		}
	}
	else if (strcmp(indata ,"running\n") == 0)
	{
		if (strcmp(outdata ,"Robot running: True\n") == 0)
		{
			return true;
		}
		else 
		{
			return false;
		}
	}
	else
	{
		return true;
	}
}