/*
*	Copyright (c) 天远三维（天津）科技有限公司研发部
*	All rights reserved
*	
*	文件名称：DashBoard.h
*	摘    要：DashBoard通信协议类头文件
*	
*	当前版本：1.0
*	作    者：xuyukai
*	完成日期：2016年12月15日
*	
*	取代版本：
*	原作者  ：
*	完成日期：
**/

//#pragma once
#include "defconfig.h"
//#include <winsock2.h> 


//#include "TYScan.h"

using namespace std;
/*
该类实例化后需要通过Connect函数进行测试远程主机是否可用
*/

class CDashBoard
{
#define MB_ERROR_WSA			-1
#define MB_ERROR_SOCK			-2
#define MB_ERROR_SOCK_SET		-3
#define MB_ERROR_SOCK_CNT		-4
#define MB_ERROR_SOCK_ERROR		-5
#define MB_ERROR_DATA_NULL		-6
#define MB_ERROR_SERVER			-7

public:
	CDashBoard(void);
	~CDashBoard(void);
private:
	SOCKET sock;            //有静态属性（远程IP，远程端口，超时时间，起始地址）	
	tstring m_strRemoteIP;	//远程主机IP
	int m_iRemotePort;		//远程端口
	int m_iSocketTimeOut;	//IP连接超时	
	SOCKADDR_IN addrSrv ;   //服务器地址	
	CRITICAL_SECTION m_criSection; //这里为MODBUS连接设置临界区，任意时刻只有一个连接
public:	
	bool m_bWsaOn;			//WSA 初始化标志位	
	static int m_iErrorNo;  //错误信息记录	
	bool m_bSockOn;			//sock连接成功标志

	//函数：InitDashBoard
	//功能：初始化DashBoard
	//输入: UR_IP 机器人IP  UR_Port 机器人端口 timeOut 超时
	//返回：正确返回 1  错误返回 -1	
	int InitDashBoard(tstring UR_IP,DWORD UR_Port,int timeOut);
	
	//函数：SetServerIP
	//功能：设置MODBUS server 的IP
	//输入: UR_IP 机器人IP 
	//返回：正确返回 1  错误返回 -1		
	int SetServerIP(const tstring& IP);

	//函数：ConnectServer
	//功能：连接主机
	//输入: 无
	//返回：正确返回 1  错误返回 -1		
	int ConnectServer(void);

	//函数：CloseConnect
	//功能：断开连接，而非关闭SOCKET句柄
	//输入: 无
	//返回：
	void CloseConnect(void);

	//函数：WriteDashBoard
	//功能：DashBoard 写入功能，主要是一些基本信息的写入
	//输入: const char* 输入字符
	//返回：正确返回 1  错误返回 -1	
	int WriteDashBoard(const char* writedata);


	//函数：ReadDashBoard
	//功能：DashBoard 读取功能，主要用于进行示教器状态的查询
	//输入: 无	
	//返回：0.其他情况 1.play  2.pasued  3.stop  
	int ReadDashBoard();

	//函数：IsMessageMatch
	//功能：判断输入指令和输出指令是否匹配
	//输入: indata 输入字符串 outdata 机器人返回字符串
	//返回：正确返回 true 错误返回 false
	bool IsMessageMatch(const char *indata,const char *outdata);
};

