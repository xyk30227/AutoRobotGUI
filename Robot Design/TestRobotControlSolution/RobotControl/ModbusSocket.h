/*
*	Copyright (c) 天远三维（天津）科技有限公司研发部
*	All rights reserved
*	
*	文件名称：ModbusSock.h
*	摘    要：Modbus通信协议类头文件
*	
*	当前版本：0.1
*	作    者：李辉
*	完成日期：2016年3月21日
*	
*	取代版本：
*	原作者  ：
*	完成日期：
**/

#ifndef MODBUS_SOCKET_H
#define MODBUS_SOCKET_H

//#pragma once
#include "defconfig.h"
//#include <winsock2.h> 
// #include <string>
// #include <iostream>
// #include <sstream>
// #include <vector>
// #include <stdio.h>
// #include <stdlib.h>
// #include <list>

using namespace std;
/*
该类实例化后需要通过Connect函数进行测试远程主机是否可用
*/

//ICMP头结构
typedef struct _icmphdr
{
	BYTE   i_type;    //类型
	BYTE   i_code;    //代码
	USHORT i_cksum;   //校验和
	USHORT i_id;      //标识符	
	USHORT i_seq;     //序列号
	ULONG  timestamp; //时间戳
}IcmpHeader;





class CModbusSocket
{
#define MB_ERROR_WSA			-1
#define MB_ERROR_SOCK			-2
#define MB_ERROR_SOCK_SET		-3
#define MB_ERROR_SOCK_CNT		-4
#define MB_ERROR_SOCK_ERROR		-5
#define MB_ERROR_DATA_NULL		-6
#define MB_ERROR_SERVER			-7
public:
	CModbusSocket(void);
	~CModbusSocket(void);

	//这是定义MODBUS的读与写
	enum FunctionCode 
	{
		//Read Multiple Registers
		Read = 3,
		//Write Multiple Registers
		Write = 16
	} ;

private:
	SOCKET sock;
	//有静态属性（远程IP，远程端口，超时时间，起始地址）
	tstring m_strRemoteIP;	//远程主机IP
	int m_iRemotePort;		//远程端口
	int m_iSocketTimeOut;	//IP连接超时

// 	//有一个Socket对象和IPE对象
// 	SOCKET sock;

	//服务器地址
	SOCKADDR_IN addrSrv ;
	//这里为MODBUS连接设置临界区，任意时刻只有一个连接
	CRITICAL_SECTION m_criSection;
public:

	//WSA 初始化标志位
	bool m_bWsaOn;

	//错误信息记录
	static int m_iErrorNo;
	//sock连接成功标志
	bool m_bSockOn;


	int InitModbusSocket(tstring UR_IP,DWORD UR_Port,int timeOut);
	//设置MODBUS server 的IP
	int SetServerIP(const tstring& IP);
	//连接主机
	int ConnectServer(void);

	//这里只是断开连接，而非关闭SOCKET句柄
	void CloseConnect(void);
	
	//WriteMultipleRegister方法需要传入写入寄存器的字符串（如“123|234|5555”，如果只有一个则不用分隔符），要写入寄存器的个数（1,2,3）要一一对应和写入的起始地址
	int WriteMultipleRegister(const unsigned int iRegisterNum,const int StartAddress, const unsigned short * const data);

	//ReadMultipleRegister需要传入的参数包括：要读取的寄存器个数（比如1,2,3），要读取的寄存器首地址（比如100）
	// ResigterNum 寄存器数量 StartAddress为起始地址 data为返回数据，不能为空
	int ReadMultipleRegister(const int ResigterNum,const int StartAddress,int * const data);

	//通过ping命令测试是否连通
	int PingToSever();
};

#endif
