/*
*	Copyright (c) 天远三维（天津）科技有限公司研发部
*	All rights reserved
*	
*	文件名称：ModbusSock.cpp
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

//#include "stdafx.h"
#include <afxsock.h>

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <list>

#include "ModbusSocket.h"
//#include "TYScan.h"
//#include "MainFrm.h"

//static SOCKET sock = INVALID_SOCKET;
//计算校验和
USHORT checksum(USHORT *buffer,int size)
{
	unsigned long cksum = 0;
	while (size > 1)
	{
		cksum += *buffer++;
		size -= sizeof(USHORT);
	}
	if (size)
	{
		cksum += *(UCHAR *)buffer;
	}
	cksum = (cksum >> 16)+(cksum & 0xffff);
	cksum += (cksum >> 16);
	return (USHORT)(~cksum);
}

int CModbusSocket::m_iErrorNo = 0;
CModbusSocket::CModbusSocket(void)
{

}


CModbusSocket::~CModbusSocket(void)
{
	EnterCriticalSection(&m_criSection);
	CloseConnect();
	if (m_bWsaOn)
	{
		WSACleanup();
	}
	LeaveCriticalSection(&m_criSection);
	DeleteCriticalSection(&m_criSection);
//	AfxWriteToLog("析构MODBUS完成\n");
}
int CModbusSocket::InitModbusSocket(tstring UR_IP,DWORD UR_Port,int timeOut)
{
	m_strRemoteIP = UR_IP;
	m_iRemotePort = UR_Port;
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

	//i_contoser = PingToSever();
	
	int iRec = ConnectServer();
	if (iRec < 0)
	{
		m_bSockOn = false;
		return -1;
	}
	InitializeCriticalSection(&m_criSection);  //使用前必须初始化
	return 1;
}

int CModbusSocket::SetServerIP(const tstring& IP)
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


int CModbusSocket::PingToSever()
{
	SOCKET  sockRaw;
	struct sockaddr_in dest,from;
	char recvbuf[100],f_name[100];
	int fromlen = sizeof(from);
	int timeout = 100;
	int iSuccess = 0;
	if (m_bWsaOn == false)
	{
		////AfxWriteToCommand(L"m_bWsaOn = false\n");
		return -1;
	}
	sockRaw = socket(AF_INET,SOCK_RAW,IPPROTO_ICMP);
	auto ret = WSAGetLastError();
	setsockopt(sockRaw,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout,sizeof(timeout));
	memset(&dest,0,sizeof(dest));
	dest.sin_addr.s_addr = inet_addr(string(m_strRemoteIP.begin(),m_strRemoteIP.end()).c_str());
	dest.sin_family = AF_INET;
	dest.sin_port = 25555;
	char icmp_data[10] = {0};
	memset(icmp_data,0,sizeof(icmp_data));
	((IcmpHeader *)icmp_data)->i_type = 8;
	((IcmpHeader *)icmp_data)->i_code = 0;
	((IcmpHeader *)icmp_data)->i_id   = (u_short)GetCurrentProcessId();
	((IcmpHeader *)icmp_data)->i_seq  = 0;
	gethostname(f_name,100);
	for (int k = 0; k < 3; k++)
	{
		((IcmpHeader *)icmp_data)->i_cksum = 0;
		((IcmpHeader *)icmp_data)->i_cksum = checksum((u_short*)icmp_data, 8);
		sendto(sockRaw,icmp_data, 8, 0, (struct sockaddr *)&dest, sizeof(dest));
		int bread = recvfrom(sockRaw,recvbuf, 100, 0, (struct sockaddr*)&from, &fromlen);
		if (bread == SOCKET_ERROR)
		{
			////AfxWriteToCommand(L"bread = SOCKET_ERROR\n");
			iSuccess = -1;
			m_bSockOn = false;
			continue;
		}
		else
		{
			((IcmpHeader *)icmp_data)->i_seq++;
			iSuccess = 1;
			continue;
		}
	}
	closesocket(sockRaw);
	return iSuccess;
}
 //这里只是断开连接，而非关闭SOCKET句柄
int CModbusSocket::ConnectServer()
{		
	//listMsg.push_back(L"Modbus Sock 设置成功！");
	//用同步方法进行连接
	//先初始化地址
	if (m_bWsaOn == false)
	{		
		return (m_iErrorNo = MB_ERROR_SERVER);
	}
	if (PingToSever() < 0)
	{
		return (m_iErrorNo = MB_ERROR_SERVER);
	}
	//EnterCriticalSection(&m_criSection);
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
	if (result < 0)
	{		
		goto ErrorBack;
	}

	m_bSockOn  = true;
	return TRUE;
ErrorBack:
	m_bSockOn = false;
	m_iErrorNo = 0 - WSAGetLastError();
	//throw(m_iErrorNo);
	closesocket(sock);
	//LeaveCriticalSection(&m_criSection);
	return m_iErrorNo;
}
//这里只是断开连接，而非关闭SOCKET句柄
void CModbusSocket::CloseConnect()
{
	shutdown(sock,SD_BOTH);
	closesocket(sock);
	sock = INVALID_SOCKET;
//	LeaveCriticalSection(&m_criSection);
	/*if (m_bSockOn == false)
	{
		return;
	}
	shutdown(sock,SD_BOTH);*/	
}

//WriteMultipleRegister方法需要传入写入寄存器的字符串（如“123|234|5555”，如果只有一个则不用分隔符），要写入寄存器的个数（1,2,3）要一一对应和写入的起始地址
int CModbusSocket::WriteMultipleRegister(const unsigned int iRegisterNum,const int StartAddress, const unsigned short * const data)
{
	//将传入的字符串按RegisterDivision进行分割
	//创建一个列表，用于存储数据
	//int errorcode = 0 ;
	EnterCriticalSection(&m_criSection);
	if (data == NULL)
	{
		return MB_ERROR_DATA_NULL;
	}
	if (m_bSockOn == false)
	{
		return m_iErrorNo;
	}
	/*if ((errorcode = ConnectServer()) < 0 )
	{
	CloseConnect();
	return errorcode;
	}*/

	//把分割好的字符串存入byte数组(这里已经知道了到底要写入几个寄存器，也就知道了数组长度，因为寄存器分高低，如果我只要写1个寄存器，则就要有2个byte，以此类推)
	vector<unsigned char> uchrtab;
	for (unsigned int i = 0; i < iRegisterNum; i++)
	{
		uchrtab.push_back(data[i]/256);
		uchrtab.push_back(data[i]%256);
	}

	vector<unsigned char> values;

	//1 凑Header

	//1.1（数据位：1-2） 定义我是谁(Transaction Identifier),协议给了我2byte空间定义我是谁（我代号就是01，如果Modbus TCP Server接收成功，返回信息也要有这个代号）
	values.push_back(0);
	values.push_back(1);

	//1.2 （数据位：3-4）定义协议号，协议给了我2byte空间定义协议号（协议号就是00，表示这是MODBUS 协议）
	values.push_back(0);
	values.push_back(0);

	//1.3 （数据位：5-6）现在考虑将56两位凑到一块
	values.push_back(unsigned char((uchrtab.size() + 7) / 256));
	values.push_back(unsigned char((uchrtab.size() + 7) % 256));


	//1.4 （数据位：7）只要补全一个0即可
	values.push_back(0);

	//1.5 （数据位：8）定义功能码（把写多个寄存器这个16码转换为byte类型数据）Function Code : 16 (Write Multiple Register)
	values.push_back((BYTE)(CModbusSocket::Write));

	//1.6 （数据位：9-10）起始地址，现在把地址这个int类型转换为两个byte(地址他说我差1，我就给他减掉1好了)
	values.push_back(StartAddress/256);
	values.push_back(StartAddress%256);

	//1.7  （数据位：11-12）寄存器数量（不可能超过255个）
	values.push_back(0);
	values.push_back((BYTE)iRegisterNum);

	//1.8 （数据位：13）发送数据的长度，跟前面保持不变
	values.push_back((BYTE)uchrtab.size());

	//2 发数据
	for (vector<BYTE>::iterator va = uchrtab.begin();va != uchrtab.end(); va++)
	{
		values.push_back(*va);
	}
#ifdef ROBOT_TEST
	char chsend[128];
	sprintf( chsend, "%s", "位置" );
	int size = send(sock,chsend,10,0);
#else 
	int size = send(sock,(char*)&values[0],(int)values.size(),0);
#endif
	if (size < 0)
	{
		goto ErrorWrite;
	}
	//这里要读取数据报告
	unsigned char ReadBufferData[256] = {0};
	size = recv(sock,(char *)ReadBufferData,256,0);

	if (size < 0)
	{
		goto ErrorWrite;
	}
	if (ReadBufferData[7] & 0x80)
	{
		//这里出现MODBUS异常
		m_iErrorNo = ReadBufferData[8];
		goto ErrorWrite;
	}
	//这里要分析返回报告——暂时先不加！！！！！！！！！！！！！！！！！！！！！
	//CloseConnect();
	LeaveCriticalSection(&m_criSection);
	return true;
ErrorWrite:
	m_iErrorNo = 0 - WSAGetLastError();
	//CloseConnect();
// 	{
// 		CString strErrorNo;
// 		strErrorNo.Format(L"ErrorWrite: %d\n",m_iErrorNo);
// 		//AfxWriteToCommand(strErrorNo);
// 	}
	LeaveCriticalSection(&m_criSection);
	return m_iErrorNo;
}

//ReadMultipleRegister需要传入的参数包括：要读取的寄存器个数（比如1,2,3），要读取的寄存器首地址（比如100）
// ResigterNum 寄存器数量 StartAddress为起始地址 data为返回数据，不能为空
int CModbusSocket::ReadMultipleRegister(int ResigterNum,int StartAddress,int * data)
{
	//我确实要读取的东西(byte永远是两个表示一个)
	//int errorcode = 0;
	EnterCriticalSection(&m_criSection);
	if (data == NULL)
	{
		return MB_ERROR_DATA_NULL;
	}

	if (m_bSockOn == false)
	{
		return m_iErrorNo;
	}

	//if ((errorcode = ConnectServer()) < 0)
	//{
	//	CloseConnect();
	//	return errorcode;
	//	//return MB_ERROR_SERVER;
	//}
	//定义长度不确定的数组sendData，每个数组元素都是byte类型的整数
	vector<unsigned char>* sendData = new vector<unsigned char>();

	//1 凑Header

	//1.1（数据位：1-2） 定义我是谁(Transaction Identifier),协议给了我2byte空间定义我是谁（我代号就是01，如果Modbus TCP Server接收成功，返回信息也要有这个代号）
	sendData->push_back(0);
	sendData->push_back(1);

	//1.2 （数据位：3-4）定义协议号，协议给了我2byte空间定义协议号（协议号就是00，表示这是MODBUS 协议）
	sendData->push_back(0);
	sendData->push_back(0);

	//1.3 （数据位：5-6）header还有几位？（对于读取来说，header后面定死了还有6位）
	sendData->push_back(0);
	sendData->push_back(6);

	//1.4 （数据位：7）只要补全一个0即可
	sendData->push_back(0);

	//1.5 （数据位：8）定义功能码（把读多个寄存器这个03码转换为byte类型数据）Function Code : 03 (Read Multiple Registers)
	sendData->push_back((UCHAR)CModbusSocket::Read);

	//1.6 （数据位：9-10）起始地址，现在把地址这个int类型转换为两个byte
	sendData->push_back(StartAddress/256);
	sendData->push_back(StartAddress%256);


	//1.7  （数据位：11-12）寄存器数量，默认要读取的寄存器数量不超过255个，所以高位不管了
	sendData->push_back(0);
	sendData->push_back((BYTE)ResigterNum);
	//2 发送查询命令
#ifdef ROBOT_TEST
	char chsend[128];
	sprintf( chsend, "%s", "位置" );
	int size = send(sock,chsend,10,0);
#else 
	int size = send(sock,(char*)&(*sendData)[0],(int)sendData->size(),0);
#endif
// 	{
// 		CString strsend;
// 		strsend.Format(L"ReadRegi_send: %d\n",size);
// 		//AfxWriteToCommand(strsend);
// 	}
	if (size < 0)
	{
		goto ErrorRead1;
	}

	//立即等待返回（由于是同步的操作，所以定义“我是谁”毫无意义，除非是异步的，返回值不知道要返回给谁）
	int ReadBuffer = 256;
	UCHAR ReadBufferData[256] = {0};
	UCHAR * TruelyDateByte = new unsigned char[ResigterNum*2];
	int* TruelyDateInt = new int[ResigterNum];
	//socket的Receive方法直接把读取到的数据返回给ReadBufferData
// 	{
// 		//AfxWriteToCommand(L"输出recv\n");
// 	}
	size = recv(sock,(char*)ReadBufferData,ReadBuffer,0);


// 	{
// 		CString strrecv;
// 		strrecv.Format(L"ReadRegi_recv: %d\n",size);
// 		//AfxWriteToCommand(strrecv);
// 	}

	if (size < 0)
	{
		goto ErrorRead;
	}
	if (ReadBufferData[7] & 0x80)
	{
		//这里出现MODBUS异常
		m_iErrorNo = ReadBufferData[8];
		goto ErrorRead;
	}
	//虽然一次把所有都读出来，但是并不是所有都是我需要的，我前面规定了我要读取几个，你给我返回几个就可以了
	//（0|1|0|0|0|9|0|3|6|）前面9位对我来说都是无意义的
	for (int i = 0; i < ResigterNum * 2; i++)
	{
		TruelyDateByte[i] = ReadBufferData[i + 9];
	}

	//然后byte再放回int
	for (int i = 0; i < ResigterNum;i++ )
	{
		data[i] = (TruelyDateByte[i * 2] * 256 + TruelyDateByte[i * 2 + 1]);
	}

	//尝试自己关闭，避免通讯错误
	//CloseConnect();
	delete[] TruelyDateInt;
	delete[] TruelyDateByte;
	delete sendData;
	LeaveCriticalSection(&m_criSection);
	//CloseConnect();
	return true;
ErrorRead:
	m_iErrorNo = 0 - WSAGetLastError();
// 	{
// 		CString strErrorNo;
// 		strErrorNo.Format(L"ErrorRead: %d\n",m_iErrorNo);
// 		//AfxWriteToCommand(strErrorNo);
// 	}
	delete[] TruelyDateInt;
	delete[] TruelyDateByte;
	LeaveCriticalSection(&m_criSection);
	return m_iErrorNo;
ErrorRead1:
	m_iErrorNo = 0 - WSAGetLastError();
// 	{
// 		CString strErrorNo;
// 		strErrorNo.Format(L"ErrorRead1: %d\n",m_iErrorNo);
// 		//AfxWriteToCommand(strErrorNo);
// 	}
	delete sendData;
	//CloseConnect();
	LeaveCriticalSection(&m_criSection);
	return m_iErrorNo;
}