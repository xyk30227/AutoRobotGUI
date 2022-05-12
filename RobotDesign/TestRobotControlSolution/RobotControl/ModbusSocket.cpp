/*
*	Copyright (c) ��Զ��ά����򣩿Ƽ����޹�˾�з���
*	All rights reserved
*	
*	�ļ����ƣ�ModbusSock.cpp
*	ժ    Ҫ��Modbusͨ��Э����ͷ�ļ�
*	
*	��ǰ�汾��0.1
*	��    �ߣ����
*	������ڣ�2016��3��21��
*	
*	ȡ���汾��
*	ԭ����  ��
*	������ڣ�
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
//����У���
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
//	AfxWriteToLog("����MODBUS���\n");
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
	InitializeCriticalSection(&m_criSection);  //ʹ��ǰ�����ʼ��
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
 //����ֻ�ǶϿ����ӣ����ǹر�SOCKET���
int CModbusSocket::ConnectServer()
{		
	//listMsg.push_back(L"Modbus Sock ���óɹ���");
	//��ͬ��������������
	//�ȳ�ʼ����ַ
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

	//�������socket�ĳ�ʱʱ��
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
//����ֻ�ǶϿ����ӣ����ǹر�SOCKET���
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

//WriteMultipleRegister������Ҫ����д��Ĵ������ַ������硰123|234|5555�������ֻ��һ�����÷ָ�������Ҫд��Ĵ����ĸ�����1,2,3��Ҫһһ��Ӧ��д�����ʼ��ַ
int CModbusSocket::WriteMultipleRegister(const unsigned int iRegisterNum,const int StartAddress, const unsigned short * const data)
{
	//��������ַ�����RegisterDivision���зָ�
	//����һ���б����ڴ洢����
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

	//�ѷָ�õ��ַ�������byte����(�����Ѿ�֪���˵���Ҫд�뼸���Ĵ�����Ҳ��֪�������鳤�ȣ���Ϊ�Ĵ����ָߵͣ������ֻҪд1���Ĵ��������Ҫ��2��byte���Դ�����)
	vector<unsigned char> uchrtab;
	for (unsigned int i = 0; i < iRegisterNum; i++)
	{
		uchrtab.push_back(data[i]/256);
		uchrtab.push_back(data[i]%256);
	}

	vector<unsigned char> values;

	//1 ��Header

	//1.1������λ��1-2�� ��������˭(Transaction Identifier),Э�������2byte�ռ䶨������˭���Ҵ��ž���01�����Modbus TCP Server���ճɹ���������ϢҲҪ��������ţ�
	values.push_back(0);
	values.push_back(1);

	//1.2 ������λ��3-4������Э��ţ�Э�������2byte�ռ䶨��Э��ţ�Э��ž���00����ʾ����MODBUS Э�飩
	values.push_back(0);
	values.push_back(0);

	//1.3 ������λ��5-6�����ڿ��ǽ�56��λ�յ�һ��
	values.push_back(unsigned char((uchrtab.size() + 7) / 256));
	values.push_back(unsigned char((uchrtab.size() + 7) % 256));


	//1.4 ������λ��7��ֻҪ��ȫһ��0����
	values.push_back(0);

	//1.5 ������λ��8�����幦���루��д����Ĵ������16��ת��Ϊbyte�������ݣ�Function Code : 16 (Write Multiple Register)
	values.push_back((BYTE)(CModbusSocket::Write));

	//1.6 ������λ��9-10����ʼ��ַ�����ڰѵ�ַ���int����ת��Ϊ����byte(��ַ��˵�Ҳ�1���Ҿ͸�������1����)
	values.push_back(StartAddress/256);
	values.push_back(StartAddress%256);

	//1.7  ������λ��11-12���Ĵ��������������ܳ���255����
	values.push_back(0);
	values.push_back((BYTE)iRegisterNum);

	//1.8 ������λ��13���������ݵĳ��ȣ���ǰ�汣�ֲ���
	values.push_back((BYTE)uchrtab.size());

	//2 ������
	for (vector<BYTE>::iterator va = uchrtab.begin();va != uchrtab.end(); va++)
	{
		values.push_back(*va);
	}
#ifdef ROBOT_TEST
	char chsend[128];
	sprintf( chsend, "%s", "λ��" );
	int size = send(sock,chsend,10,0);
#else 
	int size = send(sock,(char*)&values[0],(int)values.size(),0);
#endif
	if (size < 0)
	{
		goto ErrorWrite;
	}
	//����Ҫ��ȡ���ݱ���
	unsigned char ReadBufferData[256] = {0};
	size = recv(sock,(char *)ReadBufferData,256,0);

	if (size < 0)
	{
		goto ErrorWrite;
	}
	if (ReadBufferData[7] & 0x80)
	{
		//�������MODBUS�쳣
		m_iErrorNo = ReadBufferData[8];
		goto ErrorWrite;
	}
	//����Ҫ�������ر��桪����ʱ�Ȳ��ӣ�����������������������������������������
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

//ReadMultipleRegister��Ҫ����Ĳ���������Ҫ��ȡ�ļĴ�������������1,2,3����Ҫ��ȡ�ļĴ����׵�ַ������100��
// ResigterNum �Ĵ������� StartAddressΪ��ʼ��ַ dataΪ�������ݣ�����Ϊ��
int CModbusSocket::ReadMultipleRegister(int ResigterNum,int StartAddress,int * data)
{
	//��ȷʵҪ��ȡ�Ķ���(byte��Զ��������ʾһ��)
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
	//���峤�Ȳ�ȷ��������sendData��ÿ������Ԫ�ض���byte���͵�����
	vector<unsigned char>* sendData = new vector<unsigned char>();

	//1 ��Header

	//1.1������λ��1-2�� ��������˭(Transaction Identifier),Э�������2byte�ռ䶨������˭���Ҵ��ž���01�����Modbus TCP Server���ճɹ���������ϢҲҪ��������ţ�
	sendData->push_back(0);
	sendData->push_back(1);

	//1.2 ������λ��3-4������Э��ţ�Э�������2byte�ռ䶨��Э��ţ�Э��ž���00����ʾ����MODBUS Э�飩
	sendData->push_back(0);
	sendData->push_back(0);

	//1.3 ������λ��5-6��header���м�λ�������ڶ�ȡ��˵��header���涨���˻���6λ��
	sendData->push_back(0);
	sendData->push_back(6);

	//1.4 ������λ��7��ֻҪ��ȫһ��0����
	sendData->push_back(0);

	//1.5 ������λ��8�����幦���루�Ѷ�����Ĵ������03��ת��Ϊbyte�������ݣ�Function Code : 03 (Read Multiple Registers)
	sendData->push_back((UCHAR)CModbusSocket::Read);

	//1.6 ������λ��9-10����ʼ��ַ�����ڰѵ�ַ���int����ת��Ϊ����byte
	sendData->push_back(StartAddress/256);
	sendData->push_back(StartAddress%256);


	//1.7  ������λ��11-12���Ĵ���������Ĭ��Ҫ��ȡ�ļĴ�������������255�������Ը�λ������
	sendData->push_back(0);
	sendData->push_back((BYTE)ResigterNum);
	//2 ���Ͳ�ѯ����
#ifdef ROBOT_TEST
	char chsend[128];
	sprintf( chsend, "%s", "λ��" );
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

	//�����ȴ����أ�������ͬ���Ĳ��������Զ��塰����˭���������壬�������첽�ģ�����ֵ��֪��Ҫ���ظ�˭��
	int ReadBuffer = 256;
	UCHAR ReadBufferData[256] = {0};
	UCHAR * TruelyDateByte = new unsigned char[ResigterNum*2];
	int* TruelyDateInt = new int[ResigterNum];
	//socket��Receive����ֱ�ӰѶ�ȡ�������ݷ��ظ�ReadBufferData
// 	{
// 		//AfxWriteToCommand(L"���recv\n");
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
		//�������MODBUS�쳣
		m_iErrorNo = ReadBufferData[8];
		goto ErrorRead;
	}
	//��Ȼһ�ΰ����ж������������ǲ��������ж�������Ҫ�ģ���ǰ��涨����Ҫ��ȡ����������ҷ��ؼ����Ϳ�����
	//��0|1|0|0|0|9|0|3|6|��ǰ��9λ������˵�����������
	for (int i = 0; i < ResigterNum * 2; i++)
	{
		TruelyDateByte[i] = ReadBufferData[i + 9];
	}

	//Ȼ��byte�ٷŻ�int
	for (int i = 0; i < ResigterNum;i++ )
	{
		data[i] = (TruelyDateByte[i * 2] * 256 + TruelyDateByte[i * 2 + 1]);
	}

	//�����Լ��رգ�����ͨѶ����
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