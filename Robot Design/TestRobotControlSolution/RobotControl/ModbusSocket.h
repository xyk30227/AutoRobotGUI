/*
*	Copyright (c) ��Զ��ά����򣩿Ƽ����޹�˾�з���
*	All rights reserved
*	
*	�ļ����ƣ�ModbusSock.h
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
����ʵ��������Ҫͨ��Connect�������в���Զ�������Ƿ����
*/

//ICMPͷ�ṹ
typedef struct _icmphdr
{
	BYTE   i_type;    //����
	BYTE   i_code;    //����
	USHORT i_cksum;   //У���
	USHORT i_id;      //��ʶ��	
	USHORT i_seq;     //���к�
	ULONG  timestamp; //ʱ���
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

	//���Ƕ���MODBUS�Ķ���д
	enum FunctionCode 
	{
		//Read Multiple Registers
		Read = 3,
		//Write Multiple Registers
		Write = 16
	} ;

private:
	SOCKET sock;
	//�о�̬���ԣ�Զ��IP��Զ�̶˿ڣ���ʱʱ�䣬��ʼ��ַ��
	tstring m_strRemoteIP;	//Զ������IP
	int m_iRemotePort;		//Զ�̶˿�
	int m_iSocketTimeOut;	//IP���ӳ�ʱ

// 	//��һ��Socket�����IPE����
// 	SOCKET sock;

	//��������ַ
	SOCKADDR_IN addrSrv ;
	//����ΪMODBUS���������ٽ���������ʱ��ֻ��һ������
	CRITICAL_SECTION m_criSection;
public:

	//WSA ��ʼ����־λ
	bool m_bWsaOn;

	//������Ϣ��¼
	static int m_iErrorNo;
	//sock���ӳɹ���־
	bool m_bSockOn;


	int InitModbusSocket(tstring UR_IP,DWORD UR_Port,int timeOut);
	//����MODBUS server ��IP
	int SetServerIP(const tstring& IP);
	//��������
	int ConnectServer(void);

	//����ֻ�ǶϿ����ӣ����ǹر�SOCKET���
	void CloseConnect(void);
	
	//WriteMultipleRegister������Ҫ����д��Ĵ������ַ������硰123|234|5555�������ֻ��һ�����÷ָ�������Ҫд��Ĵ����ĸ�����1,2,3��Ҫһһ��Ӧ��д�����ʼ��ַ
	int WriteMultipleRegister(const unsigned int iRegisterNum,const int StartAddress, const unsigned short * const data);

	//ReadMultipleRegister��Ҫ����Ĳ���������Ҫ��ȡ�ļĴ�������������1,2,3����Ҫ��ȡ�ļĴ����׵�ַ������100��
	// ResigterNum �Ĵ������� StartAddressΪ��ʼ��ַ dataΪ�������ݣ�����Ϊ��
	int ReadMultipleRegister(const int ResigterNum,const int StartAddress,int * const data);

	//ͨ��ping��������Ƿ���ͨ
	int PingToSever();
};

#endif
