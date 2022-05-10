/*
*	Copyright (c) ��Զ��ά����򣩿Ƽ����޹�˾�з���
*	All rights reserved
*	
*	�ļ����ƣ�DashBoard.h
*	ժ    Ҫ��DashBoardͨ��Э����ͷ�ļ�
*	
*	��ǰ�汾��1.0
*	��    �ߣ�xuyukai
*	������ڣ�2016��12��15��
*	
*	ȡ���汾��
*	ԭ����  ��
*	������ڣ�
**/

//#pragma once
#include "defconfig.h"
//#include <winsock2.h> 


//#include "TYScan.h"

using namespace std;
/*
����ʵ��������Ҫͨ��Connect�������в���Զ�������Ƿ����
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
	SOCKET sock;            //�о�̬���ԣ�Զ��IP��Զ�̶˿ڣ���ʱʱ�䣬��ʼ��ַ��	
	tstring m_strRemoteIP;	//Զ������IP
	int m_iRemotePort;		//Զ�̶˿�
	int m_iSocketTimeOut;	//IP���ӳ�ʱ	
	SOCKADDR_IN addrSrv ;   //��������ַ	
	CRITICAL_SECTION m_criSection; //����ΪMODBUS���������ٽ���������ʱ��ֻ��һ������
public:	
	bool m_bWsaOn;			//WSA ��ʼ����־λ	
	static int m_iErrorNo;  //������Ϣ��¼	
	bool m_bSockOn;			//sock���ӳɹ���־

	//������InitDashBoard
	//���ܣ���ʼ��DashBoard
	//����: UR_IP ������IP  UR_Port �����˶˿� timeOut ��ʱ
	//���أ���ȷ���� 1  ���󷵻� -1	
	int InitDashBoard(tstring UR_IP,DWORD UR_Port,int timeOut);
	
	//������SetServerIP
	//���ܣ�����MODBUS server ��IP
	//����: UR_IP ������IP 
	//���أ���ȷ���� 1  ���󷵻� -1		
	int SetServerIP(const tstring& IP);

	//������ConnectServer
	//���ܣ���������
	//����: ��
	//���أ���ȷ���� 1  ���󷵻� -1		
	int ConnectServer(void);

	//������CloseConnect
	//���ܣ��Ͽ����ӣ����ǹر�SOCKET���
	//����: ��
	//���أ�
	void CloseConnect(void);

	//������WriteDashBoard
	//���ܣ�DashBoard д�빦�ܣ���Ҫ��һЩ������Ϣ��д��
	//����: const char* �����ַ�
	//���أ���ȷ���� 1  ���󷵻� -1	
	int WriteDashBoard(const char* writedata);


	//������ReadDashBoard
	//���ܣ�DashBoard ��ȡ���ܣ���Ҫ���ڽ���ʾ����״̬�Ĳ�ѯ
	//����: ��	
	//���أ�0.������� 1.play  2.pasued  3.stop  
	int ReadDashBoard();

	//������IsMessageMatch
	//���ܣ��ж�����ָ������ָ���Ƿ�ƥ��
	//����: indata �����ַ��� outdata �����˷����ַ���
	//���أ���ȷ���� true ���󷵻� false
	bool IsMessageMatch(const char *indata,const char *outdata);
};

