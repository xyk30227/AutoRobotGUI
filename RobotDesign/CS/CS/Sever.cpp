#include <stdio.h>
#include <WinSock2.h>
#pragma comment(lib, "ws2_32.lib")
#include <windows.h>
#define CLIENT_MAX_NUM 100
SOCKET clientSocket[CLIENT_MAX_NUM];

void Recv(int i);
void Send(int i);
char charQR[1024];

void main()
{
	
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (HIBYTE(wsaData.wVersion) != 2 || LOBYTE(wsaData.wVersion) != 2)
	{
		printf("Version Error:%d!\n", GetLastError());
		return;
	}
	printf("Version success!\n");


	SOCKET serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (SOCKET_ERROR == serverSocket)
	{
		printf("init sock error:%d\n", GetLastError());
		WSACleanup();
		return;
	}
	printf("Init success!\n");

	SOCKADDR_IN addr = { 0 };

	addr.sin_family = AF_INET;
	addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");;
	addr.sin_port = htons(602);


	int r = bind(serverSocket, (sockaddr*)&addr, sizeof addr);
	if (-1 == r)
	{
		printf("bind error!\n", GetLastError());
		closesocket(serverSocket);
		WSACleanup();
		return;
	}
	printf("bind success!\n");

	r = listen(serverSocket, 10);
	if (-1 == r)
	{
		printf("listen error!\n", GetLastError());
		closesocket(serverSocket);
		WSACleanup();
		return;
	}
	printf("listen success!\n");

	SOCKADDR_IN cAddr = { 0 };
	int len = sizeof cAddr;
	for (int i = 0; i < CLIENT_MAX_NUM; i++)
	{
		clientSocket[i]= accept(serverSocket, (sockaddr*)&cAddr, &len);
		if (SOCKET_ERROR == clientSocket[i])
		{
			printf("accept error!\n", GetLastError());
			closesocket(serverSocket);
			WSACleanup();
			return;
		}
		printf("accept %d success: %s!\n", i + 1, inet_ntoa(cAddr.sin_addr));

		CreateThread(NULL, NULL, (LPTHREAD_START_ROUTINE)Recv, (LPVOID)i, NULL, NULL);

		CreateThread(NULL, NULL, (LPTHREAD_START_ROUTINE)Send, (LPVOID)i, NULL, NULL);
	}	

	closesocket(serverSocket);
	WSACleanup();
}

void Recv(int i)
{

	//�������ⷢ����
	char buff_sent_mes1[1024] = "ScanOver";		//������ 1.ɨ�����+floader ScanOver D:\Data\CTC000000001_100.asc
	char buff_sent_mes2[1024] = "";

	char buff_sent_mes3[1024] = "SentReport";	//�ͻ��� 2.������+ �����ʽ
	char buff_sent_mes4[1024] = "";

	//�������յ�����
	char buff_recv_mes1[1024] = "InspectOver";	//������ 1.��ʼ��� 2.���ͼ�ⱨ�� InspectOver CTC000000001_100 
	char buff_recv_mes2[1024] = "";

	char buff_recv_mes3[1024] = "StartScan";	//�ͻ��� 1.��ʼɨ�� + QR StartScan CTC000000001_100 
	char buff_recv_mes4[1024] = "";	

	char buff[1024];
	while (true)
	{
		int r = recv(clientSocket[i], buff, 1023, NULL);
		if (r > 0)
		{
			buff[r] = 0;
			printf("from client%d: %s\n", i + 1, buff);

			char tempcha[1024];
			int iIndex = 0;
			for (int j = 0; j < r; j++)
			{
				if (buff[j] == ' ')
				{
					iIndex = j;
				}
			}
			
			memcpy(tempcha, buff, sizeof(char)*iIndex);
			tempcha[iIndex] = 0;
			if (strcmp(tempcha, buff_recv_mes3) == 0)
			{
				memcpy(charQR, &buff[iIndex + 1], sizeof(char)*(r - iIndex - 1));
				charQR[r - iIndex - 1] = 0;
				
				
				printf("from client%d: QR:%s\n", i + 1, charQR);
				//ģ��ɨ�����
				printf("��ʼɨ����ȴ�......\n");
				Sleep(2000);
				printf("ɨ�����......\n");
				char temp[1024];
				sprintf(temp, "ScanOver D:\\Data\\%s.asc", charQR);
				//�㲥
				for (int j = 0; j < CLIENT_MAX_NUM; j++)
				{
					if (SOCKET_ERROR != clientSocket[j])
					{
						send(clientSocket[j], temp, strlen(temp), NULL);
					}
				}
			}
			else if (strcmp(tempcha, buff_recv_mes1) == 0)
			{
				memcpy(charQR, &buff[iIndex + 1], sizeof(char)*(r - iIndex - 1));
				charQR[r - iIndex - 1] = 0;


				printf("from client%d: QR:%s\n", i + 1, charQR);
				//ģ�������
				Sleep(1000);
				char temp[1024];
				sprintf(temp, "SentReport D:\\Data\\%s_report.txt", charQR);
				//�㲥
				for (int j = 0; j < CLIENT_MAX_NUM; j++)
				{
					if (SOCKET_ERROR != clientSocket[j])
					{
						send(clientSocket[j], temp, strlen(temp), NULL);
					}
				}
			}
		}
	}
}


void Send(int i)
{
	//�������ⷢ����
	char buff_sent_mes1[1024] = "ScanOver";		//������ 1.ɨ�����+floader 2.��ⱨ���յ�  
	char buff_sent_mes2[1024] = "";

	char buff_sent_mes3[1024] = "";				//�ͻ��� 2.������+ �����ʽ
	char buff_sent_mes4[1024] = "InSpectOver";

	//�������յ�����
	char buff_recv_mes1[1024] = "StartInspect";	//������ 1.��ʼ��� 2.���ͼ�ⱨ��
	char buff_recv_mes2[1024] = "SentInspect";

	char buff_recv_mes3[1024] = "StartScan";	//�ͻ��� 1.��ʼɨ�� + QR 2.  
	char buff_recv_mes4[1024] = "";

	char buff[1024];
}