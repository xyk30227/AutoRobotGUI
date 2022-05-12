#include <stdio.h>
#include <WinSock2.h>
#pragma comment(lib, "ws2_32.lib")
#include <windows.h>
void Recv(SOCKET param);
void Send(SOCKET param);
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

	SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (SOCKET_ERROR == clientSocket)
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

	int r = connect(clientSocket, (sockaddr*)&addr, sizeof addr);
	if (-1 == r)
	{
		printf("connect error%d\n", GetLastError());
		closesocket(clientSocket);
		WSACleanup();
		return;
	}
	printf("connect success!\n");


	CreateThread(NULL, NULL, (LPTHREAD_START_ROUTINE)Recv, (LPVOID)clientSocket, NULL, NULL);

	CreateThread(NULL, NULL, (LPTHREAD_START_ROUTINE)Send, (LPVOID)clientSocket, NULL, NULL);

	while (true)
	{
		int a = 1;
	}
}


void Recv(SOCKET param)
{
	char buff[1024];
	SOCKET sock = param;
	while (true)
	{
		int r = recv(sock, buff, 1023, NULL);
		if (r > 0)
		{
			buff[r] = 0;
			printf("from sever: %s\n", buff);
		}
	}
}
void Send(SOCKET param)
{
	char buff[1024];
	SOCKET sock = param;
	while (true)
	{
		//printf("send :");
		gets(buff);
		send(sock, buff, strlen(buff), NULL);
	}
}