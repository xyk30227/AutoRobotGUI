#include <Windows.h>
#include <tlhelp32.h>
#include <tchar.h>
#include <ShellAPI.h>
#include <atlstr.h>
bool StartPrcess(CString strProcessName)
{
	USES_CONVERSION;
	//char tszProcess[64] = { 0 };
	//char *tszProcess = T2A(strProcessName);
	//lstrcpy(tszProcess, _T( strProcessName.c_str()));
	//��������
	SHELLEXECUTEINFO shellInfo;
	memset(&shellInfo, 0, sizeof(SHELLEXECUTEINFO));
	shellInfo.cbSize = sizeof(SHELLEXECUTEINFO);
	shellInfo.fMask = NULL;
	shellInfo.hwnd = NULL;
	shellInfo.lpVerb = NULL;
	shellInfo.lpFile = strProcessName.AllocSysString();// (LPCWSTR)tszProcess;                      // ִ�еĳ�����(����·��)
	shellInfo.lpParameters = NULL;
	shellInfo.lpDirectory = NULL;
	shellInfo.nShow = SW_MINIMIZE;                      //SW_SHOWNORMAL ȫ����ʾ�������
	shellInfo.hInstApp = NULL;
	printf("�����Զ�������.... \n");
	ShellExecuteEx(&shellInfo);

	return true;
}
bool KillProcess(DWORD dwPid)
{
	printf("Kill����Pid = %d\n", dwPid); getchar();
	//�رս���
	HANDLE killHandle = OpenProcess(PROCESS_TERMINATE | PROCESS_QUERY_INFORMATION |   // Required by Alpha
		PROCESS_CREATE_THREAD |   // For CreateRemoteThread
		PROCESS_VM_OPERATION |   // For VirtualAllocEx/VirtualFreeEx
		PROCESS_VM_WRITE,             // For WriteProcessMemory);
		FALSE, dwPid);
	if (killHandle == NULL)
		return false;
	TerminateProcess(killHandle, 0);
	return true;
}
bool FindProcess(CString strProcessName, DWORD& nPid)
{
	USES_CONVERSION;
	//char tszProcess[64] = { 0 };
	char *tszProcess = T2A(strProcessName);
	//���ҽ���
	STARTUPINFO st;
	PROCESS_INFORMATION pi;
	PROCESSENTRY32 ps;
	HANDLE hSnapshot;
	memset(&st, 0, sizeof(STARTUPINFO));
	st.cb = sizeof(STARTUPINFO);
	memset(&ps, 0, sizeof(PROCESSENTRY32));
	ps.dwSize = sizeof(PROCESSENTRY32);
	memset(&pi, 0, sizeof(PROCESS_INFORMATION));
	// �������� 
	hSnapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (hSnapshot == INVALID_HANDLE_VALUE)
		return false;
	if (!Process32First(hSnapshot, &ps))
		return false;
	
	do {
		char *cha = W2A(ps.szExeFile);
		if (strcmp(cha, tszProcess) == 0)
		{
			//�ҵ��ƶ��ĳ���
			nPid = ps.th32ProcessID;
			CloseHandle(hSnapshot);
			printf("�ҵ�����: %s\n", tszProcess);
			return true;
			//getchar();
			//return dwPid;
		}
	} while (Process32Next(hSnapshot, &ps));
	CloseHandle(hSnapshot);
	return false;
}

void main()
{
	DWORD ID;
	CString strName = L"RobotCapturingParametersDesigner.exe";
	bool bRet = FindProcess(strName, ID);
	if (bRet)
	{

	}
	else
	{
		strName = L"E:\\" + strName;
		StartPrcess(strName);
	}
}