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
	//启动程序
	SHELLEXECUTEINFO shellInfo;
	memset(&shellInfo, 0, sizeof(SHELLEXECUTEINFO));
	shellInfo.cbSize = sizeof(SHELLEXECUTEINFO);
	shellInfo.fMask = NULL;
	shellInfo.hwnd = NULL;
	shellInfo.lpVerb = NULL;
	shellInfo.lpFile = strProcessName.AllocSysString();// (LPCWSTR)tszProcess;                      // 执行的程序名(绝对路径)
	shellInfo.lpParameters = NULL;
	shellInfo.lpDirectory = NULL;
	shellInfo.nShow = SW_MINIMIZE;                      //SW_SHOWNORMAL 全屏显示这个程序
	shellInfo.hInstApp = NULL;
	printf("程序自动重启中.... \n");
	ShellExecuteEx(&shellInfo);

	return true;
}
bool KillProcess(DWORD dwPid)
{
	printf("Kill进程Pid = %d\n", dwPid); getchar();
	//关闭进程
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
	//查找进程
	STARTUPINFO st;
	PROCESS_INFORMATION pi;
	PROCESSENTRY32 ps;
	HANDLE hSnapshot;
	memset(&st, 0, sizeof(STARTUPINFO));
	st.cb = sizeof(STARTUPINFO);
	memset(&ps, 0, sizeof(PROCESSENTRY32));
	ps.dwSize = sizeof(PROCESSENTRY32);
	memset(&pi, 0, sizeof(PROCESS_INFORMATION));
	// 遍历进程 
	hSnapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (hSnapshot == INVALID_HANDLE_VALUE)
		return false;
	if (!Process32First(hSnapshot, &ps))
		return false;
	
	do {
		char *cha = W2A(ps.szExeFile);
		if (strcmp(cha, tszProcess) == 0)
		{
			//找到制定的程序
			nPid = ps.th32ProcessID;
			CloseHandle(hSnapshot);
			printf("找到进程: %s\n", tszProcess);
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