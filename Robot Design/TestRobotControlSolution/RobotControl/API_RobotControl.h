//作者：田园小徐
#ifndef API_ROBOT_CONTROL_H
#define API_ROBOT_CONTROL_H

#ifdef  ROBOT_CONTROL_CPP
#define API_ROBOT_CONTROL __declspec(dllexport)
#else
#define API_ROBOT_CONTROL __declspec(dllimport)
#endif

#include "defconfig.h"
#include <Windows.h>
#include <memory>
typedef void(*CallBackFunc_RobotReachPos)(void*);//机器人到站时的回调函数

//定义借口函数返回值
enum RC_STATUS
{
	RC_OK=1,
	RC_FAIL=-1,
};

//定义机器人类型
enum ROBOT_TYPE
{
	ROBOT_UR,			//UR机器人      E0505
	ROBOT_FANUC			//FANUC机器人   R0520
};

//定义一个类，该类封装了机器人控制软件接口
class API_ROBOT_CONTROL CRobotControl
{
private:
	UrRecPosAng m_PosAng2Move;
	UrRecPosAng m_curUrPosAng;

	void* m_pvParams;
	CallBackFunc_RobotReachPos m_pfOnRobotReachPos;

private:
	//设为private属性，禁止用户随意构造、继承、拷贝这个类
	CRobotControl(void);
	~CRobotControl(void);

public:
	
	//连接机器人
	RC_STATUS ConnectRobot(int iRobotType, const char* strRobotIP);

	//断开机器人
	RC_STATUS DisconnectRobot();

	//取得六个坐标值的方法
	RC_STATUS GetCurPositions(UrRecPosAng& pPosAng, enum _posAttr posAttr);

	//取得六个角度值的方法	
	RC_STATUS GetCurAngles(UrRecPosAng& pPosAng, enum _posAttr posAttr);

	//从手册可以知道，260-265的寄存器保存了当前机械手的状态信息
	RC_STATUS GetCurRobotState(URState &pState);

	//命令机器人移动至指定位置（若用户事先已经传入了回调函数，那么当机器人移动到指定位置以后，回调函数将被触发）
	RC_STATUS MoveToPos(const UrRecPosAng& pPosAng, CallBackFunc_RobotReachPos pfOnRobotReachPos=NULL, void* pvParams=NULL);

private:
	//这里定时抓取机器人信息
	//刷新机器人，获取机器状态信息
	int GetRobotInformation();

	//改变机器人移动方式  //1 代表按角度运动 ;0 代表按位置运动
	void ChangeMoveMode(int iMode);

	//回调函数，移动时请重载改函数
	//virtual void OnMoveCallBack();

	//自动开启示教器，不能开启的判断状态
	bool LoadAndOpenProgram();

	void Return2Ori();
	
	//初始化指令计数
	void InitCmdNum();

private:	
	std::shared_ptr<class CRobotObject> m_pRobotObj;
	bool m_bWorking,m_bMoving;
	void EndMove();
 	static DWORD CallMoveBack(LPVOID lpData);
 	static DWORD FreshData(LPVOID lpData);

public:
	//通过这两个接口函数创建和释放类的对象指针
	static CRobotControl* CreateRobotControl(){return new CRobotControl();}
	static void ReleaseRobotControl(CRobotControl*& p){if(p) {delete p;p=NULL;}}
};

#endif


