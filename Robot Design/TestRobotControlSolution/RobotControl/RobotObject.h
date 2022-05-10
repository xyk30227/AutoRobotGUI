//#pragma once


#ifndef ROBOT_OBJECT_H
#define ROBOT_OBJECT_H

#include "defconfig.h"
#include <atlstr.h>

using namespace std;
class CRobotObject
{
public:
	CRobotObject(void);
	~CRobotObject(void);

public:
	//为了便于操作，设置一个静态数组
	static double	m_dbarryPos[6];
	static double	m_dbarryAng[6];
	static int		m_iarryRobotState[6];

	//该变量用于CMD序号值
	static unsigned short m_uchrCmdNum;

	//读取信息状态标志
	bool m_bSockOn;

	//声明全局的速度和加速度控制条
	double m_dbSpeedRate;				//机器人速率
	double m_dbAccelerationRate;		//加速度

	//这里定义一个list，存储点位信息
	//list<UrRecPosAng> m_listPosRec;
	
	//声明m_listPosRec_iter为迭代器，用于位置保存
	//list<UrRecPosAng>::iterator m_listPosRec_iter;
	// 	//读取时声明m_listPosRec_iter_read为迭代器
	// 	list<UrRecPosAng>::iterator m_listPosRec_iter_read;
	UrRecPosAng m_sttPos;

	//这五个参数做成全局的会比较好用
	double m_dbBasicSpeed;			//基础速率
	double m_dbBasicAcceleration;	//基础加速率
	tstring m_strTarget_IP;			//目标主机IP	
	int m_iControl_Port;			//目标主机端口
	int m_iDataRefreshRate;			//数据刷新速率
	unsigned int m_iWaitCMDBack;

protected:	
	virtual void SetServerIPAndPort(tstring IP, int Port) = 0;
	//机器人速率设置
	virtual void SpeedChange(double _dbSpeed) = 0;
	virtual void AccelerationChange(double _dbAccleration) = 0;
	//直线移动
	virtual int MoveL(UrRecPosAng& UrPos, const tstring& axis, const int idir) = 0;
//	virtual int MoveL(const tstring& axis, const int idir) = 0;

	//将命令和位置信息转换为short类型
	virtual unsigned short * ToShortArry(const unsigned short usCmd, const UrRecPosAng& UrPos,  unsigned short * const data) = 0;

	//因为连接是同步的方法，会导致阻塞，所以把连接功能放到与发送一起执行
	virtual int Send_command(const unsigned char cmd, const unsigned short * const data) = 0;

	//将命令和位置信息转换为short类型
	virtual unsigned short * ToShortArry(const unsigned short usCmd, unsigned short * const data) = 0;


public:
	//初始化机器人
	virtual int InitRobot(std::wstring RobotIP) = 0;
	virtual int ConnectUR(void) = 0;
	virtual int CloseRobot() = 0;
	//这里定时抓取UR信息
	//刷新机器人，获取机器状态信息
	virtual int GetRobotInformation() = 0;
	//读取命令执行状态返回
	virtual int Read_commandBack() = 0;
	//取得六个坐标值的方法
	virtual int GetCurPositions(UrRecPosAng& pPosAng, enum _posAttr posAttr) = 0 ;
	//取得六个角度值的方法
	virtual int GetCurAngles(UrRecPosAng& pPosAng, enum _posAttr posAttr) = 0 ;
	//从手册可以知道，260-265的寄存器保存了当前机械手的状态信息
	virtual int GetCurRobotState(URState &pState) = 0;

	virtual int MoveToPos(const UrRecPosAng& pPosAng) = 0;	

	virtual void Return2Ori() = 0;

	virtual void ChangeMoveMode(int iMode) = 0;


	//改变示教器状态
	virtual bool LoadAndOpenProgram() = 0;


	// 进入示教模式
	virtual int ChangeModeToTech(void) = 0;
	// 结束示教模式
	virtual int EndTeachMode(void) = 0;
	// 暂停机器人
	virtual int PauseUR(void) = 0;
	// 关闭机器人
	virtual int StopUR(void) = 0;
	//初始化指令计数
	virtual void InitCmdNum() = 0;


protected:
	//取得六个坐标值的方法
	virtual int GetPositions() = 0;
	//取得六个角度值的方法
	virtual int GetAngles() = 0;
	//从手册可以知道，260-265的寄存器保存了当前机械手的状态信息
	virtual int GetRobotState() = 0;
};

#endif

