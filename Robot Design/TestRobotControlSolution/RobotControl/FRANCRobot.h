//#pragma once


#include "robotobject.h"
#include "ModbusSocket.h"
class CFRANCRobot :
	public CRobotObject
{
public:
	CFRANCRobot(void);
	~CFRANCRobot(void);
protected:
	virtual void SetServerIPAndPort(tstring IP, int Port);
	//机器人速率设置
	virtual void SpeedChange(double _dbSpeed);
	virtual void AccelerationChange(double _dbAccleration);
	//直线移动
	virtual int MoveL(UrRecPosAng& UrPos, const tstring& axis, const int idir);
	//	virtual int MoveL(const tstring& axis, const int idir) ;

	//将命令和位置信息转换为short类型
	virtual unsigned short * ToShortArry(const unsigned short usCmd, const UrRecPosAng& UrPos, unsigned short * const data);

	//因为连接是同步的方法，会导致阻塞，所以把连接功能放到与发送一起执行
	virtual int Send_command(const unsigned char cmd, const unsigned short * const data);

	//将命令和位置信息转换为short类型
	virtual unsigned short * ToShortArry(const unsigned short usCmd, unsigned short * const data);


public:
	//初始化机器人
	virtual int InitRobot(std::wstring RobotIP);
	virtual int ConnectUR(void);
	virtual int CloseRobot();
	//这里定时抓取UR信息
	//刷新机器人，获取机器状态信息
	virtual int GetRobotInformation();
	//读取命令执行状态返回
	virtual int Read_commandBack();

	//取得六个坐标值的方法
	virtual int GetCurPositions(UrRecPosAng& pPosAng, enum _posAttr posAttr);
	//取得六个角度值的方法
	virtual int GetCurAngles(UrRecPosAng& pPosAng, enum _posAttr posAttr);

	//从手册可以知道，260-265的寄存器保存了当前机械手的状态信息
	virtual int GetCurRobotState(URState &pState);

	virtual int MoveToPos(const UrRecPosAng& pPosAng);

	virtual void Return2Ori();

	virtual bool LoadAndOpenProgram();

	virtual void ChangeMoveMode(int iMode);
	// 进入示教模式
	virtual int  ChangeModeToTech(void);
	// 结束示教模式
	virtual int  EndTeachMode(void);
	// 暂停机器人
	virtual int  PauseUR(void);
	// 关闭机器人
	virtual int  StopUR(void);

	//初始化指令计数
	virtual void InitCmdNum();

protected:
	//取得六个坐标值的方法
	virtual int GetPositions();
	//取得六个角度值的方法
	virtual int GetAngles();
	//从手册可以知道，260-265的寄存器保存了当前机械手的状态信息
	virtual int GetRobotState();
private:
	CRITICAL_SECTION cs;
	//CDashBoard *m_pDashBoard;
	CModbusSocket *m_pModbus;
	
	//1 代表按角度运动 ;0 代表按位置运动
	int m_iMoveMode;
};

