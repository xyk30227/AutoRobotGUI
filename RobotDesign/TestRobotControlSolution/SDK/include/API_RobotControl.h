//���ߣ���԰С��
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
typedef void(*CallBackFunc_RobotReachPos)(void*);//�����˵�վʱ�Ļص�����

//�����ں�������ֵ
enum RC_STATUS
{
	RC_OK=1,
	RC_FAIL=-1,
};

//�������������
enum ROBOT_TYPE
{
	ROBOT_UR,			//UR������      E0505
	ROBOT_FANUC			//FANUC������   R0520
};

//����һ���࣬�����װ�˻����˿�������ӿ�
class API_ROBOT_CONTROL CRobotControl
{
private:
	UrRecPosAng m_PosAng2Move;
	UrRecPosAng m_curUrPosAng;

	void* m_pvParams;
	CallBackFunc_RobotReachPos m_pfOnRobotReachPos;

private:
	//��Ϊprivate���ԣ���ֹ�û����⹹�졢�̳С����������
	CRobotControl(void);
	~CRobotControl(void);

public:
	
	//���ӻ�����
	RC_STATUS ConnectRobot(int iRobotType, const char* strRobotIP);

	//�Ͽ�������
	RC_STATUS DisconnectRobot();

	//ȡ����������ֵ�ķ���
	RC_STATUS GetCurPositions(UrRecPosAng& pPosAng, enum _posAttr posAttr);

	//ȡ�������Ƕ�ֵ�ķ���	
	RC_STATUS GetCurAngles(UrRecPosAng& pPosAng, enum _posAttr posAttr);

	//���ֲ����֪����260-265�ļĴ��������˵�ǰ��е�ֵ�״̬��Ϣ
	RC_STATUS GetCurRobotState(URState &pState);

	//����������ƶ���ָ��λ�ã����û������Ѿ������˻ص���������ô���������ƶ���ָ��λ���Ժ󣬻ص���������������
	RC_STATUS MoveToPos(const UrRecPosAng& pPosAng, CallBackFunc_RobotReachPos pfOnRobotReachPos=NULL, void* pvParams=NULL);

private:
	//���ﶨʱץȡ��������Ϣ
	//ˢ�»����ˣ���ȡ����״̬��Ϣ
	int GetRobotInformation();

	//�ı�������ƶ���ʽ  //1 �����Ƕ��˶� ;0 ����λ���˶�
	void ChangeMoveMode(int iMode);

	//�ص��������ƶ�ʱ�����ظĺ���
	//virtual void OnMoveCallBack();

	//�Զ�����ʾ���������ܿ������ж�״̬
	bool LoadAndOpenProgram();

	void Return2Ori();
	
	//��ʼ��ָ�����
	void InitCmdNum();

private:	
	std::shared_ptr<class CRobotObject> m_pRobotObj;
	bool m_bWorking,m_bMoving;
	void EndMove();
 	static DWORD CallMoveBack(LPVOID lpData);
 	static DWORD FreshData(LPVOID lpData);

public:
	//ͨ���������ӿں����������ͷ���Ķ���ָ��
	static CRobotControl* CreateRobotControl(){return new CRobotControl();}
	static void ReleaseRobotControl(CRobotControl*& p){if(p) {delete p;p=NULL;}}
};

#endif


