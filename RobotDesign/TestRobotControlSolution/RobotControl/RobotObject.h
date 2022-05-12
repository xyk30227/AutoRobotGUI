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
	//Ϊ�˱��ڲ���������һ����̬����
	static double	m_dbarryPos[6];
	static double	m_dbarryAng[6];
	static int		m_iarryRobotState[6];

	//�ñ�������CMD���ֵ
	static unsigned short m_uchrCmdNum;

	//��ȡ��Ϣ״̬��־
	bool m_bSockOn;

	//����ȫ�ֵ��ٶȺͼ��ٶȿ�����
	double m_dbSpeedRate;				//����������
	double m_dbAccelerationRate;		//���ٶ�

	//���ﶨ��һ��list���洢��λ��Ϣ
	//list<UrRecPosAng> m_listPosRec;
	
	//����m_listPosRec_iterΪ������������λ�ñ���
	//list<UrRecPosAng>::iterator m_listPosRec_iter;
	// 	//��ȡʱ����m_listPosRec_iter_readΪ������
	// 	list<UrRecPosAng>::iterator m_listPosRec_iter_read;
	UrRecPosAng m_sttPos;

	//�������������ȫ�ֵĻ�ȽϺ���
	double m_dbBasicSpeed;			//��������
	double m_dbBasicAcceleration;	//����������
	tstring m_strTarget_IP;			//Ŀ������IP	
	int m_iControl_Port;			//Ŀ�������˿�
	int m_iDataRefreshRate;			//����ˢ������
	unsigned int m_iWaitCMDBack;

protected:	
	virtual void SetServerIPAndPort(tstring IP, int Port) = 0;
	//��������������
	virtual void SpeedChange(double _dbSpeed) = 0;
	virtual void AccelerationChange(double _dbAccleration) = 0;
	//ֱ���ƶ�
	virtual int MoveL(UrRecPosAng& UrPos, const tstring& axis, const int idir) = 0;
//	virtual int MoveL(const tstring& axis, const int idir) = 0;

	//�������λ����Ϣת��Ϊshort����
	virtual unsigned short * ToShortArry(const unsigned short usCmd, const UrRecPosAng& UrPos,  unsigned short * const data) = 0;

	//��Ϊ������ͬ���ķ������ᵼ�����������԰����ӹ��ܷŵ��뷢��һ��ִ��
	virtual int Send_command(const unsigned char cmd, const unsigned short * const data) = 0;

	//�������λ����Ϣת��Ϊshort����
	virtual unsigned short * ToShortArry(const unsigned short usCmd, unsigned short * const data) = 0;


public:
	//��ʼ��������
	virtual int InitRobot(std::wstring RobotIP) = 0;
	virtual int ConnectUR(void) = 0;
	virtual int CloseRobot() = 0;
	//���ﶨʱץȡUR��Ϣ
	//ˢ�»����ˣ���ȡ����״̬��Ϣ
	virtual int GetRobotInformation() = 0;
	//��ȡ����ִ��״̬����
	virtual int Read_commandBack() = 0;
	//ȡ����������ֵ�ķ���
	virtual int GetCurPositions(UrRecPosAng& pPosAng, enum _posAttr posAttr) = 0 ;
	//ȡ�������Ƕ�ֵ�ķ���
	virtual int GetCurAngles(UrRecPosAng& pPosAng, enum _posAttr posAttr) = 0 ;
	//���ֲ����֪����260-265�ļĴ��������˵�ǰ��е�ֵ�״̬��Ϣ
	virtual int GetCurRobotState(URState &pState) = 0;

	virtual int MoveToPos(const UrRecPosAng& pPosAng) = 0;	

	virtual void Return2Ori() = 0;

	virtual void ChangeMoveMode(int iMode) = 0;


	//�ı�ʾ����״̬
	virtual bool LoadAndOpenProgram() = 0;


	// ����ʾ��ģʽ
	virtual int ChangeModeToTech(void) = 0;
	// ����ʾ��ģʽ
	virtual int EndTeachMode(void) = 0;
	// ��ͣ������
	virtual int PauseUR(void) = 0;
	// �رջ�����
	virtual int StopUR(void) = 0;
	//��ʼ��ָ�����
	virtual void InitCmdNum() = 0;


protected:
	//ȡ����������ֵ�ķ���
	virtual int GetPositions() = 0;
	//ȡ�������Ƕ�ֵ�ķ���
	virtual int GetAngles() = 0;
	//���ֲ����֪����260-265�ļĴ��������˵�ǰ��е�ֵ�״̬��Ϣ
	virtual int GetRobotState() = 0;
};

#endif

