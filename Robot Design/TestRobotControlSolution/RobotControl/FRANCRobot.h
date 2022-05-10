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
	//��������������
	virtual void SpeedChange(double _dbSpeed);
	virtual void AccelerationChange(double _dbAccleration);
	//ֱ���ƶ�
	virtual int MoveL(UrRecPosAng& UrPos, const tstring& axis, const int idir);
	//	virtual int MoveL(const tstring& axis, const int idir) ;

	//�������λ����Ϣת��Ϊshort����
	virtual unsigned short * ToShortArry(const unsigned short usCmd, const UrRecPosAng& UrPos, unsigned short * const data);

	//��Ϊ������ͬ���ķ������ᵼ�����������԰����ӹ��ܷŵ��뷢��һ��ִ��
	virtual int Send_command(const unsigned char cmd, const unsigned short * const data);

	//�������λ����Ϣת��Ϊshort����
	virtual unsigned short * ToShortArry(const unsigned short usCmd, unsigned short * const data);


public:
	//��ʼ��������
	virtual int InitRobot(std::wstring RobotIP);
	virtual int ConnectUR(void);
	virtual int CloseRobot();
	//���ﶨʱץȡUR��Ϣ
	//ˢ�»����ˣ���ȡ����״̬��Ϣ
	virtual int GetRobotInformation();
	//��ȡ����ִ��״̬����
	virtual int Read_commandBack();

	//ȡ����������ֵ�ķ���
	virtual int GetCurPositions(UrRecPosAng& pPosAng, enum _posAttr posAttr);
	//ȡ�������Ƕ�ֵ�ķ���
	virtual int GetCurAngles(UrRecPosAng& pPosAng, enum _posAttr posAttr);

	//���ֲ����֪����260-265�ļĴ��������˵�ǰ��е�ֵ�״̬��Ϣ
	virtual int GetCurRobotState(URState &pState);

	virtual int MoveToPos(const UrRecPosAng& pPosAng);

	virtual void Return2Ori();

	virtual bool LoadAndOpenProgram();

	virtual void ChangeMoveMode(int iMode);
	// ����ʾ��ģʽ
	virtual int  ChangeModeToTech(void);
	// ����ʾ��ģʽ
	virtual int  EndTeachMode(void);
	// ��ͣ������
	virtual int  PauseUR(void);
	// �رջ�����
	virtual int  StopUR(void);

	//��ʼ��ָ�����
	virtual void InitCmdNum();

protected:
	//ȡ����������ֵ�ķ���
	virtual int GetPositions();
	//ȡ�������Ƕ�ֵ�ķ���
	virtual int GetAngles();
	//���ֲ����֪����260-265�ļĴ��������˵�ǰ��е�ֵ�״̬��Ϣ
	virtual int GetRobotState();
private:
	CRITICAL_SECTION cs;
	//CDashBoard *m_pDashBoard;
	CModbusSocket *m_pModbus;
	
	//1 �����Ƕ��˶� ;0 ����λ���˶�
	int m_iMoveMode;
};

