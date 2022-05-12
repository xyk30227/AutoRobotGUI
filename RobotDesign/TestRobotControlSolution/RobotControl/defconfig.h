

#ifndef _DEFCONFIG_H_
#define _DEFCONFIG_H_

#include <list>
#include <stdint.h>
//���������ɶ���
#define	RosNUM 6

//������Ĭ�Ͽ��Ʋ���
//modbus

#ifdef ROBOT_TEST
	#define REMOTEIP	L"127.0.0.1"
#else
	#define REMOTEIP	L"192.168.1.10"
#endif

#define REMOTEPORT	29999
#define AUTOCONNECT	true
//UR�˶����Ʋ���
#define	BASICSPEED	0.25
#define BASICACCELERATION	1.200
#define BASICREFRESHRATE	100

//��ȡ����ֵ��ʱ
#define WAITCMDBACK	1000 //Ĭ��Ϊ1s

//����������
#define SRVIP	REMOTEIP
//��������
#define POS_STEP	0.1
#define ANG_STEP	0.1
//static bool bOnWrite = false;


#include <list>
#include <xstring>
#include <string>
#ifdef _UNICODE
#define tstring wstring
#else
#define tstring string
#endif
//extern std::list<std::tstring> listMsg;


//�����˵��˶���λ��¼��Ϣ
#ifndef CONFIG_STRUCT
#define CONFIG_STRUCT
enum _posAttr
{
	processPos,
	recPos,
};

typedef struct sttUrRecPosAng
{
	double dbPos[6];
	double dbAng[6];
	double dbSpeedRate;				//����������
	double dbAccelerationRate;		//���ٶ�
	enum _posAttr posAttr;
	unsigned int uiRecNum;
	int    iMoveType;
	int		iMoveTime;
} UrRecPosAng,* pUrRecPosAng;
typedef struct  URState
{
	uint16_t	bOnCnt;
	uint16_t	bOnSafetyStop;
	uint16_t	bOnEmergencyStop;
	uint16_t	bOnTech;
	uint16_t	bOnPressPowerBtn;
	uint16_t	bOnStopWithSafetySignal;
}URState, *pURState;
#endif
/**
* �����˵�״̬��Ϣ
* �ܹ�6��״̬
*/
// #pragma pack(push)
// #pragma pack(2)
// #pragma pack(pop)

#endif

