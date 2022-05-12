

#ifndef _DEFCONFIG_H_
#define _DEFCONFIG_H_

#include <list>
#include <stdint.h>
//机器人自由度数
#define	RosNUM 6

//机器人默认控制参数
//modbus

#ifdef ROBOT_TEST
	#define REMOTEIP	L"127.0.0.1"
#else
	#define REMOTEIP	L"192.168.1.10"
#endif

#define REMOTEPORT	29999
#define AUTOCONNECT	true
//UR运动控制参数
#define	BASICSPEED	0.25
#define BASICACCELERATION	1.200
#define BASICREFRESHRATE	100

//读取返回值延时
#define WAITCMDBACK	1000 //默认为1s

//服务器参数
#define SRVIP	REMOTEIP
//步进参数
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


//机器人的运动点位记录信息
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
	double dbSpeedRate;				//机器人速率
	double dbAccelerationRate;		//加速度
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
* 机器人的状态信息
* 总共6种状态
*/
// #pragma pack(push)
// #pragma pack(2)
// #pragma pack(pop)

#endif

