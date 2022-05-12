//#include "StdAfx.h"
#include "RobotObject.h"

 unsigned short CRobotObject::m_uchrCmdNum = 0;
 double			CRobotObject::m_dbarryPos[6] = {0.0};
 int			CRobotObject::m_iarryRobotState[6] = {0};
 double			CRobotObject::m_dbarryAng[6] = {0.0};


CRobotObject::CRobotObject(void)
{
}


CRobotObject::~CRobotObject(void)
{
	Sleep(1);
}
