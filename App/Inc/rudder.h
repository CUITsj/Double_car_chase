#ifndef __RUDDER_H__
#define __RUDDER_H__

#include "common.h"
#include "include.h"

#define S3010_FTM FTM0
#define S3010_CH FTM_CH0
#define S3010_HZ 50
#define S3010_MID 7900  //�����С����ɫ7900����7300

extern int S3010_DUTY;

void S3010_pid();

#endif