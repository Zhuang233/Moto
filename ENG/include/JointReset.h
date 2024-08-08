#ifndef __JOINT_RESET_H__
#define __JOINT_RESET_H__
#include "time.h"
#include <stdbool.h>
void reset_qs(void);
void reset_hy(void);
extern bool qs_inited;
extern bool hy_inited;
extern uint8_t roll_yaw_reseted;
void last_roll_init(void);
#endif
