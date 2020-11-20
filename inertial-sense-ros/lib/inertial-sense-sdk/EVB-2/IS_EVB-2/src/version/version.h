#ifndef VERSION_H
#define VERSION_H

#include "repositoryInfo.h"
#include "buildInfo.h"
#include "user_board.h"

#define HARDWARE_VERSION_CHAR0  2
#define HARDWARE_VERSION_CHAR1  0
#define HARDWARE_VERSION_CHAR2  (g_hdw_detect)
#define HARDWARE_VERSION_CHAR3  0

#define FIRMWARE_VERSION_CHAR0  REPO_VERSION_MAJOR
#define FIRMWARE_VERSION_CHAR1  REPO_VERSION_MINOR
#define FIRMWARE_VERSION_CHAR2  REPO_VERSION_REVIS
#define FIRMWARE_VERSION_CHAR3  0                   // firmware specific revision


extern uint8_t g_hdw_detect;




#endif // VERSION_H
