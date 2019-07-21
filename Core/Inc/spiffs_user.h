#ifndef __spiffs_user_H
#define __spiffs_user_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
//#include "cmsis_os.h"
#include "spiffs.h"
#include "w25qxx.h"

extern spiffs fs;

int32_t sys_spiffs_format(void);
int32_t sys_spiffs_mount_coreflash(void);
void sys_spiffs_ls(void);
void test_spiffs(void);

#ifdef __cplusplus
}
#endif

#endif
