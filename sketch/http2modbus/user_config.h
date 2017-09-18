#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "user_interface.h"

#include "httpd.h"

int str2hex(uint8_t *data, const char *hexstr, uint32_t len);
uint16_t crc16(const uint8_t *data, uint16_t len);

#endif
