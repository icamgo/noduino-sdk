/*
 *  Copyright (c) 2016 - 2025 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/
#ifndef __HTTPD_H__
#define __HTTPD_H__

#include "user_interface.h"

#include "lwip/ip_addr.h"
#include "mem.h"
#include "ets_sys.h"
#include "os_type.h"

#include "espconn.h"

// Output normal info
#define	INFO			1

// Output debug message
//#define	DEBUG			1

#ifdef INFO
#define HTTPD_INFO( format, ... ) os_printf( "HTTPD: "format, ## __VA_ARGS__ )
#else
#define HTTPD_INFO( format, ... )
#endif

#ifdef DEBUG
#define HTTPD_DEBUG( format, ... ) os_printf( "HTTPD: "format, ## __VA_ARGS__ )
#else
#define HTTPD_DEBUG( format, ... )
#endif

#define HTTP_OK_HDR	"HTTP/1.0 200 OK\r\n\
Server: lwIP/1.4.0\r\n\
Content-type: %s\r\n\
Content-Length: %d\r\n\
Connection: close\r\n\r\n%s\r\n"

typedef struct httpd {
	struct espconn *esp_conn;
	uint32_t port;
} httpd_t;

int httpd_start(httpd_t *d);
void httpd_stop(httpd_t *d);

#endif
