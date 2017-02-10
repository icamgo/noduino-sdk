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
#ifndef __SSDP_H__
#define __SSDP_H__

#define UDP_SRV_IP		ipaddr_addr("239.255.255.250")
#define UDP_SRV_PORT	1900

#define SSDP_RESP	\
"HTTP/1.1 200 OK\r\n\
EXT:\r\n"

#define	SSDP_NOTIFY			\
"NOTIFY * HTTP/1.1\r\n\
"HOST: 239.255.255.250:1900\r\n\
"NTS: ssdp:alive\r\n"

static const char *SSDP_PKT =
"%s"										// SSDP_RESP / SSDP_NOTIFY
"CACHE-CONTROL: max-age=86400\r\n"			// SSDP_INTERVAL
"SERVER: Noduino/1.0 UPNP/1.1 %s/%s\r\n"	// modelName/modelNumber
"USN: uuid:%s\r\n"							// uuid
"%s: %s\r\n"								// "NT" or "ST", deviceType
"LOCATION: http://%s:%d/setup.xml\r\n"		// localIP(), port
"%s\r\n"									// extra information
"\r\n";

#endif
