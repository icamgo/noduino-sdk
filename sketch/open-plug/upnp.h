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
#include "user_config.h"

#define UDP_SRV_IP		ipaddr_addr("239.255.255.250")
#define UDP_SRV_PORT	1900

#define SSDP_DISCOVER_RESP "HTTP/1.1 200 OK\r\n\
CACHE-CONTROL: max-age=86400\r\n\
DATE: Sat, 26 Nov 2016 04:56:29 GMT\r\n\
EXT:\r\n\
LOCATION: http://%s:%d/setup.xml\r\n\
OPT: \"http://schemas.upnp.org/upnp/1/0/\"; ns=01\r\n\
01-NLS: b9200ebb-736d-4b93-bf03-835149d13983\r\n\
SERVER: Unspecified, UPnP/1.0, Unspecified\r\n\
ST: urn:Belkin:device:**\r\n\
USN: uuid: %s::urn:Belkin:device:**\r\n\
X-User-Agent: redsonic\r\n\r\n"

int upnp_ssdp_start();
void upnp_ssdp_stop();
