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
#ifndef __HTTP_SRV_H__
#define __HTTP_SRV_H__

#include "ets_sys.h"
#include "os_type.h"

#include "upnp.h"

#include "espconn.h"

#define HTTP_OK_HDR	"HTTP/1.0 200 OK\r\n\
Server: lwIP/1.4.0\r\n\
Content-type: %s\r\n\
Content-Length: %d\r\n\
Connection: close\r\n\r\n%s\r\n"


#define WEMO_SETUP_XML	"<?xml version=\"1.0\"?>\
<root>\
<device>\
<deviceType>%s</deviceType>\
<friendlyName>%s</friendlyName>\
<manufacturer>Belkin International Inc.</manufacturer>\
<manufacturerURL>http://www.noduino.org</manufacturerURL>\
<modelName>Noduino %s</modelName>\
<modelNumber>%s</modelNumber>\
<UDN>uuid:%s</UDN>\
<serialNumber>221517K0101769</serialNumber>\
<binaryState>0</binaryState>\
<serviceList>\
<service>\
<serviceType>urn:Belkin:service:basicevent:1</serviceType>\
<serviceId>urn:Belkin:serviceId:basicevent1</serviceId>\
<controlURL>/upnp/control/basicevent1</controlURL>\
<eventSubURL>/upnp/event/basicevent1</eventSubURL>\
<SCPDURL>/eventservice.xml</SCPDURL>\
</service>\
</serviceList>\
</device>\
</root>"

#define WEMO_SERVICE_XML "<?scpd xmlns=\"urn:Belkin:service-1-0\"?>\
<actionList>\
<action>\
<name>SetBinaryState</name>\
<argumentList>\
<argument>\
<retval/>\
<name>BinaryState</name>\
<relatedStateVariable>BinaryState</relatedStateVariable>\
<direction>in</direction>\
</argument>\
</argumentList>\
<serviceStateTable>\
<stateVariable sendEvents=\"yes\">\
<name>BinaryState</name>\
<dataType>Boolean</dataType>\
<defaultValue>0</defaultValue>\
</stateVariable>\
<stateVariable sendEvents=\"yes\">\
<name>level</name>\
<dataType>string</dataType>\
<defaultValue>0</defaultValue>\
</stateVariable>\
</serviceStateTable>\
</action>\
</scpd>\r\n\r\n"

#define	HUE_XML			\
"<root><device>\
<deviceType>urn:schemas-upnp-org:device:basic:1</deviceType>\
<friendlyName>%s</friendlyName>\
<manufacturer>Royal Philips Electronics</manufacturer>\
<manufacturerURL>http://www.noduino.org</manufacturerURL>\
<modelDescription>Noduino %s</modelDescription>\
<modelName>Philips hue bridge 2012</modelName>\
<modelNumber>%s</modelNumber>\
<serialNumber>001788102201</serialNumber>\
<UDN>uuid:2f402f80-da50-11e1-9b23-001788102201</UDN>\
</device></root>"

int httpd_start(upnp_dev_t *d);
void httpd_stop(upnp_dev_t *d);

#endif
